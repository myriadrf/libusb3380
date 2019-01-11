/*
 * Core functions for libusb3380
 * Copyright (c) 2017 Sergey Kostanbaev <sergey.kostanbaev@fairwaves.co>
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 */

#include "xtrx_port.h"

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <assert.h>
#include <signal.h>
#include <stdarg.h>

#include "libusb3380.h"
#include "usb3380.h"

typedef struct pcibar {
	uint32_t addr;
	uint32_t length;
} pcibar_t;

struct libusb3380_pcidev {
	libusb3380_context_t *ctx;
	uint16_t devid; /**< { bus | dev | func } */
	uint16_t vid;
	uint16_t did;

	uint8_t cfg_msi_numcap;
	uint8_t cfg_msi_addr;

	pcibar_t bars[6];
};

struct libusb3380_context {
	libusb_device_handle* handle;
	libusb_context* context;
	unsigned int default_to;

	uint32_t gpio_dirs;
	uint32_t mem_addr_msi;

	struct libusb3380_pcidev dev;
};

enum {
	MSI_DEF_ADDR = 0xfee02000,
	MSI_DEF_ADDR_HIGH = 0,

	MSI_DEF_DATA = 0x4080,
};


typedef enum libusb3380_pci_master_stage {
	STAGE_OUT,
	STAGE_IN
} libusb3380_pci_master_stage_t;


/** Internal queue element */
typedef struct libusb3380_data_qbase {
	union {
		libusb3380_as_base_t* pbase;
		libusb3380_qgpep_t *pgpep;
		libusb3380_qpci_master_t *ppcims;
		libusb3380_qmsi_int_t *pmsi;
	} blk;
} libusb3380_data_qbase_t;


struct libusb3380_async_manager;

typedef struct libusb3380_queue_header {
	libusb3380_data_qbase_t queue;

	union {
		pthread_mutex_t qmutex;
		unsigned gpep_idx[2];
	};
	struct libusb_transfer *transfer;
} libusb3380_queue_header_t;

enum {
	MAX_MSI_INTERRUPTS = 32,
};
typedef struct libusb3380_async_manager {
	/* currently we supports only one device */
	struct libusb3380_pcidev *dev;

	libusb3380_configuration_t cfg;
	unsigned total_gpeps;
	unsigned gpep_in_idx_st[LIBUSB3380_GPEP_COUNT];
	unsigned gpep_out_idx_st[LIBUSB3380_GPEP_COUNT];

	libusb3380_queue_header_t *q_gpep;
	libusb3380_qgpep_t *gpep;


	libusb3380_queue_header_t q_pciout;

	libusb3380_queue_header_t q_msi;

	libusb3380_queue_header_t q_csrout;

	/* Root complex master */
	libusb3380_qpci_master_t pciout;
	sem_t pciout_notify;

	/* MSI */
	libusb3380_qmsi_int_t msi;
	sem_t msi_notify;

	libusb3380_qcsr_t csr;
	sem_t csr_notify;

	/* IO thread + (monitoring) */
	pthread_t io_thread;

	/* allocated resources */
	unsigned msi_ints[MAX_MSI_INTERRUPTS];

	/* interrupt RCIN */
	uint32_t msi_data[4];

	on_msi_cb_t msi_cb;
	void* msi_param;

	bool stop;
} libusb3380_async_manager_t;

static void def_log(libusb3380_loglevel_t level,
					void* obj,
					const char* func,
					const char* file,
					int line,
					const char* message, ...) __attribute__ ((format (printf, 6, 7)));

void def_log(libusb3380_loglevel_t level,
			 void* obj,
			 const char* func,
			 const char* file,
			 int line,
			 const char* message, ...)
{
	(void)obj;

	const char* sevirity;
	switch (level) {
	case USB3380_LOG_ERROR:   sevirity = "[ERROR]"; break;
	case USB3380_LOG_WARNING: sevirity = "[WARN] "; break;
	case USB3380_LOG_INFO:    sevirity = "[INFO] "; break;
	case USB3380_LOG_DEBUG:   sevirity = "[DEBUG]"; break;
	case USB3380_LOG_DUMP:    sevirity = "[DUMP] "; break;
	}

	char tmp_buf[1024];

	va_list ap;
	va_start(ap, message);
	vsnprintf(tmp_buf, sizeof(tmp_buf), message, ap);
	va_end(ap);

	fprintf(stderr, "%s %-16s %s\n", sevirity, func, tmp_buf);
}

static usb3380_logfunc_t s_logfunc = def_log;
static void* s_logobj = NULL;
static libusb3380_loglevel_t s_loglevel = USB3380_LOG_INFO;

#define LOG(x, ...) do { \
	if (s_loglevel >= (x)) s_logfunc((x), s_logobj, __FUNCTION__, __FILE__, __LINE__, __VA_ARGS__); \
	} while(0)

#define LOG_ERR(...)   LOG(USB3380_LOG_ERROR,  __VA_ARGS__)
#define LOG_WARN(...)  LOG(USB3380_LOG_WARNING, __VA_ARGS__)
#define LOG_INFO(...)  LOG(USB3380_LOG_INFO, __VA_ARGS__)
#define LOG_DEBUG(...) LOG(USB3380_LOG_DEBUG, __VA_ARGS__)
#define LOG_DUMP(...)  LOG(USB3380_LOG_DUMP, __VA_ARGS__)

void usb3380_set_logfunc(usb3380_logfunc_t func, void *param)
{
	s_logobj = param;
	s_logfunc = func;
}

void usb3380_set_loglevel(libusb3380_loglevel_t level)
{
	s_loglevel = level;
}


static unsigned char convert_gpep_no(unsigned no)
{
	switch (no) {
	case LIBUSB3380_GPEP0: return EP_GPEP0;
	case LIBUSB3380_GPEP1: return EP_GPEP1;
	case LIBUSB3380_GPEP2: return EP_GPEP2;
	case LIBUSB3380_GPEP3: return EP_GPEP3;
	default: return 0xff;
	}
}

int usb3380_gpio_dir(libusb3380_context_t* ctx, uint8_t diroutbits)
{
	ctx->gpio_dirs = diroutbits & 0xf;

	uint32_t reg;
	usb3380_csr_mm_cfg_read(ctx, 0x6C0, &reg);

	LOG_DEBUG("USBPM = %08x", reg);

	reg |= (1<<11);

	return usb3380_csr_mm_cfg_write(ctx, 0x6C0, reg);
}

int usb3380_gpio_out(libusb3380_context_t* ctx, uint8_t out)
{
	uint32_t reg = (ctx->gpio_dirs << 4) | ((out) & 0xf);
	LOG_DEBUG("GPIO OUT: %d%d%d%d (%08x)",
			  (out & 8) ? 1 : 0,
			  (out & 4) ? 1 : 0,
			  (out & 2) ? 1 : 0,
			  (out & 1) ? 1 : 0,
			  reg);

	return usb3380_csr_mm_cfg_write(ctx, GPIOCTRL, reg);
}

int usb3380_gpio_in(libusb3380_context_t* ctx, uint8_t *in)
{
	uint32_t reg;
	int res = usb3380_csr_mm_cfg_read(ctx, GPIOCTRL, &reg);
	if (res) {
		return res;
	}

	LOG_DEBUG("GPIO IN: %08x", reg);

	*in = (reg & 0xf);
	return 0;
}


static int usb3380_int_queue_init(libusb3380_queue_header_t *q)
{
	int res;
	res = pthread_mutex_init(&q->qmutex, NULL);
	if (res)
		return -res;

	q->transfer = libusb_alloc_transfer(0);
	if (q->transfer == NULL)
		return -ENOMEM;

	q->queue.blk.pbase = NULL;
	return 0;
}


static int usb3380_int_queue_gpep_init(libusb3380_queue_header_t *q,
									   unsigned gpepno, unsigned idx)
{
	q->gpep_idx[0] = gpepno;
	q->gpep_idx[1] = idx;

	q->transfer = libusb_alloc_transfer(0);
	if (q->transfer == NULL)
		return -ENOMEM;

	q->queue.blk.pbase = NULL;
	return 0;
}

static void usb3380_int_queue_deinit(libusb3380_queue_header_t *q)
{
	pthread_mutex_destroy(&q->qmutex);
	libusb_free_transfer(q->transfer);
}

static void usb3380_int_queue_gpep_deinit(libusb3380_queue_header_t *q)
{
	libusb_free_transfer(q->transfer);
}


static void* usb3380_io_thread(void *arg)
{
	struct libusb3380_async_manager* mgr = (struct libusb3380_async_manager*)arg;
	int res = 0;

#if defined(__linux) || defined(__APPLE__)
	sigset_t set;

	pthread_setname_np(pthread_self(), "usb3380_io");

	sigfillset(&set);
	pthread_sigmask(SIG_SETMASK, &set, NULL);

	struct sched_param shed;
	shed.sched_priority = 2;

	res = pthread_setschedparam(pthread_self(), SCHED_FIFO, &shed);
	if (res) {
		LOG_WARN("IO thread: Unable to set realtime priority: error %d", res);
	}
#endif

	LOG_INFO("IO thread started");

	while (!mgr->stop) {
		struct timeval tv;
		tv.tv_sec = 1;
		tv.tv_usec = 0;
		res = libusb_handle_events_timeout(mgr->dev->ctx->context, &tv);
	}

	LOG_INFO("IO thread termitaed with result %d", res);
	return (void*)((intptr_t)res);
}


static void fill_base_in_cb(libusb3380_as_base_t* base,
							const struct libusb_transfer *transfer)
{
	switch (transfer->status) {
	case LIBUSB_TRANSFER_COMPLETED:
		if (transfer->actual_length == transfer->length)
			base->status = DQS_SUCCESS;
		else
			base->status = DQS_PARTIAL;
		break;

	case LIBUSB_TRANSFER_TIMED_OUT:
		base->status = DQS_TIMEOUT;
		break;

	case LIBUSB_TRANSFER_CANCELLED:
		base->status = DQS_CANCELLED;
		break;

	case LIBUSB_TRANSFER_STALL:
		/* TODO clear EP */
		LOG_ERR("GPEP 0x%02x ERROR: STALL", transfer->endpoint);
		base->status = DQS_ABORT;
		break;

	case LIBUSB_TRANSFER_NO_DEVICE:
		LOG_ERR("GPEP 0x%02x ERROR: NO DEVICE", transfer->endpoint);
		base->status = DQS_ABORT;
		break;

	case LIBUSB_TRANSFER_OVERFLOW:
		LOG_ERR("GPEP 0x%02x ERROR: OVERFLOW", transfer->endpoint);
		base->status = DQS_ABORT;
		break;

	default:
	case LIBUSB_TRANSFER_ERROR:
		LOG_ERR("GPEP 0x%02x ERROR: %d", transfer->endpoint, transfer->status);
		base->status = DQS_ABORT;
		break;
	}

	base->written = (unsigned)transfer->actual_length;
}

static void notify_pciout(libusb3380_async_manager_t* mgr)
{
	int res;
	fill_base_in_cb(&mgr->pciout.base, mgr->q_pciout.transfer);

	res = sem_post(&mgr->pciout_notify);
	assert(res == 0);
}

static LIBUSB_CALL void on_pciout_cb(struct libusb_transfer *transfer)
{
	libusb3380_async_manager_t* mgr = (libusb3380_async_manager_t*)transfer->user_data;
	libusb3380_qpci_master_t* pciout = &mgr->pciout;

	LOG_DUMP("on_pciout_cb addr=%x data=%x:%x STAT=%d!",
			 pciout->out.addr,
			 pciout->data[0],
			 pciout->data[1],
			 transfer->status);

	if (transfer->status == LIBUSB_TRANSFER_COMPLETED &&
			transfer->actual_length == transfer->length &&
			pciout->type == RC_PCIE_READ &&
			transfer->endpoint == (EP_PCIOUT | LIBUSB_ENDPOINT_OUT)) {

		// Submit PCIIN request
		transfer->endpoint = EP_PCIIN | LIBUSB_ENDPOINT_IN;
		transfer->length = pciout->base.size;

		int res = libusb_submit_transfer(transfer);
		if (!res) {
			return;
		}
		// Notify abortion
		transfer->status = LIBUSB_TRANSFER_ERROR;
	}

	// Notify waiting thread
	notify_pciout(mgr);
}

static void prepare_ptransfer_pciout(libusb3380_async_manager_t* mgr)
{
	libusb_fill_bulk_transfer(mgr->q_pciout.transfer,
							  mgr->dev->ctx->handle,
							  EP_PCIOUT | LIBUSB_ENDPOINT_OUT,
							  (uint8_t*)mgr->pciout.data,
							  8,
							  on_pciout_cb,
							  mgr,
							  1000);

	mgr->q_pciout.transfer->flags = 0;
}

static LIBUSB_CALL void on_rcin_cb(struct libusb_transfer *transfer)
{
	libusb3380_async_manager_t* mgr = (libusb3380_async_manager_t*)transfer->user_data;
	int res;
	int msi_num = -1;

	LOG_DUMP("on_rcin_cb addr=%08x:%08x:%08x:%08x!",
			mgr->msi_data[0], mgr->msi_data[1], mgr->msi_data[2],
			mgr->msi_data[3]);

	if (transfer->status == LIBUSB_TRANSFER_COMPLETED) {
		msi_num = mgr->msi_data[3] - MSI_DEF_DATA;
		if (msi_num >= 0 && msi_num < 32) {
			mgr->msi_ints[msi_num]++;
		} else {
			LOG_DUMP("on_rcin_cb got unexpected msi %08x:%08x:%08x:%08x!",
					mgr->msi_data[0], mgr->msi_data[1], mgr->msi_data[2],
					mgr->msi_data[3]);
		}
	}

	// TODO: determine interrupt queue based on msi_num received

	fill_base_in_cb(&mgr->msi.base, mgr->q_msi.transfer);
	res = sem_post(&mgr->msi_notify);
	assert(res == 0);
}

static void prepare_ptransfer_rcin(libusb3380_async_manager_t* mgr,
								   libusb_transfer_cb_fn callback,
								   unsigned timeout)
{
	libusb_fill_interrupt_transfer(mgr->q_msi.transfer,
								   mgr->dev->ctx->handle,
								   EP_RCIN | LIBUSB_ENDPOINT_IN,
								   (uint8_t*)mgr->msi_data,
								   16,
								   callback,
								   mgr,
								   timeout);
	mgr->q_msi.transfer->flags = 0;
}

static int async_pciout_post(libusb3380_async_manager_t* mgr,
							 libusb3380_pci_tlp_type_t type,
							 uint32_t addr,
							 uint32_t *data,
							 unsigned dw_count)
{
	int res;
	libusb3380_qpci_master_t* pciout = &mgr->pciout;

	res = pthread_mutex_lock(&mgr->q_pciout.qmutex);
	if (res) {
		// TODO CHECKME FIXME
		return res;
	}

restart:
	pciout->base.size = dw_count * 4;
	pciout->base.timeout = 1000;
	pciout->base.status = DQS_ABORT;
	pciout->base.written = 0;

	pciout->out.ctrl = 0xf | PCIMSTCTL_CMD_MEMORY | PCIMSTCTL_MASTER_START
			| (dw_count << PCIMSTCTL_PCIE_DW_LEN_OFF_BITS);
	if (type == RC_PCIE_READ) {
		pciout->out.ctrl |= PCIMSTCTL_PCIE_READ;
	} else {
		memcpy(pciout->out.data, data, dw_count * 4);

		pciout->out.ctrl |= PCIMSTCTL_PCIE_WRITE;
	}
	pciout->out.addr = addr;
	pciout->type = type;

	mgr->q_pciout.transfer->endpoint = EP_PCIOUT | LIBUSB_ENDPOINT_OUT;
	mgr->q_pciout.transfer->length = (type == RC_PCIE_READ) ? 8 : (int)(8 + dw_count * 4);

	res = libusb_submit_transfer(mgr->q_pciout.transfer);
	if (res) {
		goto failed_in_lock; // TODO error code
	}

	// Wait for transaction finalization
	res = sem_wait(&mgr->pciout_notify);
	if (res) {
		goto failed_in_lock;
	}

	if (pciout->base.status == DQS_PARTIAL) {
		if (type == RC_PCIE_READ) {
			LOG_WARN("Restart read transaction! addr=%x size=%d", addr,
					 pciout->base.size);
			goto restart;
		} else {
			LOG_WARN("Writr failed! addr=%x", addr);
		}
	}

	if (pciout->base.status == DQS_SUCCESS && type == RC_PCIE_READ) {
		memcpy(data, pciout->data, dw_count * 4);
	}

	if (pciout->base.status == DQS_TIMEOUT)
		res = -ETIMEDOUT;
	else if (pciout->base.status == DQS_ABORT)
		res = -EIO;
	else if (pciout->base.status != DQS_SUCCESS)
		res = -EFAULT;

failed_in_lock:
	pthread_mutex_unlock(&mgr->q_pciout.qmutex);
	return res;
}

int usb3380_async_pci_write32(struct libusb3380_async_manager* mgr,
							  uint32_t addr,
							  const uint32_t *data,
							  unsigned dw_count)
{
	return async_pciout_post(mgr, RC_PCIE_WRITE, addr, (uint32_t*)data,
							 dw_count);
}

int usb3380_async_pci_read32(struct libusb3380_async_manager* mgr,
							  uint32_t addr,
							  uint32_t *data,
							  unsigned dw_count)
{
	return async_pciout_post(mgr, RC_PCIE_READ, addr, data, dw_count);
}

int usb3380_async_await_msi(struct libusb3380_async_manager* mgr,
							unsigned num)
{
	int res;
	libusb3380_qmsi_int_t* rcin = &mgr->msi;
	res = pthread_mutex_lock(&mgr->q_msi.qmutex);
	if (res) {
		// TODO CHECKME FIXME
		return res;
	}

	prepare_ptransfer_rcin(mgr, on_rcin_cb, 100);
	res = libusb_submit_transfer(mgr->q_msi.transfer);
	if (res) {
		goto failed_in_lock; // TODO error code
	}

	// Wait for transaction finalization
	res = sem_wait(&mgr->msi_notify);
	if (res) {
		goto failed_in_lock;
	}

	if (rcin->base.status == DQS_TIMEOUT)
		res = -ETIMEDOUT;
	else if (rcin->base.status == DQS_ABORT)
		res = -EIO;

failed_in_lock:
	pthread_mutex_unlock(&mgr->q_msi.qmutex);
	LOG_DUMP("usb3380_async_await_msi failed!");
	return res;
}

static int libusb_to_errno(int libusberr)
{
	switch (libusberr) {
	case LIBUSB_SUCCESS:
		return 0;

	case LIBUSB_ERROR_IO:
		return -EIO;

	case LIBUSB_ERROR_INVALID_PARAM:
		return -EINVAL;

	case LIBUSB_ERROR_ACCESS:
		return -EPERM;

	case LIBUSB_ERROR_NO_DEVICE:
		return -ENODEV;

	case LIBUSB_ERROR_NOT_FOUND:
		return -ENXIO;

	case LIBUSB_ERROR_BUSY:
		return -EBUSY;

	case LIBUSB_ERROR_TIMEOUT:
		return -ETIMEDOUT;

	case LIBUSB_ERROR_OVERFLOW:
		return -EOVERFLOW;

	case LIBUSB_ERROR_PIPE:
		return -EPIPE;

	case LIBUSB_ERROR_INTERRUPTED:
		return -EINTR;

	case LIBUSB_ERROR_NO_MEM:
		return -ENOMEM;

	case LIBUSB_ERROR_NOT_SUPPORTED:
		return -EOPNOTSUPP;

	case LIBUSB_ERROR_OTHER:
		return -ESRCH; //TODO find better;
	};
	return -EFAULT;
}

static int gpep_transfer_post(libusb3380_queue_header_t* q,
								 libusb3380_qgpep_t *d)
{
	q->transfer->length = d->base.size;
	q->transfer->buffer = d->pdata;

	return libusb_to_errno(libusb_submit_transfer(q->transfer));
}

static LIBUSB_CALL void on_gpep_cb(struct libusb_transfer *transfer)
{
	libusb3380_queue_header_t* qh = (libusb3380_queue_header_t*)transfer->user_data;

	LOG_DUMP("TransferCB EP:%02x [%d;%d] done", transfer->endpoint,
			 qh->gpep_idx[0], qh->gpep_idx[1]);

	fill_base_in_cb(qh->queue.blk.pbase, qh->transfer);
	qh->queue.blk.pgpep->cb_done(qh->queue.blk.pgpep, qh->gpep_idx[0], qh->gpep_idx[1]);
}

static void prepare_ptransfer_gpep(libusb3380_async_manager_t* mgr,
								   libusb3380_queue_header_t* hdr,
								   libusb3380_qgpep_t* queue,
								   unsigned char ep_no,
								   unsigned int timeout)
{
	hdr->queue.blk.pgpep = queue;

	libusb_fill_bulk_transfer(hdr->transfer,
							  mgr->dev->ctx->handle,
							  ep_no,
							  (uint8_t*)NULL,
							  0,
							  on_gpep_cb,
							  hdr,
							  timeout);

	hdr->transfer->flags = 0;
}

int usb3380_async_gpep_out_post(struct libusb3380_async_manager* mgr,
								libusb3380_gpep_t ep_no,
								unsigned idx,
								const uint8_t* data, unsigned size,
								on_gpep_cb_t cb, void* param)
{
	if (ep_no > LIBUSB3380_GPEP3 || ep_no < LIBUSB3380_GPEP0)
		return -EINVAL;
	if (mgr->cfg.gp_out_cnts[ep_no] <= idx)
		return -EINVAL;

	unsigned gidx = mgr->gpep_out_idx_st[ep_no] + idx;
	libusb3380_qgpep_t *gpep_out = &mgr->gpep[gidx];

	// TODO check multi thread access
	gpep_out->base.size = size;
	gpep_out->pdata = (uint8_t*)data;
	gpep_out->cb_done = cb;
	gpep_out->param = param;

	return gpep_transfer_post(&mgr->q_gpep[gidx], gpep_out);
}

int usb3380_async_gpep_in_post(struct libusb3380_async_manager* mgr,
							   libusb3380_gpep_t ep_no,
							   unsigned idx,
							   uint8_t* data, unsigned size,
							   on_gpep_cb_t cb, void* param)
{
	if (ep_no > LIBUSB3380_GPEP3 || ep_no < LIBUSB3380_GPEP0)
		return -EINVAL;
	if (mgr->cfg.gp_in_cnts[ep_no] <= idx)
		return -EINVAL;

	unsigned gidx = mgr->gpep_in_idx_st[ep_no] + idx;
	libusb3380_qgpep_t *gpep_in = &mgr->gpep[gidx];

	// TODO check multi thread access
	gpep_in->base.size = size;
	gpep_in->pdata = data;
	gpep_in->cb_done = cb;
	gpep_in->param = param;

	return gpep_transfer_post(&mgr->q_gpep[gidx], gpep_in);
}

int usb3380_async_gpep_cancel(struct libusb3380_async_manager* mgr,
							  bool ep_in,
							  libusb3380_gpep_t gpep,
							  unsigned idx)
{
	if (gpep > LIBUSB3380_GPEP3 || gpep < LIBUSB3380_GPEP0)
		return -EINVAL;

	unsigned gidx;
	if (ep_in) {
		if (mgr->cfg.gp_in_cnts[gpep] <= idx)
			return -EINVAL;
		gidx = mgr->gpep_in_idx_st[gpep] + idx;
	} else {
		if (mgr->cfg.gp_out_cnts[gpep] <= idx)
			return -EINVAL;
		gidx = mgr->gpep_out_idx_st[gpep] + idx;
	}

	return libusb_to_errno(libusb_cancel_transfer(mgr->q_gpep[gidx].transfer));
}

int usb3380_async_set_gpep_timeout(struct libusb3380_async_manager* mgr,
								   bool ep_in, libusb3380_gpep_t gpep,
								   unsigned idx,
								   unsigned to_ms)
{
	if (gpep > LIBUSB3380_GPEP3 || gpep < LIBUSB3380_GPEP0)
		return -EINVAL;

	unsigned gidx;
	if (ep_in) {
		if (mgr->cfg.gp_in_cnts[gpep] <= idx)
			return -EINVAL;
		gidx = mgr->gpep_in_idx_st[gpep] + idx;
	} else {
		if (mgr->cfg.gp_out_cnts[gpep] <= idx)
			return -EINVAL;
		gidx = mgr->gpep_out_idx_st[gpep] + idx;
	}
	mgr->q_gpep[gidx].transfer->timeout = to_ms;
	return 0;
}

int usb3380_async_start(struct libusb3380_pcidev *dev,
						const struct libusb3380_configuration *configuration,
						struct libusb3380_async_manager **out)
{
	libusb3380_async_manager_t* mgr;
	int res;
	const unsigned def_timeout = 1000;
	unsigned idx;

	mgr = (libusb3380_async_manager_t*)malloc(sizeof(libusb3380_async_manager_t));
	if (!mgr)
		return -ENOMEM;
	memset(mgr, 0, sizeof(*mgr));

	mgr->dev = dev;
	mgr->stop = false;
	mgr->cfg = *configuration;
	for (unsigned k = 0; k < LIBUSB3380_GPEP_COUNT; k++) {
		mgr->total_gpeps += configuration->gp_in_cnts[k];
		mgr->total_gpeps += configuration->gp_out_cnts[k];
	}

	if (mgr->total_gpeps > 0) {
		mgr->q_gpep = (libusb3380_queue_header_t*)malloc(sizeof(libusb3380_queue_header_t) * mgr->total_gpeps);
		if (mgr->q_gpep == NULL) {
			res = -ENOMEM;
			goto q_gpep_alloc_fail;
		}
		mgr->gpep = (libusb3380_qgpep_t*)malloc(sizeof(libusb3380_qgpep_t) * mgr->total_gpeps);
		if (mgr->gpep == NULL) {
			res = -ENOMEM;
			goto gpep_alloc_fail;
		}
		memset(mgr->q_gpep, 0, sizeof(libusb3380_queue_header_t) * mgr->total_gpeps);
		memset(mgr->gpep, 0, sizeof(libusb3380_qgpep_t) * mgr->total_gpeps);

		idx = 0;
		for (unsigned l = 0; l < LIBUSB3380_GPEP_COUNT; l++) {
			mgr->gpep_in_idx_st[l] = idx;
			for (unsigned m = 0; m < mgr->cfg.gp_in_cnts[l]; m++, idx++) {
				res = usb3380_int_queue_gpep_init(&mgr->q_gpep[idx], l, m);
				if (res)
					goto failed_pciout;
				prepare_ptransfer_gpep(mgr,
									   &mgr->q_gpep[idx],
									   &mgr->gpep[idx],
									   convert_gpep_no(l) | LIBUSB_ENDPOINT_IN,
									   def_timeout);
			}
		}
		for (unsigned l = LIBUSB3380_GPEP_COUNT; l < 2 * LIBUSB3380_GPEP_COUNT; l++) {
			mgr->gpep_out_idx_st[l - LIBUSB3380_GPEP_COUNT] = idx;
			for (unsigned m = 0; m < mgr->cfg.gp_out_cnts[l - LIBUSB3380_GPEP_COUNT]; m++, idx++) {
				res = usb3380_int_queue_gpep_init(&mgr->q_gpep[idx], l, m);
				if (res)
					goto failed_pciout;
				prepare_ptransfer_gpep(mgr,
									   &mgr->q_gpep[idx],
									   &mgr->gpep[idx],
									   convert_gpep_no(l - LIBUSB3380_GPEP_COUNT) | LIBUSB_ENDPOINT_OUT,
									   def_timeout);
			}
		}
		assert(mgr->total_gpeps == idx);
	}

	res = usb3380_int_queue_init(&mgr->q_pciout);
	if (res)
		goto failed_pciout;

	res = usb3380_int_queue_init(&mgr->q_msi);
	if (res)
		goto failed_msi;

	prepare_ptransfer_pciout(mgr);

	res = sem_init(&mgr->pciout_notify, 0, 0);
	if (res) {
		res = -errno;
		goto failed_sem_pci;
	}
	res = sem_init(&mgr->msi_notify, 0, 0);
	if (res) {
		res = -errno;
		goto failed_sem_rc;
	}

	res = pthread_create(&mgr->io_thread, NULL, usb3380_io_thread, mgr);
	if (res) {
		res = -res;
		goto failed_thread;
	}

	*out = mgr;
	return 0;

failed_thread:
	sem_destroy(&mgr->msi_notify);
failed_sem_rc:
	sem_destroy(&mgr->pciout_notify);
failed_sem_pci:
	usb3380_int_queue_deinit(&mgr->q_msi);
failed_msi:
	usb3380_int_queue_deinit(&mgr->q_pciout);
failed_pciout:
	for (idx = 0; idx < mgr->total_gpeps; idx++) {
		if (mgr->q_gpep[idx].transfer) {
			usb3380_int_queue_gpep_deinit(&mgr->q_gpep[idx]);
		}
	}
	free(mgr->q_gpep);
gpep_alloc_fail:
	free(mgr->gpep);
q_gpep_alloc_fail:
	free(mgr);
	return res;
}

int usb3380_async_stop(struct libusb3380_async_manager* mgr)
{
	mgr->stop = true;
	pthread_join(mgr->io_thread, NULL);

	sem_destroy(&mgr->msi_notify);
	sem_destroy(&mgr->pciout_notify);
	usb3380_int_queue_deinit(&mgr->q_msi);
	usb3380_int_queue_deinit(&mgr->q_pciout);

	for (unsigned idx = 0; idx < mgr->total_gpeps; idx++) {
		usb3380_int_queue_gpep_deinit(&mgr->q_gpep[idx]);
	}
	free(mgr->q_gpep);
	free(mgr->gpep);

	free(mgr);
	return 0;
}


static int usb3380_int_csrout_sync(libusb3380_context_t* ctx, uint32_t csrctl,
								   uint32_t csrdata)
{
	CSROUT_ep_t pkt = { csrctl, csrdata };
	int written = 0;
	int res = libusb_bulk_transfer(ctx->handle,
								   EP_CSROUT | LIBUSB_ENDPOINT_OUT,
								   (unsigned char*)&pkt,
								   (csrctl & CSR_READ) ? 4 : 8,
								   &written,
								   ctx->default_to);
	return res;
}

static int usb3380_int_csrin_sync(libusb3380_context_t* ctx, uint32_t* csrdata)
{
	int written = 0;
	int res = libusb_bulk_transfer(ctx->handle,
								   EP_CSRIN | LIBUSB_ENDPOINT_IN,
								   (unsigned char*)csrdata,
								   4,
								   &written,
								   ctx->default_to);
	return res;
}

static int usb3380_int_csr_cfg_read(libusb3380_context_t* ctx, uint32_t ctrl,
									uint32_t* data)
{
	int res;

	res = usb3380_int_csrout_sync(ctx, ctrl, 0);
	if (res) {
		LOG_ERR("Unable to usb3380_int_csrout_sync!");
		return res;
	}

	res = usb3380_int_csrin_sync(ctx, data);
	if (res) {
		LOG_ERR("Unable to usb3380_int_csrin_sync!");
		return res;
	}

	return 0;
}

int usb3380_csr_pcie_cfg_read(libusb3380_context_t* ctx, uint32_t addr,
							  uint32_t* data)
{
	return usb3380_int_csr_cfg_read(ctx, CSR_READ | CSR_SS_PCIE_CFG |
									CSR_START | (addr << CSR_ADDR_OFF_BITS),
									data);
}

int usb3380_csr_pcie_cfg_write(libusb3380_context_t* ctx, uint32_t addr,
							   uint32_t data)
{
	return usb3380_int_csrout_sync(ctx, CSR_WRITE | CSR_BYTE_EN_ALL |
								   CSR_SS_PCIE_CFG | CSR_START |
								   (addr << CSR_ADDR_OFF_BITS), data);
}

int usb3380_csr_mm_cfg_read(libusb3380_context_t* ctx, uint32_t addr,
							uint32_t* data)
{
	return usb3380_int_csr_cfg_read(ctx, CSR_READ | CSR_SS_MM_CFG | CSR_START |
									(addr << CSR_ADDR_OFF_BITS), data);
}

int usb3380_csr_mm_cfg_write(libusb3380_context_t* ctx, uint32_t addr,
							 uint32_t data)
{
	return usb3380_int_csrout_sync(ctx, CSR_WRITE | CSR_BYTE_EN_ALL |
								   CSR_SS_MM_CFG | CSR_START |
								   (addr << CSR_ADDR_OFF_BITS), data);
}

int usb3380_csr_mm_mcu_read(libusb3380_context_t* ctx, uint32_t addr,
							uint32_t* data)
{
	return usb3380_int_csr_cfg_read(ctx, CSR_READ | CSR_SS_8051_PRG |
									CSR_START | (addr << CSR_ADDR_OFF_BITS),
									data);
}

int usb3380_csr_mm_mcu_write(libusb3380_context_t* ctx, uint32_t addr,
							 uint32_t data)
{
	return usb3380_int_csrout_sync(ctx, CSR_WRITE | CSR_BYTE_EN_ALL |
								   CSR_SS_8051_PRG | CSR_START |
								   (addr << CSR_ADDR_OFF_BITS), data);
}

int usb3380_csr_mm_mcu_copy(libusb3380_context_t* ctx, uint32_t addr,
							const uint32_t* pdata, size_t dwords)
{
	int res = 0;
	for (unsigned i = 0; i < dwords; i++) {
		res = usb3380_csr_mm_mcu_write(ctx, addr + 4 * i, pdata[i]);
		if (res)
			break;
	}
	return res;
}

int usb3380_mcu_sem_acq(libusb3380_context_t* ctx, uint32_t *res)
{
	return usb3380_csr_mm_cfg_read(ctx, SEMAPHORE, res);
}

int usb3380_mcu_sem_rel(libusb3380_context_t* ctx)
{
	return usb3380_csr_mm_cfg_write(ctx, SEMAPHORE, 0);
}

int usb3380_mcu_reset(libusb3380_context_t* ctx, int reset)
{
	uint32_t devinit;
	int res;

	res = usb3380_csr_mm_cfg_read(ctx, DEVINIT, &devinit);
	if (res)
		return res;

	if (reset) {
		devinit |= MCU_RESET_MASK;
	} else {
		devinit &= ~MCU_RESET_MASK;
	}

	return usb3380_csr_mm_cfg_write(ctx, DEVINIT, devinit);
}

int usb3380_idxreg_read(libusb3380_context_t* ctx, uint16_t addr,
						uint32_t* data)
{
	int res;
	res = usb3380_csr_mm_cfg_write(ctx, IDXADDR, addr);
	if (res) {
		return res;
	}
	res = usb3380_csr_mm_cfg_read(ctx, IDXDATA, data);
	return res;
}

int usb3380_idxreg_write(libusb3380_context_t* ctx, uint16_t addr,
						 uint32_t data)
{
	int res;
	res = usb3380_csr_mm_cfg_write(ctx, IDXADDR, addr);
	if (res) {
		return res;
	}
	res = usb3380_csr_mm_cfg_write(ctx, IDXDATA, data);
	return res;
}


static int usb3380_int_pciout_sync(libusb3380_context_t* ctx,
							uint32_t pcictl,
							uint32_t pciaddr,
							const uint32_t* pcidata,
							const unsigned pcidata_size_dw)
{
	uint32_t tmp[pcidata_size_dw + 2];
	int written = 0;
	int res;

	tmp[0] = pcictl;
	tmp[1] = pciaddr;
	if (pcidata_size_dw) {
		memcpy(&tmp[2], pcidata, 4 * pcidata_size_dw);
	}

	res = libusb_bulk_transfer(ctx->handle,
								   EP_PCIOUT | LIBUSB_ENDPOINT_OUT,
								   (unsigned char*)&tmp,
								   8 + 4 * pcidata_size_dw,
								   &written,
								   ctx->default_to);
	return res;
}

static int usb3380_int_pciin_sync(libusb3380_context_t* ctx,
						   uint32_t* pcidata,
						   unsigned pcidata_size_dw,
						   int* written)
{
	int res = libusb_bulk_transfer(ctx->handle,
								   EP_PCIIN | LIBUSB_ENDPOINT_IN,
								   (unsigned char*)pcidata,
								   4 * pcidata_size_dw,
								   written,
								   ctx->default_to);
	return res;
}

static int usb3380_int_rcin_sync(libusb3380_context_t* ctx, uint32_t* rcdata,
								 unsigned size_dw, int* written, long timeout)
{
	int res = libusb_interrupt_transfer(ctx->handle,
										EP_RCIN | LIBUSB_ENDPOINT_IN,
										(unsigned char*)rcdata,
										4 * size_dw,
										written,
										timeout);
	return res;
}

static int usb3380_int_statin_sync(libusb3380_context_t* ctx, uint32_t* rcdata,
								   unsigned size_dw, int* written, long timeout)
{
	int res = libusb_interrupt_transfer(ctx->handle,
										EP_STATIN | LIBUSB_ENDPOINT_IN,
										(unsigned char*)rcdata,
										4 * size_dw,
										written,
										timeout);
	return res;
}


static int usb3380_int_pci_read(libusb3380_context_t* ctx, uint32_t ctrl,
								uint32_t addr, uint32_t* data,
								unsigned data_size_dw)
{
	int res;
	int written;

	res = usb3380_int_pciout_sync(ctx, ctrl, addr, NULL, 0);
	if (res) {
		LOG_ERR("Unable to usb3380_int_pciout_sync! error=`%s` (%d)",
				libusb_error_name(res), res);
		return res;
	}

	res = usb3380_int_pciin_sync(ctx, data, data_size_dw, &written);
	if (res) {
		LOG_ERR("Unable to usb3380_int_pciin_sync! written=%d error=`%s` (%d)",
				written, libusb_error_name(res), res);
		return res;
	}

	if (written != 4 * data_size_dw) {
		LOG_ERR("Written %d != %d requested!", written, data_size_dw);
		return LIBUSB_ERROR_IO;
	}
	return 0;
}


int usb3380_pci_cfg_read(libusb3380_context_t* ctx, uint32_t addr,
						 uint32_t* data)
{
	uint32_t flags = CSR_BYTE_EN_ALL | PCIMSTCTL_CMD_CONFIG |
			PCIMSTCTL_MASTER_START | PCIMSTCTL_PCIE_READ |
			(1 << PCIMSTCTL_PCIE_DW_LEN_OFF_BITS);
	int res = usb3380_int_pci_read(ctx, flags, addr, data, 1);
	return libusb_to_errno(res);
}

int usb3380_pci_cfg_write(libusb3380_context_t* ctx, uint32_t addr,
						  uint32_t data)
{
	uint32_t flags = CSR_BYTE_EN_ALL | PCIMSTCTL_CMD_CONFIG |
			PCIMSTCTL_MASTER_START | PCIMSTCTL_PCIE_WRITE |
			(1 << PCIMSTCTL_PCIE_DW_LEN_OFF_BITS);
	int res = usb3380_int_pciout_sync(ctx, flags, addr, &data, 1);
	return libusb_to_errno(res);
}


int usb3380_context_init(libusb3380_context_t** octx)
{
	return usb3380_context_init_ex(octx, NULL, NULL);
}

int usb3380_context_init_ex(libusb3380_context_t** octx, libusb_device *dev, libusb_context *lctx)
{
	int res;
	libusb3380_context_t* ctx;

	ctx = (libusb3380_context_t*)malloc(sizeof(libusb3380_context_t));
	if (ctx == NULL)
		return -ENOMEM;

	memset(ctx, 0, sizeof(libusb3380_context_t));
	ctx->context = lctx;

	if (lctx == NULL) {
		if (libusb_init(&ctx->context)) {
			LOG_ERR("Unable to initialize LIBUSB");
			res = -EFAULT;
			goto cleanup_mem;
		}
	}

	if (dev == NULL) {
		ctx->handle = libusb_open_device_with_vid_pid(ctx->context,
													  LIBUSB3380_VID,
													  LIBUSB3380_PID);
	} else {
		res = libusb_open(dev, &ctx->handle);
		if (res) {
			LOG_ERR("Unable to initialize DEVICE: error `%s` (%d)!",
					libusb_strerror((enum libusb_error)res), res);
			res = libusb_to_errno(res);
			goto cleanup_mem;
		}
	}

	if (ctx->handle == NULL) {
		LOG_ERR("Unable to open USB3380!");
		res = -ENXIO;
		goto cleanup_ctx;
	}

#if 0
	res = libusb_reset_device(ctx->handle);
	if (res) {
		LOG_ERR("Unable to reset USB3380: error `%s` (%d)!",
				libusb_strerror((enum libusb_error)res), res);
		res = libusb_to_errno(res);
		goto cleanup_handle;
	}
#endif

	if (libusb_kernel_driver_active(ctx->handle, 0)) {
		res = libusb_detach_kernel_driver(ctx->handle, 0);
		if (res) {
			LOG_DEBUG("Unable to detach kernel driver curently active for 3380: `%s` (%d)!",
					libusb_strerror((enum libusb_error)res), res);
		}
	}

	res = libusb_claim_interface(ctx->handle, 0);
	if (res) {
		LOG_ERR("Unable to claim interface: error `%s` (%d)!",
				libusb_strerror((enum libusb_error)res), res);
		res = libusb_to_errno(res);
		goto cleanup_handle;
	}


	ctx->default_to = 1000;
	*octx = ctx;
	return 0;

cleanup_handle:
	libusb_close(ctx->handle);
cleanup_ctx:
	libusb_exit(ctx->context);
cleanup_mem:
	free(ctx);
	return res;
}

void usb3380_context_free(libusb3380_context_t* ctx)
{
	libusb_release_interface(ctx->handle, 0);
	libusb_close(ctx->handle);
	libusb_exit(ctx->context);
	free(ctx);
}



static int parse_bar_config(const libusb3380_usbc_bar_ctrl_t* cfg,
							uint32_t* regout)
{
	unsigned i;
	uint32_t reg = 0;

	if ((cfg->length != 0 && cfg->length < BAR_64K) || cfg->length > BAR_2G) {
		LOG_ERR("Incorrect BAR size: %llu (%d bits)", 1ULL << cfg->length, cfg->length);
		return -EINVAL;
	}

	for (i = 0; i < 4; i++) {
		reg <<= 4;

		if (cfg->gpep_in_type[3-i])
			reg |= 1;

		reg |= ((cfg->qadrants_ep_map[3-i]) & 3) << 1;
	}

	if (cfg->length) {
		reg |= ~0U << cfg->length;
	}

	*regout = reg;
	return 0;
}

void usb3380_init_rc_def(libusb3380_pcie_rc_cfg_t *cfg)
{
	memset(cfg, 0, sizeof(*cfg));
}

int usb3380_init_root_complex(libusb3380_context_t* ctx,
							  const libusb3380_pcie_rc_cfg_t* cfg)
{
	int res;
	uint32_t reg;
	bool reinit = false;

	/* Check intersection of BAR2 and BAR3 spaces */
	if (cfg->bar2.length != 0 && cfg->bar3.length != 0) {
		uint32_t bar2_s = cfg->bar2.addr;
		uint32_t bar3_s = cfg->bar3.addr;
		uint32_t bar2_e = bar2_s + (1U << cfg->bar2.length);
		uint32_t bar3_e = bar3_s + (1U << cfg->bar3.length);

		if (((bar2_s < bar3_s) && (bar2_e > bar3_s)) ||
				((bar3_s < bar2_s) && (bar3_e > bar2_s))) {
			LOG_ERR("BAR2 & BAR3 intersects [0x%08x:0x%08x] [0x%08x:0x%08x]",
					bar2_s, bar2_e - 1, bar3_s, bar3_e - 1);
			return -EINVAL;
		}
	}

#if 0
	res = usb3380_csr_mm_cfg_read(ctx, DEVINIT, &reg);
	if (res) {
		return res;
	}
	res = usb3380_csr_mm_cfg_write(ctx, DEVINIT, reg | (1<<2));
	if (res) {
		return res;
	}
	res = usb3380_csr_mm_cfg_write(ctx, DEVINIT, reg);
	if (res) {
		return res;
	}
	do {
		res = usb3380_csr_mm_cfg_read(ctx, DEVINIT, &reg);
		if (res) {
			return res;
		}
		usleep(1000);
	} while (reg & (1<<2));
#endif
#if 0
	for (int i = 1; i < 5; i++) {
		res = usb3380_csr_mm_cfg_read(ctx, EP_CFG | EP_GPEP0_OFF * i, &reg);
		if (res) {
			return res;
		}
		LOG_DUMP("GPEP%d - %s %d", i-1,
			   (reg & (1<<7)) ? "IN ": "OUT", reg & 0xF);
	}
#endif
#if 0
	uint32_t reg;
	res = usb3380_csr_mm_cfg_read(ctx, DEP_CFG + DEP_RCIN_OFF, &reg);
	if (res) {
		return res;
	}
	LOG_DUMP("RCIN CFG %08x", reg);
	reg = reg & (~(1<<8));
	res = usb3380_csr_mm_cfg_write(ctx, DEP_CFG + DEP_RCIN_OFF, reg);
	if (res) {
		return res;
	}
#endif

	res = usb3380_idxreg_read(ctx, CHIPREV, &reg);
	if (res) {
		return res;
	}
	LOG_DEBUG("Chip REV %08x", reg);

	res = usb3380_csr_mm_cfg_read(ctx, USBCTL, &reg);
	if (res) {
		return res;
	}
	LOG_INFO("USB338x Speed: %d Mbps",
		   (reg & USBSTAT_SS_MODE) ? 5000 :
		   (reg & USBSTAT_HS_MODE) ? 480 :
		   (reg & USBSTAT_FS_MODE) ? 12 : 0);


	res = usb3380_idxreg_read(ctx, HS_INTPOLL_RATE, &reg);
	if (res) {
		return res;
	}

	res = usb3380_idxreg_write(ctx, HS_INTPOLL_RATE, 0x01);
	if (res) {
		return res;
	}

	res = usb3380_idxreg_write(ctx, FS_INTPOLL_RATE, 0x01);
	if (res) {
		return res;
	}

	/* Need to reinitialize */
	if (reg != 0x1) {
		LOG_WARN("Need to reinitialize the USB device dueto configuration change");
		reinit = true;
	}

	/* Do not terminate transaction with undefined data */
	res = usb3380_csr_mm_cfg_write(ctx, FIFOCTL, 0);
	if (res) {
		return res;
	}

	/* Enable dispatching everything to RCIN */
	res = usb3380_csr_mm_cfg_write(ctx, ROOTMSGDISP, 0);
	if (res) {
		return res;
	}

	res = parse_bar_config(&cfg->bar2, &reg);
	if (res) {
		return res;
	}

	res = usb3380_csr_mm_cfg_write(ctx, BAR2CTL, reg);
	if (res) {
		return res;
	}

	res = parse_bar_config(&cfg->bar3, &reg);
	if (res) {
		return res;
	}

	res = usb3380_csr_mm_cfg_write(ctx, BAR3CTL, reg);
	if (res) {
		return res;
	}

	/* Disable PCI BAR0 (CSR) and 8051 */
	res = usb3380_csr_mm_cfg_write(ctx, PCICTL, (0 << PCIBAR1) | (0 << PCIBAR0));
	if (res) {
		return res;
	}

	/* Configure BAR2 */
	res = usb3380_csr_pcie_cfg_write(ctx, PCIE_PORT_USBC | PCICFG_BAR2,
									 cfg->bar2.addr);
	if (res) {
		return res;
	}

	/* Configure BAR3 */
	res = usb3380_csr_pcie_cfg_write(ctx, PCIE_PORT_USBC | PCICFG_BAR3,
									 cfg->bar3.addr);
	if (res) {
		return res;
	}

	/* Configure BAR4 */
	res = usb3380_csr_pcie_cfg_write(ctx, PCIE_PORT_USBC | PCICFG_BAR4, 0);
	if (res) {
		return res;
	}

	/* Configure BAR5: (MSI to RCIN endpoint) */
	res = usb3380_csr_pcie_cfg_write(ctx, PCIE_PORT_USBC | PCICFG_BAR5, MSI_DEF_ADDR);
	if (res) {
		return res;
	}

	/* Enable IO / Memory / PCI master on USB Controller */
	res = usb3380_csr_pcie_cfg_write(ctx, PCIE_PORT_USBC | PCICFG_CMDSTAT, 0x07);
	if (res) {
		return res;
	}

	/* Enable IO / Memory / PCI master on PCIe Root Complex */
	res = usb3380_csr_pcie_cfg_write(ctx, PCIE_PORT_2 | PCICFG_CMDSTAT, 0x07);
	if (res) {
		return res;
	}

	/* Enable IO / Memory / PCI master on virtual PCI-PCI Bridge */
	res = usb3380_csr_pcie_cfg_write(ctx, PCIE_PORT_0 | PCICFG_CMDSTAT, 0x07);
	if (res) {
		return res;
	}

	/* Increase payload size to 256 bytes */
	res = usb3380_csr_pcie_cfg_read(ctx, PCIE_PORT_0 | PCICFG_PCIECAP_DEVSC, &reg);
	if (res) {
		return res;
	}
	reg = reg | (1U << 5);
	res = usb3380_csr_pcie_cfg_write(ctx, PCIE_PORT_0 | PCICFG_PCIECAP_DEVSC, reg);
	if (res) {
		return res;
	}
	res = usb3380_csr_pcie_cfg_read(ctx, PCIE_PORT_2 | PCICFG_PCIECAP_DEVSC, &reg);
	if (res) {
		return res;
	}
	reg = reg | (1U << 5);
	res = usb3380_csr_pcie_cfg_write(ctx, PCIE_PORT_2 | PCICFG_PCIECAP_DEVSC, reg);
	if (res) {
		return res;
	}
	res = usb3380_csr_pcie_cfg_read(ctx, PCIE_PORT_USBC| PCICFG_PCIECAP_DEVSC, &reg);
	if (res) {
		return res;
	}
	reg = reg | (1U << 5);
	res = usb3380_csr_pcie_cfg_write(ctx, PCIE_PORT_USBC | PCICFG_PCIECAP_DEVSC, reg);
	if (res) {
		return res;
	}
#if 0
	// Include only GPEP0 in IN creadit calculation
	res = usb3380_csr_pcie_cfg_read(ctx, PCIE_PORT_USBC | 0x9f0, &reg);
	if (res) {
		return res;
	}
	LOG_DEBUG("INCH %8x", reg);
#endif
	// Enabgle credit calculation only for requested GPEPs
	unsigned i;
	for (reg = 0, i = 0; i < 4; i++) {
		if (cfg->gpep_fifo_in_size[i] == 0)
			reg |= (1U << (28 + i));
	}
	res = usb3380_csr_pcie_cfg_write(ctx, PCIE_PORT_USBC | 0x9f0, reg);
	if (res) {
		return res;
	}

	// Enable AER on USBC
#if 0
	res = usb3380_csr_pcie_cfg_read(ctx, PCIE_PORT_2 | 0x9f8, &reg);
	if (res) {
		return res;
	}
	LOG_DUMP("INCH %8x", reg);
	reg |= (1 << 27) | (1 << 29);
	res = usb3380_csr_pcie_cfg_write(ctx, PCIE_PORT_2 | 0x9f8, reg);
	if (res) {
		return res;
	}
#endif

	const uint8_t root_bus = 0;
	const uint8_t port2_bus = 1;
	const uint8_t port0_bus = 2;

	res = usb3380_csr_pcie_cfg_write(ctx, PCIE_PORT_2 | PCICFG_TYPE1_BUSNUM,
									 (root_bus << BUSNO_PRIMARY_OFF) |
									 (port2_bus << BUSNO_SECONDARY_OFF) |
									 (port0_bus << BUSNO_SUBORDINATE_OFF));
	if (res) {
		return res;
	}

	res = usb3380_csr_pcie_cfg_write(ctx, PCIE_PORT_0 | PCICFG_TYPE1_BUSNUM,
									 (port2_bus << BUSNO_PRIMARY_OFF) |
									 (port0_bus << BUSNO_SECONDARY_OFF) |
									 (port0_bus << BUSNO_SUBORDINATE_OFF));
	if (res) {
		return res;
	}

	// Check EP states
	const char* ep_names[6] = { "CSROUT", "CSRIN", "PCIOUT", "PCIIN", "STATIN",
								"RCIN" };
	for (i = 0; i < 6; i++) {
		res = usb3380_csr_mm_cfg_read(ctx, DEP_RSP + i * 0x10, &reg);
		if (res) {
			return res;
		}
		LOG_DEBUG("EPd %9s: %08x Halt:%d Toggle:%d",
				ep_names[i], reg, (reg & 1) ? 1 : 0, (reg & 2) ? 1 : 0);

		res = usb3380_csr_mm_cfg_write(ctx, DEP_RSP + i * 0x10, 0);
		if (res) {
			return res;
		}
	}

	const char* gpep_names[11] = { "EP0", "GPEP0 OUT", "GPEP1 OUT", "GPEP2 OUT",
								   "GPEP3 OUT", NULL, NULL, "GPEP0 IN",
								   "GPEP1 IN", "GPEP2 IN", "GPEP3 IN" };
	for (i = 0; i < 11; i++) {
		if (i == 5 || i == 6)
			continue;

		uint32_t eprsp, avail;
		res = usb3380_csr_mm_cfg_read(ctx, EP_STAT + i * 0x20, &reg);
		if (res) {
			return res;
		}
		res = usb3380_csr_mm_cfg_read(ctx, EP_RSP + i * 0x20, &eprsp);
		if (res) {
			return res;
		}
		res = usb3380_csr_mm_cfg_read(ctx, EP_AVAIL + i * 0x20, &avail);
		if (res) {
			return res;
		}

		LOG_DEBUG("EPg %9s: %04x %08x %08x Pkts: %2d Avail:%5d Empty:%d Full:%d STALL:%d TO:%d",
				  gpep_names[i], EP_CFG + i*0x20, eprsp, reg, (reg >> 24) & 0x1f, avail,
				  (reg & (1<<10)) ? 1 : 0, (reg & (1<<11)) ? 1 : 0,
				  (reg & (1<<20)) ? 1 : 0, (reg & (1<<21)) ? 1 : 0);

		res = usb3380_csr_mm_cfg_write(ctx, EP_STAT + i * 0x20, 1<<9);
		if (res) {
			return res;
		}

		res = usb3380_csr_mm_cfg_write(ctx, EP_RSP + i * 0x20, 0);
		if (res) {
			return res;
		}
	}

	const char* ep_sizes[9] = { "EP0", "GPEP0", "GPEP1", "GPEP2", "GPEP3",
								NULL, NULL, "PCIx", "RCIN" };
	for (i = 0; i < 9; i++) {
		if (i == 5 || i == 6)
			continue;
		res = usb3380_csr_mm_cfg_read(ctx, EP_FIFO_SIZE_BASE + i * 0x20, &reg);
		if (res) {
			return res;
		}

		unsigned o_sz = (64U << (reg & 7));
		unsigned o_st = ((reg >> 6) & 0x1ff) * 64;
		unsigned i_sz = (64U << ((reg >> 16) & 7));
		unsigned i_st = ((reg >> (6+16)) & 0x1ff) * 64;

		LOG_DEBUG("EP  %9s: %08x OUT sz:%4d OUT addr:%5d--%5d  IN sz:%4d IN addr:%5d--%5d",
				ep_sizes[i], reg,
				o_sz, o_st, o_st + o_sz,
				i_sz, i_st, i_st + i_sz);
	}

	const struct fifo_size_base {
		uint16_t ep; uint8_t sz; uint16_t off;
	} fifo_in_sz[] = {
		{ EP_EP0_OFF,      EP_FIFO_512,  0 },
		{ EP_GPEP0_OFF,    EP_FIFO_4096, 0 + 8 + 1 },
		{ EP_GPEP1_OFF,    EP_FIFO_128,  0 + 8 + 1 + 64 + 1 },
		{ EP_GPEP2_OFF,    EP_FIFO_4096, 384 },
		{ EP_GPEP3_OFF,    EP_FIFO_128,  384 + 64 + 1 },
		{ EP_PCIINOUT_OFF, EP_FIFO_256,  384 + 64 + 1 + 4 + 1 },
		{ EP_RCIN_OFF,     EP_FIFO_256,  384 + 64 + 1 + 4 + 1 + 4 + 1 },
	};

	for (i = 0; i < sizeof(fifo_in_sz) / sizeof(fifo_in_sz[0]); i++) {
		if (fifo_in_sz[i].sz > EP_FIFO_4096)
			continue;

		res = usb3380_csr_mm_cfg_read(ctx, EP_FIFO_SIZE_BASE + fifo_in_sz[i].ep, &reg);
		if (res) {
			return res;
		}

		// preserve OUT settings and override IN
		reg = (0xffff & reg) | (fifo_in_sz[i].sz << (16)) | (fifo_in_sz[i].off << (6+16));
		res = usb3380_csr_mm_cfg_write(ctx, EP_FIFO_SIZE_BASE + fifo_in_sz[i].ep, reg);
		if (res) {
			return res;
		}
	}

	for (i = 1; i < 5; i++) {
		res = usb3380_csr_mm_cfg_read(ctx, EP_CFG + i * 0x20, &reg);
		if (res) {
			return res;
		}
		LOG_DEBUG("EP  %9s: %08x", ep_sizes[i], reg);

		reg |= 15U << 24;

		res = usb3380_csr_mm_cfg_write(ctx, EP_CFG + i * 0x20, reg);
		if (res) {
			return res;
		}
	}

	if (reinit)
		return -EAGAIN;

	return 0;
}

uint32_t usb3380_pci_dev_bar_addr(libusb3380_pcidev_t* ctx, unsigned bar)
{
	if (bar > 5) {
		return 0;
	}

	if (ctx->bars[bar].addr & BAR_TYPE_IO) {
		return (ctx->bars[bar].addr & 0xFFFFFFFC);
	} else {
		return (ctx->bars[bar].addr & 0xFFFFFFF0);
	}
}

uint32_t usb3380_pci_dev_bar_length(libusb3380_pcidev_t* ctx, unsigned bar)
{
	if (bar > 5) {
		return 0;
	}

	return ctx->bars[bar].length;
}


static int initdev_pcie_cap(libusb3380_context_t* ctx, libusb3380_pcidev_t *dev,
							uint8_t dev_bus, uint8_t cap_addr,
							uint32_t reg)
{
	uint32_t tmp;
	int res;

	LOG_DEBUG("PCIe CAP0: Ver=%d PCIe_Type=%x",
			(reg >> (16)) & 0xf,
			(reg >> (16+4)) & 0xf);

	res = usb3380_pci_cfg_read(ctx, MAKE_CFG_0(dev_bus, 0, 0, cap_addr + 4), &tmp);
	if (res) {
		return res;
	}
	LOG_DEBUG("PCIe CAP1: MaxPayload=%d",
			128 << (tmp & 0x3));


	res = usb3380_pci_cfg_read(ctx, MAKE_CFG_0(dev_bus, 0, 0, cap_addr + 8), &tmp);
	if (res) {
		return res;
	}
	LOG_DEBUG("PCIe CTRL: MaxPayload=%d MaxReadReq=%d",
			128 << ((tmp >> 5) & 0x7),
			128 << ((tmp >> 12) & 0x7));

	tmp = tmp | (1U << 5);
	tmp = tmp & (~(7U << 12));
	tmp = tmp | (5U << 12);
	res = usb3380_pci_cfg_write(ctx, MAKE_CFG_0(dev_bus, 0, 0, cap_addr + 8), tmp);
	if (res) {
		return res;
	}

	res = usb3380_pci_cfg_read(ctx, MAKE_CFG_0(dev_bus, 0, 0, cap_addr + 12), &tmp);
	if (res) {
		return res;
	}
	LOG_DEBUG("PCIe LCAP: Speed=Gen%d Width=x%d",
			(tmp & 0xf),
			(tmp >> 4) & 0x3f);


	res = usb3380_pci_cfg_read(ctx, MAKE_CFG_0(dev_bus, 0, 0, cap_addr + 16), &tmp);
	if (res) {
		return res;
	}
	LOG_DEBUG("PCIe LSTA: Speed=Gen%d Width=x%d",
			((tmp >> 16) & 0xf),
			(tmp >> (4 + 16)) & 0x3f);

	return 0;
}

int usb3380_init_first_dev(libusb3380_context_t* ctx, unsigned flags,
						   libusb3380_pcidev_t **out)
{
	uint32_t maxbarlen_mem = 0;
	uint32_t maxbarlen_io = 0;
	uint32_t reg, rb;
	int res;
	unsigned i;
	unsigned ioidx, memidx;
	if (ctx->dev.ctx) {
		return -ENOMEM;
	}
	const uint8_t dev_bus = 2;

	res = usb3380_pci_cfg_read(ctx, MAKE_CFG_0(dev_bus, 0, 0, PCICFG_MSCCTL), &reg);
	if (res) {
		return res;
	}

	if (((reg >> HEADER_TYPE_OFF_BITS) & HEADER_TYPE_MASK) != HEADER_TYPE_STANDARD) {
		LOG_ERR("Unknown header type %x!", ((reg >> HEADER_TYPE_OFF_BITS) & HEADER_TYPE_MASK));
		return -ENODEV;
	}

	res = usb3380_pci_cfg_read(ctx, MAKE_CFG_0(dev_bus, 0, 0, PCICFG_VIDDID), &reg);
	if (res) {
		return res;
	}

	ctx->dev.devid = 0x0100;
	ctx->dev.did = reg >> 16;
	ctx->dev.vid = (uint16_t)reg;
	ctx->dev.cfg_msi_numcap = 0;
	ctx->dev.cfg_msi_addr = 0;

	/* Get bar sizes */
	for (i = 0; i < 6; i++) {
		res = usb3380_pci_cfg_read(ctx, MAKE_CFG_0(dev_bus, 0, 0, PCICFG_BAR0 + i*4), &reg);
		if (res) {
			return res;
		}

		res = usb3380_pci_cfg_write(ctx, MAKE_CFG_0(dev_bus, 0, 0, PCICFG_BAR0 + i*4), ~0U);
		if (res) {
			return res;
		}

		res = usb3380_pci_cfg_read(ctx, MAKE_CFG_0(dev_bus, 0, 0, PCICFG_BAR0 + i*4), &rb);
		if (res) {
			return res;
		}

		if (reg & BAR_TYPE_IO) {
			ctx->dev.bars[i].addr = reg;
			ctx->dev.bars[i].length = (~(rb & 0xFFFFFFFC) | 0x3) + 1;
			if (maxbarlen_io < ctx->dev.bars[i].length) {
				maxbarlen_io = ctx->dev.bars[i].length;
			}
		} else {
			ctx->dev.bars[i].addr = reg;
			ctx->dev.bars[i].length = (~(rb & 0xFFFFFFF0) | 0xF) + 1;
			if (maxbarlen_mem < ctx->dev.bars[i].length) {
				maxbarlen_mem = ctx->dev.bars[i].length;
			}

			if ((reg & BAR_TYPE_MEM_MSK) == BAR_TYPE_MEM_32BIT) {
			} else if ((reg & BAR_TYPE_MEM_MSK) == BAR_TYPE_MEM_64BIT) {
				if ((rb & 0xFFFFFFF0) == 0) {
					LOG_ERR("PCIdev: 64 bit mode with 4GB+ isn't supported! [%08x]", reg);
					return -ENODEV;
				}

				/* zero high addr part */
				res = usb3380_pci_cfg_write(ctx, MAKE_CFG_0(dev_bus, 0, 0, PCICFG_BAR0 + i*4 + 4), 0);
				if (res) {
					return res;
				}
				i++;
				ctx->dev.bars[i].length = 0;

			} else {
				LOG_ERR("PCIdev: ISA mode currently not supported! [%08x]", reg);
				return -ENODEV;
			}
		}
	}

	/* Writing back addresses */
	for (i = 0, ioidx = 0, memidx = 0; i < 6; i++) {
		if (ctx->dev.bars[i].length == 0) {
			ctx->dev.bars[i].addr = 0;
			continue;
		}

		if (ctx->dev.bars[i].addr & BAR_TYPE_IO) {
			ctx->dev.bars[i].addr = (++ioidx * maxbarlen_io) | BAR_TYPE_IO;
		} else {
			ctx->dev.bars[i].addr = (++memidx * maxbarlen_mem) | (ctx->dev.bars[i].addr & 0xF);
		}

		res = usb3380_pci_cfg_write(ctx, MAKE_CFG_0(dev_bus, 0, 0, PCICFG_BAR0 + i*4), ctx->dev.bars[i].addr);
		if (res) {
			return res;
		}
	}

	/* Checking capabilities */
	res = usb3380_pci_cfg_read(ctx, MAKE_CFG_0(dev_bus, 0, 0, PCICFG_CMDSTAT), &reg);
	if (res) {
		return res;
	}
	if (reg & 0x100000) {
		uint8_t cap_addr;
		uint8_t nxt_addr;
		res = usb3380_pci_cfg_read(ctx, MAKE_CFG_0(dev_bus, 0, 0, PCICFG_TYPE1_CAPPTR), &reg);
		if (res) {
			return res;
		}
		cap_addr = reg & 0xFC;
		do {
			uint8_t type;
			LOG_DUMP("Capability points to %x", cap_addr);
			res = usb3380_pci_cfg_read(ctx, MAKE_CFG_0(dev_bus, 0, 0, cap_addr), &reg);
			if (res) {
				return res;
			}
			nxt_addr = (reg >> 8) & 0xFC;
			type = reg & 0xff;

			switch (type) {
			case PCICAP_MSI:
			{
				uint16_t msicap = reg >> 16;
				unsigned msi64bit = (msicap & PCICAP_MSI_64BIT);
				unsigned bitmsgs = (msicap >> PCICAP_MSI_MMCAP_OFF_BITS) & 7;

				if (flags & NO_MSI) {
					LOG_DEBUG("MSI disabled, ignoring");
					break;
				}
				LOG_DEBUG("MSI %dbit up to %d msgs",
					   (msi64bit) ? 64 : 32,
					   1 << bitmsgs);

				ctx->dev.cfg_msi_numcap = bitmsgs;
				ctx->dev.cfg_msi_addr = cap_addr;

				res = usb3380_pci_cfg_write(ctx, MAKE_CFG_0(dev_bus, 0, 0, ctx->dev.cfg_msi_addr + 4), MSI_DEF_ADDR);
				if (res) {
					return res;
				}
				if (msi64bit) {
					res = usb3380_pci_cfg_write(ctx, MAKE_CFG_0(dev_bus, 0, 0, ctx->dev.cfg_msi_addr + 8), MSI_DEF_ADDR_HIGH);
					if (res) {
						return res;
					}
				}
				res = usb3380_pci_cfg_write(ctx, MAKE_CFG_0(dev_bus, 0, 0, ctx->dev.cfg_msi_addr + (msi64bit ? 12U : 8U)),
											MSI_DEF_DATA);
				if (res) {
					return res;
				}
				/* Enable all vectos MSI */
				res = usb3380_pci_cfg_write(ctx, MAKE_CFG_0(dev_bus, 0, 0, ctx->dev.cfg_msi_addr),
											(msicap | (bitmsgs << PCICAP_MSI_MMEN_OFF_BITS) | PCICAP_MSI_ENABLE) << 16);
				if (res) {
					return res;
				}

				break;
			}
			case PCICAP_PCIE:
				res = initdev_pcie_cap(ctx, &ctx->dev, dev_bus, cap_addr, reg);
				if (res) {
					return res;
				}
				break;
			default:
				LOG_DEBUG("PCI Capability ID:%02x ignored", type);
			}

			LOG_DUMP("Nxt:%x ID:%x", cap_addr, type);
			cap_addr = nxt_addr;
		} while (cap_addr != 0);
	}

	/* Enabling device */
	res = usb3380_pci_cfg_read(ctx, MAKE_CFG_0(dev_bus, 0, 0, PCICFG_BAR0 + i*4), &reg);
	if (res) {
		return res;
	}
	res = usb3380_pci_cfg_write(ctx, MAKE_CFG_0(dev_bus, 0, 0, PCICFG_CMDSTAT), reg | 0x7);
	if (res) {
		return res;
	}

	LOG_INFO("Initialized PCIe dev %04x [%04x:%04x]", ctx->dev.devid, ctx->dev.vid, ctx->dev.did);
	for (i = 0; i < 6; i++) {
		if (ctx->dev.bars[i].length == 0)
			continue;

		LOG_DEBUG("BAR[%i] %s Len: 0x%08x mmaped to 0x%08x",
			   i,
			   (ctx->dev.bars[i].addr & BAR_TYPE_IO) ? "IO " : "Mem",
			   ctx->dev.bars[i].length,
			   ctx->dev.bars[i].addr);
	}

	ctx->dev.ctx = ctx;
	*out = &ctx->dev;
	return 0;
}

uint16_t usb3380_pci_dev_did(libusb3380_pcidev_t* ctx)
{
	return ctx->did;
}

uint16_t usb3380_pci_dev_vid(libusb3380_pcidev_t* ctx)
{
	return ctx->vid;
}


int usb3380_pci_dev_mem_read32(libusb3380_context_t* ctx, uint32_t addr,
							   uint32_t* data)
{
	uint32_t flags = CSR_BYTE_EN_ALL | PCIMSTCTL_CMD_MEMORY |
			PCIMSTCTL_MASTER_START | PCIMSTCTL_PCIE_READ |
			(1 << PCIMSTCTL_PCIE_DW_LEN_OFF_BITS);
	int res = usb3380_int_pci_read(ctx, flags, addr, data, 1);
	return libusb_to_errno(res);
}

int usb3380_pci_dev_mem_read32_n(libusb3380_context_t* ctx, uint32_t addr,
								 uint32_t* data, unsigned count_dw)
{
	uint32_t flags = CSR_BYTE_EN_ALL | PCIMSTCTL_CMD_MEMORY |
			PCIMSTCTL_MASTER_START | PCIMSTCTL_PCIE_READ |
			(count_dw << PCIMSTCTL_PCIE_DW_LEN_OFF_BITS);
	int res = usb3380_int_pci_read(ctx, flags, addr, data, count_dw);
	return libusb_to_errno(res);
}

int usb3380_pci_dev_mem_write32(libusb3380_context_t *ctx, uint32_t addr,
								uint32_t data)
{
	uint32_t flags = CSR_BYTE_EN_ALL | PCIMSTCTL_CMD_MEMORY |
			PCIMSTCTL_MASTER_START | PCIMSTCTL_PCIE_WRITE |
			(1 << PCIMSTCTL_PCIE_DW_LEN_OFF_BITS);
	int res = usb3380_int_pciout_sync(ctx, flags, addr, &data, 1);
	return libusb_to_errno(res);
}

int usb3380_pci_dev_mem_write32_n(libusb3380_context_t* ctx, uint32_t addr,
								  const uint32_t* data, unsigned count_dw)
{
	uint32_t flags = CSR_BYTE_EN_ALL | PCIMSTCTL_CMD_MEMORY |
			PCIMSTCTL_MASTER_START | PCIMSTCTL_PCIE_WRITE |
			(count_dw << PCIMSTCTL_PCIE_DW_LEN_OFF_BITS);
	int res = usb3380_int_pciout_sync(ctx, flags, addr, data, count_dw);
	return libusb_to_errno(res);
}

int usb3380_pci_wait_interrupt(libusb3380_context_t *ctx, long timeoutms)
{
	if (ctx->dev.cfg_msi_numcap) {
		uint32_t bufmsgs[4];
		int written;
		int res;

		res = usb3380_int_rcin_sync(ctx, bufmsgs, 4, &written, timeoutms);
		if (res) {
			LOG_ERR("Unable to usb3380_int_rcin_sync! error=`%s` (%d)",
					libusb_error_name(res), res);
			return libusb_to_errno(res);
		}

		LOG_DUMP("MSI %08x %08x %08x %08x", bufmsgs[0], bufmsgs[1], bufmsgs[2], bufmsgs[3]);
	} else {
		uint32_t bufmsgs[2];
		int written;
		int res;

		res = usb3380_int_statin_sync(ctx, bufmsgs, 2, &written, timeoutms);
		if (res) {
			return libusb_to_errno(res);
		}

		LOG_DUMP("STAT %08x %08x", bufmsgs[0], bufmsgs[1]);
	}

	return 0;
}

int usb3380_gpep_read(libusb3380_context_t* ctx, libusb3380_gpep_t no,
					  uint8_t* data, int size, int* written, unsigned to)
{
	unsigned char ep_no = convert_gpep_no(no);
	int res = libusb_bulk_transfer(ctx->handle,
								   ep_no | LIBUSB_ENDPOINT_IN,
								   (unsigned char*)data,
								   size,
								   written,
								   to);
	return libusb_to_errno(res);
}


int usb3380_gpep_write(libusb3380_context_t* ctx, libusb3380_gpep_t no,
					   const uint8_t* data, int size, int* written, unsigned to)
{
	unsigned char ep_no = convert_gpep_no(no);
	int res = libusb_bulk_transfer(ctx->handle,
								   ep_no | LIBUSB_ENDPOINT_OUT,
								   (unsigned char*)data,
								   size,
								   written,
								   to);
	return libusb_to_errno(res);
}


static LIBUSB_CALL void on_async_msi_cb(struct libusb_transfer *transfer)
{
	libusb3380_async_manager_t* mgr = (libusb3380_async_manager_t*)transfer->user_data;
	int msi_num = -1;

	fill_base_in_cb(&mgr->msi.base, mgr->q_msi.transfer);

	if (mgr->msi.base.status == DQS_SUCCESS) {
		msi_num = mgr->msi_data[3] - MSI_DEF_DATA;

		LOG_DUMP("on_async_msi_cb addr=%08x:%08x:%08x:%08x num=%d",
				mgr->msi_data[0], mgr->msi_data[1], mgr->msi_data[2],
				mgr->msi_data[3], msi_num);

		if (msi_num < 0 || msi_num > 31) {
			msi_num = -1;
		}
	}

	mgr->msi_cb(mgr->msi_param, msi_num, mgr->msi.base.status == DQS_TIMEOUT);
}


int usb3380_msi_in_post(struct libusb3380_async_manager* mgr,
						unsigned timeoutms,
						on_msi_cb_t cb,
						void* param)
{
	int res;
	mgr->msi_cb = cb;
	mgr->msi_param = param;

	prepare_ptransfer_rcin(mgr, on_async_msi_cb, timeoutms);
	res = libusb_submit_transfer(mgr->q_msi.transfer);

	return libusb_to_errno(res);
}

int usb3380_msi_in_cancel(struct libusb3380_async_manager* mgr)
{
	return libusb_to_errno(libusb_cancel_transfer(mgr->q_msi.transfer));
}
