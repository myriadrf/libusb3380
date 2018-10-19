/*
 * Public libusb3380 header file
 * Copyright (c) 2017 Sergey Kostanbaev <sergey.kostanbaev@fairwaves.co>
 * For more information, please visit: http://xtrx.io
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

#ifndef LIBUSB3380_H
#define LIBUSB3380_H

#include <libusb-1.0/libusb.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

struct libusb3380_context;
typedef struct libusb3380_context libusb3380_context_t;

enum libusb3380_context_vidpid {
	LIBUSB3380_VID = 0x0525,
	LIBUSB3380_PID = 0x3380,
};

int usb3380_context_init(libusb3380_context_t** octx);
int usb3380_context_init_ex(libusb3380_context_t** octx, libusb_device *dev, libusb_context *ctx);
void usb3380_context_free(libusb3380_context_t* ctx);


typedef enum libusb3380_loglevel {
	USB3380_LOG_ERROR,
	USB3380_LOG_WARNING,
	USB3380_LOG_INFO,
	USB3380_LOG_DEBUG,
	USB3380_LOG_DUMP,
} libusb3380_loglevel_t;

typedef void (*usb3380_logfunc_t)(libusb3380_loglevel_t lvl, void* obj,
								  const char* message, ...);

void usb3380_set_logfunc(usb3380_logfunc_t func, void *param);
void usb3380_set_loglevel(libusb3380_loglevel_t level);


typedef enum libusb3380_gpep {
	LIBUSB3380_GPEP0,
	LIBUSB3380_GPEP1,
	LIBUSB3380_GPEP2,
	LIBUSB3380_GPEP3,
	LIBUSB3380_GPEP_COUNT,
} libusb3380_gpep_t;

typedef enum libusb3380_bar_size {
	BAR_NONE = 0,
	BAR_64K  = 16,
	BAR_128K = 17,
	BAR_256K = 18,
	BAR_512K = 19,
	BAR_1M   = 20,
	BAR_2M   = 21,
	BAR_4M   = 22,
	BAR_8M   = 23,
	BAR_16M  = 24,
	BAR_32M  = 25,
	BAR_64M  = 26,
	BAR_128M = 27,
	BAR_256M = 28,
	BAR_512M = 29,
	BAR_1G   = 30,
	BAR_2G   = 31,
} libusb3380_bar_size_t;

typedef struct libusb3380_usbc_bar_ctrl {
	uint32_t addr;
	libusb3380_bar_size_t length;

	/** Each quandrant of this region is mmaped to the specific GPEP index */
	libusb3380_gpep_t qadrants_ep_map[4];
	bool gpep_in_type[4];

	unsigned flags;
} libusb3380_usbc_bar_ctrl_t;

enum libusb3380_pcie_rc_flags {
	LIBUSB3380_EN_256B_PAYLOAD = 1,
};


typedef struct libusb3380_pcie_rc_cfg {
	libusb3380_usbc_bar_ctrl_t bar2;
	libusb3380_usbc_bar_ctrl_t bar3;

	unsigned gpep_fifo_in_size[LIBUSB3380_GPEP_COUNT];
	unsigned gpep_fifo_out_size[LIBUSB3380_GPEP_COUNT];

	unsigned flags;
} libusb3380_pcie_rc_cfg_t;

void usb3380_init_rc_def(libusb3380_pcie_rc_cfg_t *cfg);

int usb3380_gpio_dir(libusb3380_context_t* ctx, uint8_t diroutbits);
int usb3380_gpio_out(libusb3380_context_t* ctx, uint8_t out);
int usb3380_gpio_in(libusb3380_context_t* ctx, uint8_t *in);

/**
 * @brief usb3380_init_bridges Initialize internal bridges
 * @param ctx
 * @return
 */
int usb3380_init_root_complex(libusb3380_context_t* ctx,
							  const libusb3380_pcie_rc_cfg_t *cfg);

int usb3380_csr_pcie_cfg_read(libusb3380_context_t* ctx, uint32_t addr,
							  uint32_t* data);
int usb3380_csr_pcie_cfg_write(libusb3380_context_t* ctx, uint32_t addr,
							   uint32_t data);

int usb3380_csr_mm_cfg_read(libusb3380_context_t* ctx, uint32_t addr,
							uint32_t* data);
int usb3380_csr_mm_cfg_write(libusb3380_context_t* ctx, uint32_t addr,
							 uint32_t data);

int usb3380_csr_mm_mcu_read(libusb3380_context_t* ctx, uint32_t addr,
							uint32_t* data);
int usb3380_csr_mm_mcu_write(libusb3380_context_t* ctx, uint32_t addr,
							 uint32_t data);

int usb3380_csr_mm_mcu_copy(libusb3380_context_t* ctx, uint32_t addr,
							const uint32_t* pdata, size_t dwords);

int usb3380_mcu_sem_acq(libusb3380_context_t* ctx, uint32_t* res);
int usb3380_mcu_sem_rel(libusb3380_context_t* ctx);
int usb3380_mcu_reset(libusb3380_context_t* ctx, int reset);

int usb3380_idxreg_read(libusb3380_context_t* ctx, uint16_t addr,
						uint32_t* data);
int usb3380_idxreg_write(libusb3380_context_t* ctx, uint16_t addr,
						 uint32_t data);


#define MAKE_CFG(bus, dev, func, reg, type) \
	(((type) << 24) | (((reg) & 0xFC) << 24) | (((dev) & 0x1f) << 11) | \
	(((func) & 0x7) << 8) | ((bus) & 0xFF))

#define MAKE_CFG_1(bus, dev, func, reg) MAKE_CFG(bus, dev, func, reg, 1)
#define MAKE_CFG_0(bus, dev, func, reg) MAKE_CFG(bus, dev, func, reg, 0)

int usb3380_pci_cfg_read(libusb3380_context_t* ctx, uint32_t addr,
						 uint32_t* data);
int usb3380_pci_cfg_write(libusb3380_context_t* ctx, uint32_t addr,
						  uint32_t data);

struct libusb3380_pcidev;
typedef struct libusb3380_pcidev libusb3380_pcidev_t;


enum usb3380_init_flags {
	NO_MSI = 1,
};

int usb3380_init_first_dev(libusb3380_context_t* ctx, unsigned flags,
						   libusb3380_pcidev_t **out);

uint16_t usb3380_pci_dev_did(libusb3380_pcidev_t* ctx);
uint16_t usb3380_pci_dev_vid(libusb3380_pcidev_t* ctx);

uint32_t usb3380_pci_dev_bar_addr(libusb3380_pcidev_t* ctx, unsigned bar);
uint32_t usb3380_pci_dev_bar_length(libusb3380_pcidev_t* ctx, unsigned bar);

int usb3380_pci_dev_mem_read32(libusb3380_context_t* ctx, uint32_t addr,
							   uint32_t* data);
int usb3380_pci_dev_mem_write32(libusb3380_context_t* ctx, uint32_t addr,
								uint32_t data);

int usb3380_pci_dev_mem_write32_n(libusb3380_context_t* ctx, uint32_t addr,
								  const uint32_t* data, unsigned count_dw);
int usb3380_pci_dev_mem_read32_n(libusb3380_context_t* ctx, uint32_t addr,
								 uint32_t* data, unsigned count_dw);

int usb3380_pci_wait_interrupt(libusb3380_context_t *ctx, long timeoutms);


int usb3380_gpep_read(libusb3380_context_t* ctx, libusb3380_gpep_t ep,
					  uint8_t *data, int size, int* written, unsigned to);

int usb3380_gpep_write(libusb3380_context_t* ctx, libusb3380_gpep_t ep,
					   const uint8_t* data, int size, int* written,
					   unsigned to);

/* Async interfce */


enum {
	LIBUSB3380_MAX_TLP_SIZE = 256,
};

typedef struct libusb3380_pciout_buf {
	uint32_t ctrl;
	uint32_t addr;
	uint8_t data[LIBUSB3380_MAX_TLP_SIZE];
} libusb3380_pciout_buf_t;

typedef enum libusb3380_status {
	DQS_SUCCESS = 0,
	DQS_TIMEOUT = 1,
	DQS_ABORT = 2,
	DQS_PARTIAL = 3,
	DQS_CANCELLED = 4,
} libusb3380_status_t;


typedef struct libusb3380_as_base {
	/** number of bytes to IN or OUT (or MSI interrupt number awaiting) */
	unsigned size;

	/** number of miliseconds to the wait endpoint result */
	unsigned timeout;

	/** status of operation */
	libusb3380_status_t status;

	/** number of bytes INed or OUTed */
	unsigned written;
} libusb3380_as_base_t;


typedef enum libusb3380_pci_tlp_type {
	RC_PCIE_WRITE = 0,
	RC_PCIE_READ = 1,
} libusb3380_pci_tlp_type_t;

/* awaiting root complex transaction result */
typedef struct libusb3380_qpci_master {
	libusb3380_as_base_t base;

	libusb3380_pci_tlp_type_t type;
	union {
		libusb3380_pciout_buf_t out;
		uint32_t data[LIBUSB3380_MAX_TLP_SIZE / 4 + 2];
	};
} libusb3380_qpci_master_t;

typedef struct libusb3380_qcsr {
	libusb3380_as_base_t base;

	bool read;
	uint32_t data[2];
} libusb3380_qcsr_t;


struct libusb3380_qgpep;
typedef void (*on_gpep_cb_t)(const struct libusb3380_qgpep* gpep,
							 unsigned gpepno,
							 unsigned idx);

/* awaiting gpep data (device DMA in and out)  */
typedef struct libusb3380_qgpep {
	libusb3380_as_base_t base;

	/** Data block to be IN or OUT */
	uint8_t* pdata;

	on_gpep_cb_t cb_done;

	void* param;
} libusb3380_qgpep_t;

/* awaiting PCIe MSI interrupt */
typedef struct libusb3380_qmsi_int {
	libusb3380_as_base_t base;
} libusb3380_qmsi_int_t;

typedef struct libusb3380_configuration {
	unsigned gp_in_cnts[LIBUSB3380_GPEP_COUNT];
	unsigned gp_out_cnts[LIBUSB3380_GPEP_COUNT];
} libusb3380_configuration_t;

struct libusb3380_async_manager;
int usb3380_async_start(struct libusb3380_pcidev *dev,
						const struct libusb3380_configuration *configuration,
						struct libusb3380_async_manager** out);
int usb3380_async_stop(struct libusb3380_async_manager* mgr);

int usb3380_async_pci_write32(struct libusb3380_async_manager* mgr,
							  uint32_t addr,
							  const uint32_t *data,
							  unsigned dw_count);

int usb3380_async_pci_read32(struct libusb3380_async_manager* mgr,
							  uint32_t addr,
							  uint32_t *data,
							  unsigned dw_count);

int usb3380_async_await_msi(struct libusb3380_async_manager* mgr,
							unsigned num);

int usb3380_async_set_gpep_timeout(struct libusb3380_async_manager* mgr,
								   bool ep_in, libusb3380_gpep_t gpep,
								   unsigned idx,
								   unsigned to_ms);


/**
 * @brief usb3380_async_gpep_out_post
 * @param mgr
 * @param data
 * @param size
 * @return 0 success, error otherwise
 *
 * Functions post data to send and return immediatly if sending queue isn't
 * full, otherwise it bolcks untill we get some space in the queue
 */
int usb3380_async_gpep_out_post(struct libusb3380_async_manager* mgr,
								libusb3380_gpep_t ep_no, unsigned idx,
								const uint8_t* data, unsigned size,
								on_gpep_cb_t cb, void* param);

int usb3380_async_gpep_in_post(struct libusb3380_async_manager* mgr,
							   libusb3380_gpep_t ep_no, unsigned idx,
							   uint8_t* data, unsigned size,
							   on_gpep_cb_t cb, void* param);

int usb3380_async_gpep_cancel(struct libusb3380_async_manager* mgr,
							  bool ep_in,
							  libusb3380_gpep_t ep_no,
							  unsigned idx);


typedef void (*on_msi_cb_t)(void* param,
							int msinum,
							bool timedout);

int usb3380_msi_in_post(struct libusb3380_async_manager* mgr,
						unsigned timeoutms,
						on_msi_cb_t cb,
						void* param);
int usb3380_msi_in_cancel(struct libusb3380_async_manager* mgr);


#ifdef __cplusplus
};
#endif

#endif
