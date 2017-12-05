cmake_minimum_required(VERSION 2.8)
project(libusb3380 C)

# Set the version information here
set(MAJOR_VERSION 0)
set(API_COMPAT    0)
set(MINOR_VERSION 1)
set(MAINT_VERSION git)

set(LIBVER  "${MAJOR_VERSION}.${API_COMPAT}.${MINOR_VERSION}")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules/")

find_package(libusb-1.0 REQUIRED)


set(LIBUSB3380_LIBRARY_DIR      lib${LIB_SUFFIX})
set(LIBUSB3380_INCLUDE_DIR      include)

set(CPACK_PACKAGE_NAME                "libusb3380")
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "USB3380 library")
set(CPACK_PACKAGE_VENDOR              "Fairwaves, Inc.")
set(CPACK_PACKAGE_CONTACT             "http://fairwaves.co/wp/contact-us/")
set(CPACK_PACKAGE_VERSION             ${LIBVER}-1)
set(CPACK_DEBIAN_PACKAGE_DEPENDS      "libc6 (>= 2.17), libusb-1.0")

CONFIGURE_FILE(
    ${CMAKE_CURRENT_SOURCE_DIR}/libusb3380.pc.in
    ${CMAKE_CURRENT_BINARY_DIR}/libusb3380.pc
@ONLY)

INSTALL(
    FILES ${CMAKE_CURRENT_BINARY_DIR}/libusb3380.pc
    DESTINATION ${LIBUSB3380_LIBRARY_DIR}/pkgconfig
)


include_directories(${LIBUSB_1_INCLUDE_DIRS})
include_directories(.)

add_library(usb3380 SHARED libusb3380.c)
target_link_libraries(usb3380 ${LIBUSB_1_LIBRARIES} ${SYSTEM_LIBS})
set_target_properties(usb3380 PROPERTIES VERSION ${LIBVER} SOVERSION ${MAJOR_VERSION})

install(TARGETS usb3380 DESTINATION ${LIBUSB3380_LIBRARY_DIR})
install(FILES libusb3380.h DESTINATION ${LIBUSB3380_INCLUDE_DIR})

add_executable(test_libusb3380 test_libusb3380.c)
target_link_libraries(test_libusb3380 usb3380 m ${SYSTEM_LIBS})


