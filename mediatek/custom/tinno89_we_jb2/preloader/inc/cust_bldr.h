#ifndef CUST_BLDR_H
#define CUST_BLDR_H

#include "boot_device.h"
#include "uart.h"

/*=======================================================================*/
/* Pre-Loader Features                                                   */
/*=======================================================================*/
#ifdef MTK_EMMC_SUPPORT
#define CFG_BOOT_DEV                (BOOTDEV_SDMMC)
#else
#define CFG_BOOT_DEV                (BOOTDEV_NAND)
#endif
#define CFG_FPGA_PLATFORM           (0)
#define CFG_BATTERY_DETECT          (1)

#define CFG_UART_TOOL_HANDSHAKE     (1)
#define CFG_USB_TOOL_HANDSHAKE      (1)
#define CFG_USB_DOWNLOAD            (1)
#define CFG_PMT_SUPPORT             (1)

#define CFG_LOG_BAUDRATE            (921600)
#define CFG_META_BAUDRATE           (115200)
#define CFG_UART_LOG                (UART1)
#define CFG_UART_META               (UART1)

#define CFG_EMERGENCY_DL_SUPPORT    (1)
#define CFG_EMERGENCY_DL_TIMEOUT_MS (1000 * 30) /* 30 s */

/*=======================================================================*/
/* Misc Options                                                          */	
/*=======================================================================*/
#define FEATURE_MMC_ADDR_TRANS

#endif /* CUST_BLDR_H */
