/* linux/drivers/hwmon/adxl345.c
 *
 * (C) Copyright 2008 
 * MediaTek <www.mediatek.com>
 *
 * KXTJ2 driver for MT6516
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA  KXTJ2
 */
#ifndef KXTJ2_H
#define KXTJ2_H

extern struct acc_hw* kxtj2_get_cust_acc_hw(void) ;
	 
#include <linux/ioctl.h>
	 
#define KXTJ2_I2C_SLAVE_ADDR		0x0F << 1
	 
 /* KXTJ2 Register Map  (Please refer to KXTJ2 Specifications) */
#define KXTJ2_REG_DEVID			0x0F
#define	KXTJ2_REG_BW_RATE			0x21
#define KXTJ2_REG_POWER_CTL		0x1B
#define KXTJ2_REG_CTL_REG2		0x1D		//Ivan change to reg2
#define KXTJ2_DCST_RESP			0x0C
#define KXTJ2_REG_DATA_FORMAT		0x1B
#define KXTJ2_REG_DATA_RESOLUTION		0x1B
#define KXTJ2_RANGE_DATA_RESOLUTION_MASK	0x40
#define KXTJ2_REG_DATAX0			0x06	 
#define KXTJ2_FIXED_DEVID			0x09	 
#define KXTJ2_BW_200HZ				0x04
#define KXTJ2_BW_100HZ				0x03
#define KXTJ2_BW_50HZ				0x02	 
#define KXTJ2_MEASURE_MODE		0x80		 
#define KXTJ2_RANGE_MASK		0x18
#define KXTJ2_RANGE_2G			0x00
#define KXTJ2_RANGE_4G			0x08
#define KXTJ2_RANGE_8G			0x10
#define KXTJ2_REG_INT_ENABLE	0x1E

#define KXTJ2_SELF_TEST           0x10
	 	 
	 
#define KXTJ2_SUCCESS						0
#define KXTJ2_ERR_I2C						-1
#define KXTJ2_ERR_STATUS					-3
#define KXTJ2_ERR_SETUP_FAILURE				-4
#define KXTJ2_ERR_GETGSENSORDATA			-5
#define KXTJ2_ERR_IDENTIFICATION			-6
	 
	 
	 
#define KXTJ2_BUFSIZE				256
	 
#endif

