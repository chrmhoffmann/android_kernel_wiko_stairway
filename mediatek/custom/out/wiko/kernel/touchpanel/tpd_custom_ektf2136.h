/* Copyright Statement:
 *
 * This software/firmware and related documentation ("MediaTek Software") are
 * protected under relevant copyright laws. The information contained herein
 * is confidential and proprietary to MediaTek Inc. and/or its licensors.
 * Without the prior written permission of MediaTek inc. and/or its licensors,
 * any reproduction, modification, use or disclosure of MediaTek Software,
 * and information contained herein, in whole or in part, shall be strictly prohibited.
 */
/* MediaTek Inc. (C) 2010. All rights reserved.
 *
 * BY OPENING THIS FILE, RECEIVER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES
 * THAT THE SOFTWARE/FIRMWARE AND ITS DOCUMENTATIONS ("MEDIATEK SOFTWARE")
 * RECEIVED FROM MEDIATEK AND/OR ITS REPRESENTATIVES ARE PROVIDED TO RECEIVER ON
 * AN "AS-IS" BASIS ONLY. MEDIATEK EXPRESSLY DISCLAIMS ANY AND ALL WARRANTIES,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE OR NONINFRINGEMENT.
 * NEITHER DOES MEDIATEK PROVIDE ANY WARRANTY WHATSOEVER WITH RESPECT TO THE
 * SOFTWARE OF ANY THIRD PARTY WHICH MAY BE USED BY, INCORPORATED IN, OR
 * SUPPLIED WITH THE MEDIATEK SOFTWARE, AND RECEIVER AGREES TO LOOK ONLY TO SUCH
 * THIRD PARTY FOR ANY WARRANTY CLAIM RELATING THERETO. RECEIVER EXPRESSLY ACKNOWLEDGES
 * THAT IT IS RECEIVER'S SOLE RESPONSIBILITY TO OBTAIN FROM ANY THIRD PARTY ALL PROPER LICENSES
 * CONTAINED IN MEDIATEK SOFTWARE. MEDIATEK SHALL ALSO NOT BE RESPONSIBLE FOR ANY MEDIATEK
 * SOFTWARE RELEASES MADE TO RECEIVER'S SPECIFICATION OR TO CONFORM TO A PARTICULAR
 * STANDARD OR OPEN FORUM. RECEIVER'S SOLE AND EXCLUSIVE REMEDY AND MEDIATEK'S ENTIRE AND
 * CUMULATIVE LIABILITY WITH RESPECT TO THE MEDIATEK SOFTWARE RELEASED HEREUNDER WILL BE,
 * AT MEDIATEK'S OPTION, TO REVISE OR REPLACE THE MEDIATEK SOFTWARE AT ISSUE,
 * OR REFUND ANY SOFTWARE LICENSE FEES OR SERVICE CHARGE PAID BY RECEIVER TO
 * MEDIATEK FOR SUCH MEDIATEK SOFTWARE AT ISSUE.
 *
 * The following software/firmware and/or related documentation ("MediaTek Software")
 * have been modified by MediaTek Inc. All revisions are subject to any receiver's
 * applicable license agreements with MediaTek Inc.
 */

#ifndef TOUCHPANEL_H__
#define TOUCHPANEL_H__

/* Pre-defined definition */
#include "tpd.h"
#include <linux/interrupt.h>
#include <cust_eint.h>
#include <linux/i2c.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/rtpm_prio.h>
#include <linux/wait.h>
#include <linux/time.h>
#include <linux/delay.h>
#include <linux/miscdevice.h>
#include <linux/cdev.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <mach/mt_pm_ldo.h>
#include <mach/mt_typedefs.h>
#include <mach/mt_boot.h>
#include "cust_gpio_usage.h"
#define TPD_USE_VIRTUAL_KEY
#define TPD_TYPE_CAPACITIVE
#define TPD_TYPE_RESISTIVE
#define TPD_I2C_NUMBER           0
#define SUPORT_5_POINTS
#define tpd_driver_name 	"Ektf2136"
#define TP_LIB_NAME     "eKTF2136.i"
#define  EKTF_POWER 	MT65XX_POWER_LDO_VGP4
#define slaver_addr			 0x2a
#define ELAN_ESD_PROTECT               

#define SUPPORT_UPDATE_FW
#define SW_FIRMWARE_UPDATE
#define TPD_WAKEUP_TRIAL         60
#define TPD_WAKEUP_DELAY         100

//Ivan Copy from GT9xx
#define GTP_ADDR_LENGTH             2
#define I2C_MASTER_CLOCK              300
#define MAX_TRANSACTION_LENGTH        8
#define MAX_I2C_TRANSFER_SIZE         (MAX_TRANSACTION_LENGTH - GTP_ADDR_LENGTH)


#define TPD_DELAY                (2*HZ/100)
#define TPD_CUST_RES_X          	720
#define TPD_CUST_RES_Y           1280

#define EKTF_WIDTH      704
#define EKTF_HEIGHT     1280//1255//1344
#define TPD_CALIBRATION_MATRIX  {962,0,0,0,1600,0,0,0};

//#define TPD_HAVE_CALIBRATION
#define TPD_HAVE_BUTTON
//#define TPD_HAVE_TREMBLE_ELIMINATION

//#define TPD_HAVE_POWER_ON_OFF

#define PRESSURE_FACTOR	10
#define TPD_BUTTON_HEIGHT		1280

#define VIRTUAL_KEY_DEB_TIME	3
#define TPD_Y_OFFSET		10

#define TPD_KEY_COUNT           3
#define TPD_KEYS                {KEY_MENU,KEY_HOMEPAGE, KEY_BACK}
static const int TPD_KEYSFACTORY[TPD_KEY_COUNT] =  {KEY_F1, KEY_F2, KEY_F3};

#define TPD_YMAX		1408		//Truely, BYD = 516
#define TPD_BUTTON_SIZE_HEIGHT  (TPD_YMAX - TPD_BUTTON_HEIGHT - TPD_Y_OFFSET)
#define TPD_BUTTON_Y_CENTER   	(TPD_BUTTON_HEIGHT + (TPD_YMAX - TPD_BUTTON_HEIGHT)/2 + TPD_Y_OFFSET)

//edit by Magnum 2012-8-8  virtulkey location
#define TPD_CUST_KEY_Y      	(TPD_CUST_RES_Y + (TPD_YMAX - TPD_BUTTON_HEIGHT)/2 )   //510
#define X_UNIT                  (TPD_CUST_RES_X / (2*TPD_KEY_COUNT))  
#define TPD_CUST_KEY_X1         (X_UNIT)      // 40
#define TPD_CUST_KEY_X2         (X_UNIT * 3)    //2           
#define TPD_CUST_KEY_X3         (X_UNIT * 5)	   //

//TP virtual key customization

// |                                                                                                   |
// |                                                                                                   |  Touch Pad area ( H < 480)
//  ---------------------------------------------------------------------------------------------------
// |                                           TPD_Y_OFFSET                                            |  Virtual key area ( H > 480)
// |---------------------------------------------------------------------------------------------------
// |TPD_B1_FP | [TPD_B1_W] | TPD_B2_FP | [TPD_B2_W] | TPD_B3_FP | [TPD_B3_W] | TPD_B4_FP | [TPD_B4_W]  |  
// -----------------------------------------------------------------------------------------------------

#define TPD_B1_FP	0		//Button 1 pad space
#define TPD_B1_W		240		//Button 1 Width
#define TPD_B2_FP	0		//Button 2 pad space
#define TPD_B2_W		240		//Button 2 Width
#define TPD_B3_FP	0		//Button 3 pad space
#define TPD_B3_W		240		//Button 3 Width
#define TPD_B4_FP	0		//Button 4 pad space
#define TPD_B4_W		75		//Button 4 width
//#define TPD_KEYS_DIM            {{80,850,160,TPD_BUTTON_HEIGH},{240,850,160,TPD_BUTTON_HEIGH},{400,850,160,TPD_BUTTON_HEIGH}}


//-------------------------------------------------------------------------
#define TPD_BUTTON1_X_CENTER	TPD_B1_FP + TPD_B1_W/2
#define TPD_BUTTON2_X_CENTER	TPD_B1_FP + TPD_B1_W + TPD_B2_FP + TPD_B2_W/2
#define TPD_BUTTON3_X_CENTER	TPD_B1_FP + TPD_B1_W + TPD_B2_FP + TPD_B2_W + TPD_B3_FP + TPD_B3_W/2
#define TPD_BUTTON4_X_CENTER	TPD_B1_FP + TPD_B1_W + TPD_B2_FP + TPD_B2_W + TPD_B3_FP + TPD_B3_W + TPD_B4_FP + TPD_B4_W/2


#define TPD_KEYS_DIM            {{TPD_BUTTON1_X_CENTER, TPD_BUTTON_Y_CENTER, TPD_B1_W, TPD_BUTTON_SIZE_HEIGHT},	\
				 {TPD_BUTTON2_X_CENTER, TPD_BUTTON_Y_CENTER, TPD_B2_W, TPD_BUTTON_SIZE_HEIGHT},	\
				 {TPD_BUTTON3_X_CENTER, TPD_BUTTON_Y_CENTER, TPD_B3_W, TPD_BUTTON_SIZE_HEIGHT}}

extern struct i2c_client * ektf2136_i2c_client ;
extern char ektf_vendor_version ;
extern char ektf_panel_version ;
extern int FW_VERSION;
extern int work_lock;
void tpd_wr_datas(unsigned char *buff, int count);
void tpd_rd_datas(unsigned char *buff, int count);
void tpd_reset();
int ektf2136_ts_get_fw_version(void);
int ektf2136_register_misc(void);
#endif /* TOUCHPANEL_H__ */
