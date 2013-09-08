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

/*****************************************************************************
*  Copyright Statement:
*  --------------------
*  This software is protected by Copyright and the information contained
*  herein is confidential. The software may not be copied and the information
*  contained herein may not be used or disclosed except with the written
*  permission of MediaTek Inc. (C) 2008
*
*  BY OPENING THIS FILE, BUYER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES
*  THAT THE SOFTWARE/FIRMWARE AND ITS DOCUMENTATIONS ("MEDIATEK SOFTWARE")
*  RECEIVED FROM MEDIATEK AND/OR ITS REPRESENTATIVES ARE PROVIDED TO BUYER ON
*  AN "AS-IS" BASIS ONLY. MEDIATEK EXPRESSLY DISCLAIMS ANY AND ALL WARRANTIES,
*  EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF
*  MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE OR NONINFRINGEMENT.
*  NEITHER DOES MEDIATEK PROVIDE ANY WARRANTY WHATSOEVER WITH RESPECT TO THE
*  SOFTWARE OF ANY THIRD PARTY WHICH MAY BE USED BY, INCORPORATED IN, OR
*  SUPPLIED WITH THE MEDIATEK SOFTWARE, AND BUYER AGREES TO LOOK ONLY TO SUCH
*  THIRD PARTY FOR ANY WARRANTY CLAIM RELATING THERETO. MEDIATEK SHALL ALSO
*  NOT BE RESPONSIBLE FOR ANY MEDIATEK SOFTWARE RELEASES MADE TO BUYER'S
*  SPECIFICATION OR TO CONFORM TO A PARTICULAR STANDARD OR OPEN FORUM.
*
*  BUYER'S SOLE AND EXCLUSIVE REMEDY AND MEDIATEK'S ENTIRE AND CUMULATIVE
*  LIABILITY WITH RESPECT TO THE MEDIATEK SOFTWARE RELEASED HEREUNDER WILL BE,
*  AT MEDIATEK'S OPTION, TO REVISE OR REPLACE THE MEDIATEK SOFTWARE AT ISSUE,
*  OR REFUND ANY SOFTWARE LICENSE FEES OR SERVICE CHARGE PAID BY BUYER TO
*  MEDIATEK FOR SUCH MEDIATEK SOFTWARE AT ISSUE.
*
*  THE TRANSACTION CONTEMPLATED HEREUNDER SHALL BE CONSTRUED IN ACCORDANCE
*  WITH THE LAWS OF THE STATE OF CALIFORNIA, USA, EXCLUDING ITS CONFLICT OF
*  LAWS PRINCIPLES.  ANY DISPUTES, CONTROVERSIES OR CLAIMS ARISING THEREOF AND
*  RELATED THERETO SHALL BE SETTLED BY ARBITRATION IN SAN FRANCISCO, CA, UNDER
*  THE RULES OF THE INTERNATIONAL CHAMBER OF COMMERCE (ICC).
*
*****************************************************************************/
#ifdef BUILD_LK
	#include <platform/mt_gpio.h>

	#define LCM_PRINT printf
#define LCM_DBG(fmt, arg...) \
	LCM_PRINT("[LCM-LT4015W-DSI] %s (line:%d) :" fmt "\r\n", __func__, __LINE__, ## arg)

#else

#include <linux/string.h>

#if defined(BUILD_UBOOT)
	#include <asm/arch/mt_gpio.h>
	#define LCM_PRINT printf
	#ifndef KERN_INFO
		#define KERN_INFO
	#endif
#else
	#include <linux/kernel.h>
	#include <mach/mt_gpio.h>
	#define LCM_PRINT printk
#endif

#if 1
#define LCM_DBG(fmt, arg...) \
	LCM_PRINT("[LCM-NT35510-DSI] %s (line:%d) :" fmt "\r\n", __func__, __LINE__, ## arg)
#else
#define LCM_DBG(fmt, arg...) do {} while (0)
#endif
#endif
#include "lcm_drv.h"

// ---------------------------------------------------------------------------
//  Local Constants
// ---------------------------------------------------------------------------

#define FRAME_WIDTH  										(480)
#define FRAME_HEIGHT 										(854)
#define LCM_ID       (0x80)

#define REGFLAG_DELAY             							0XFE
#define REGFLAG_END_OF_TABLE      							0xFF   // END OF REGISTERS MARKER

// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------

static LCM_UTIL_FUNCS lcm_util = {0};

#define SET_RESET_PIN(v)    								(lcm_util.set_reset_pin((v)))

#define UDELAY(n) 											(lcm_util.udelay(n))
#define MDELAY(n) 											(lcm_util.mdelay(n))


// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------

#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)	lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update);
#define dsi_set_cmdq(pdata, queue_size, force_update)		lcm_util.dsi_set_cmdq(pdata, queue_size, force_update);
#define wrtie_cmd(cmd)										lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)					lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg											lcm_util.dsi_read_reg()
#define read_reg_v2(cmd, buffer, buffer_size)   				lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)
#ifndef GPIO_LCD_BACKLIGHT_EN_PIN
#define GPIO_LCD_BACKLIGHT_EN_PIN GPIO68
#define GPIO_LCD_BACKLIGHT_EN_PIN_M_GPIO GPIO_MODE_GPIO
#endif       


static void lcm_init(void);

static struct LCM_setting_table {
    unsigned cmd;
    unsigned char count;
    unsigned char para_list[64];
};

static void config_gpio(void)
{
	LCM_DBG();
	mt_set_gpio_mode(GPIO_LCD_BACKLIGHT_EN_PIN, GPIO_LCD_BACKLIGHT_EN_PIN_M_GPIO);
	mt_set_gpio_dir(GPIO_LCD_BACKLIGHT_EN_PIN, GPIO_DIR_OUT);
	mt_set_gpio_pull_enable(GPIO_LCD_BACKLIGHT_EN_PIN, GPIO_PULL_DISABLE);
}

static void light_lcd_on(int on)
{
	LCM_DBG("on=%d", on);
	if ( on ){
		mt_set_gpio_out(GPIO_LCD_BACKLIGHT_EN_PIN, GPIO_OUT_ONE);			
	}
	else{
		mt_set_gpio_out(GPIO_LCD_BACKLIGHT_EN_PIN, GPIO_OUT_ZERO);						
	}
}
#if 0
static struct LCM_setting_table lcm_initialization_setting[] = {
	
	/*
	Note :

	Data ID will depends on the following rule.
	
		count of parameters > 1	=> Data ID = 0x39
		count of parameters = 1	=> Data ID = 0x15
		count of parameters = 0	=> Data ID = 0x05

	Structure Format :

	{DCS command, count of parameters, {parameter list}}
	{REGFLAG_DELAY, milliseconds of time, {}},

	...

	Setting ending by predefined flag
	
	{REGFLAG_END_OF_TABLE, 0x00, {}}
	*/

	{0xF0,	5,	{0x55, 0xaa, 0x52, 0x08, 0x01}},
	{REGFLAG_DELAY, 1, {}},

	{0xD1,	52,	{0x00, 0x00, 0x00, 0x16,
				 0x00, 0x42, 0x00, 0x61,
				 0x00, 0x74, 0x00, 0x97,
				 0x00, 0xAD, 0x00, 0xDE,
				 0x01, 0x00, 0x01, 0x26,
				 0x01, 0x50, 0x01, 0x87,
				 0x01, 0xB3, 0x01, 0xB6,
			 	 0x01, 0xDC, 0x02, 0x04,
				 0x02, 0x1C, 0x02, 0x34,
				 0x02, 0x4E, 0x02, 0x8A,
				 0x02, 0xC2, 0x03, 0x04,
				 0x03, 0x2E, 0x03, 0x74,
				 0x03, 0xEB, 0x03, 0xFF}},
	{REGFLAG_DELAY, 1, {}},

	{0xD2,	52,	{0x00, 0x00, 0x00, 0x16,
				 0x00, 0x42, 0x00, 0x61,
				 0x00, 0x74, 0x00, 0x97,
				 0x00, 0xAD, 0x00, 0xDE,
				 0x01, 0x00, 0x01, 0x26,
				 0x01, 0x50, 0x01, 0x87,
				 0x01, 0xB3, 0x01, 0xB6,
			 	 0x01, 0xDC, 0x02, 0x04,
				 0x02, 0x1C, 0x02, 0x34,
				 0x02, 0x4E, 0x02, 0x8A,
				 0x02, 0xC2, 0x03, 0x04,
				 0x03, 0x2E, 0x03, 0x74,
				 0x03, 0xEB, 0x03, 0xFF}},
	{REGFLAG_DELAY, 1, {}},

	{0xD3,	52,	{0x00, 0x00, 0x00, 0x16,
				 0x00, 0x42, 0x00, 0x61,
				 0x00, 0x74, 0x00, 0x97,
				 0x00, 0xAD, 0x00, 0xDE,
				 0x01, 0x00, 0x01, 0x26,
				 0x01, 0x50, 0x01, 0x87,
				 0x01, 0xB3, 0x01, 0xB6,
			 	 0x01, 0xDC, 0x02, 0x04,
				 0x02, 0x1C, 0x02, 0x34,
				 0x02, 0x4E, 0x02, 0x8A,
				 0x02, 0xC2, 0x03, 0x04,
				 0x03, 0x2E, 0x03, 0x74,
				 0x03, 0xEB, 0x03, 0xFF}},
	{REGFLAG_DELAY, 1, {}},

	{0xD4,	52,	{0x00, 0x00, 0x00, 0x16,
				 0x00, 0x42, 0x00, 0x61,
				 0x00, 0x74, 0x00, 0x97,
				 0x00, 0xAD, 0x00, 0xDE,
				 0x01, 0x00, 0x01, 0x26,
				 0x01, 0x50, 0x01, 0x87,
				 0x01, 0xB3, 0x01, 0xB6,
			 	 0x01, 0xDC, 0x02, 0x04,
				 0x02, 0x1C, 0x02, 0x34,
				 0x02, 0x4E, 0x02, 0x8A,
				 0x02, 0xC2, 0x03, 0x04,
				 0x03, 0x2E, 0x03, 0x74,
				 0x03, 0xEB, 0x03, 0xFF}},
	{REGFLAG_DELAY, 1, {}},

	{0xD5,	52,	{0x00, 0x00, 0x00, 0x16,
				 0x00, 0x42, 0x00, 0x61,
				 0x00, 0x74, 0x00, 0x97,
				 0x00, 0xAD, 0x00, 0xDE,
				 0x01, 0x00, 0x01, 0x26,
				 0x01, 0x50, 0x01, 0x87,
				 0x01, 0xB3, 0x01, 0xB6,
			 	 0x01, 0xDC, 0x02, 0x04,
				 0x02, 0x1C, 0x02, 0x34,
				 0x02, 0x4E, 0x02, 0x8A,
				 0x02, 0xC2, 0x03, 0x04,
				 0x03, 0x2E, 0x03, 0x74,
				 0x03, 0xEB, 0x03, 0xFF}},
	{REGFLAG_DELAY, 1, {}},

	{0xD6,	52,	{0x00, 0x00, 0x00, 0x16,
				 0x00, 0x42, 0x00, 0x61,
				 0x00, 0x74, 0x00, 0x97,
				 0x00, 0xAD, 0x00, 0xDE,
				 0x01, 0x00, 0x01, 0x26,
				 0x01, 0x50, 0x01, 0x87,
				 0x01, 0xB3, 0x01, 0xB6,
			 	 0x01, 0xDC, 0x02, 0x04,
				 0x02, 0x1C, 0x02, 0x34,
				 0x02, 0x4E, 0x02, 0x8A,
				 0x02, 0xC2, 0x03, 0x04,
				 0x03, 0x2E, 0x03, 0x74,
				 0x03, 0xEB, 0x03, 0xFF}},
	{REGFLAG_DELAY, 10, {}},

	{0xB9,	3,	{0x24, 0x24, 0x24}},
	{REGFLAG_DELAY, 1, {}},

	{0xBA,	3,	{0x24, 0x24, 0x24}},
	{REGFLAG_DELAY, 1, {}},

	{0xBC,	3,	{0x00, 0x88, 0x01}},
	{REGFLAG_DELAY, 1, {}},

	{0xBD,	3,	{0x00, 0x88, 0x01}},
	{REGFLAG_DELAY, 1, {}},

	{0xBE,	2,	{0x00, 0x78}},
	{REGFLAG_DELAY, 1, {}},

	{0xF0,	5,	{0x55, 0xAA, 0x52, 0x08, 0x00}},
	{REGFLAG_DELAY, 1, {}},

	{0xB1,	2,	{0xEF, 0x00}},
	{REGFLAG_DELAY, 1, {}},

	{0xBC,	3,	{0x05, 0x05, 0x05}},
	{REGFLAG_DELAY, 1, {}},

    // Sleep Out
	{0x11, 1, {0x00}},
	// LCM driver IC specifies 15ms needed after sleep out. But we need more delay time to make sure latest RAM data has been refreshed to screen.
	{REGFLAG_DELAY, 200, {}},

	{0x2A,	4,	{0x00, 0x00, 0x01, 0xDF}},
	{REGFLAG_DELAY, 1, {}},

	{0x2B,	4,	{0x00, 0x00, 0x03, 0x55}},
	{REGFLAG_DELAY, 1, {}},

    // Display on
	{0x29, 1, {0x00}},
	{REGFLAG_DELAY, 200, {}},

	// Display on
	{0x2c, 1, {0x00}},
	{REGFLAG_DELAY, 1, {}},

	// Note
	// Strongly recommend not to set Sleep out / Display On here. That will cause messed frame to be shown as later the backlight is on.


	// Setting ending by predefined flag
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

static struct LCM_setting_table lcm_initialization_setting[] = {
	
	/*
	Note :

	Data ID will depends on the following rule.
	
		count of parameters > 1	=> Data ID = 0x39
		count of parameters = 1	=> Data ID = 0x15
		count of parameters = 0	=> Data ID = 0x05

	Structure Format :

	{DCS command, count of parameters, {parameter list}}
	{REGFLAG_DELAY, milliseconds of time, {}},

	...

	Setting ending by predefined flag
	
	{REGFLAG_END_OF_TABLE, 0x00, {}}
	*/

	//#Enable Page1
	{0xF0,	5,	{0x55,0xAA,0x52,0x08,0x01}},
	{REGFLAG_DELAY, 1, {}},
	//# AVDD: manual,
	{0xB6,	3,	{0x34,0x34,0x34}},
	{REGFLAG_DELAY, 1, {}},
	{0xB0,	3,	{0x0C,0x0C,0x0C}},
	{REGFLAG_DELAY, 1, {}},
	//# AVEE: manual, ©\6V
	{0xB7,	3,	{0x24,0x24,0x24}},
	{REGFLAG_DELAY, 1, {}},
	{0xB1,	3,	{0x0C,0x0C,0x0C}},
	{REGFLAG_DELAY, 1, {}},

	//#Power Control for VCL
	{0xB8,	1,	{0x34}},
	{REGFLAG_DELAY, 1, {}},
	{0xB2,	1,	{0x00}},
	{REGFLAG_DELAY, 1, {}},
	
	//# VGH: Clamp Enable,
	{0xB9,	3,	{0x24,0x24,0x24}},//{0x34,0x34,0x34}
	{REGFLAG_DELAY, 1, {}},
	{0xB3,	3,	{0x08,0x08,0x08}},
	{REGFLAG_DELAY, 1, {}},

	//# VGL(LVGL):
	{0xBA,	3,	{0x24,0x24,0x24}},//{0x14,0x14,0x14}
	{REGFLAG_DELAY, 1, {}},
	
	//# VGL_REG(VGLO)
	{0xB5,	3,	{0x08,0x08,0x08}},
	{REGFLAG_DELAY, 1, {}},
	
	//# VGMP/VGSP:
	{0xBC,	3,	{0x00,0x08,0x00}},//{0x00,0x80,0x00}
	{REGFLAG_DELAY, 1, {}},
	
	//# VGMN/VGSN
	{0xBD,	3,	{0x00,0x08,0x00}},//{0x00,0x80,0x00}
	{REGFLAG_DELAY, 1, {}},
	
	//# VCOM=©\0.1
	{0xBE,	2,	{0x00,0x48}},//{0x00,0x2F}
	{REGFLAG_DELAY, 1, {}},

	//#R+
	{0xD1,	52,	{0x00,0x37,0x00,0x53,0x00,0x79,0x00,0x97,0x00,0xB1,
						 0x00,0xD5,0x00,0xF4,0x01,0x23,0x01,0x49,0x01,0x87,
						 0x01,0xB6,0x02,0x00,0x02,0x3B,0x02,0x3D,0x02,0x75,
						 0x02,0xB1,0x02,0xD5,0x03,0x09,0x03,0x28,0x03,0x52,
						 0x03,0x6B,0x03,0x8D,0x03,0xA2,0x03,0xBB,0x03,0xC1,
						 0x03,0xC1}},
	{REGFLAG_DELAY, 1, {}},
	//#G+
	{0xD2,	52,	{0x00,0x37,0x00,0x53,0x00,0x79,0x00,0x97,0x00,0xB1,
						 0x00,0xD5,0x00,0xF4,0x01,0x23,0x01,0x49,0x01,0x87,
						 0x01,0xB6,0x02,0x00,0x02,0x3B,0x02,0x3D,0x02,0x75,
						 0x02,0xB1,0x02,0xD5,0x03,0x09,0x03,0x28,0x03,0x52,
						 0x03,0x6B,0x03,0x8D,0x03,0xA2,0x03,0xBB,0x03,0xC1,
						 0x03,0xC1}},
	{REGFLAG_DELAY, 1, {}},
	//#B+
	{0xD3,	52,	{0x00,0x37,0x00,0x53,0x00,0x79,0x00,0x97,0x00,0xB1,
						 0x00,0xD5,0x00,0xF4,0x01,0x23,0x01,0x49,0x01,0x87,
						 0x01,0xB6,0x02,0x00,0x02,0x3B,0x02,0x3D,0x02,0x75,
						 0x02,0xB1,0x02,0xD5,0x03,0x09,0x03,0x28,0x03,0x52,
						 0x03,0x6B,0x03,0x8D,0x03,0xA2,0x03,0xBB,0x03,0xC1,
						 0x03,0xC1}},
	{REGFLAG_DELAY, 1, {}},
	//#R-
	{0xD4,	52,	{0x00,0x37,0x00,0x53,0x00,0x79,0x00,0x97,0x00,0xB1,
						 0x00,0xD5,0x00,0xF4,0x01,0x23,0x01,0x49,0x01,0x87,
						 0x01,0xB6,0x02,0x00,0x02,0x3B,0x02,0x3D,0x02,0x75,
						 0x02,0xB1,0x02,0xD5,0x03,0x09,0x03,0x28,0x03,0x52,
						 0x03,0x6B,0x03,0x8D,0x03,0xA2,0x03,0xBB,0x03,0xC1,
						 0x03,0xC1}},
	{REGFLAG_DELAY, 1, {}},
	//#G-
	{0xD5,	52,	{0x00,0x37,0x00,0x53,0x00,0x79,0x00,0x97,0x00,0xB1,
						 0x00,0xD5,0x00,0xF4,0x01,0x23,0x01,0x49,0x01,0x87,
						 0x01,0xB6,0x02,0x00,0x02,0x3B,0x02,0x3D,0x02,0x75,
						 0x02,0xB1,0x02,0xD5,0x03,0x09,0x03,0x28,0x03,0x52,
						 0x03,0x6B,0x03,0x8D,0x03,0xA2,0x03,0xBB,0x03,0xC1,
						 0x03,0xC1}},
	{REGFLAG_DELAY, 1, {}},
	//#B-
	{0xD6,	52,	{0x00,0x37,0x00,0x53,0x00,0x79,0x00,0x97,0x00,0xB1,
						 0x00,0xD5,0x00,0xF4,0x01,0x23,0x01,0x49,0x01,0x87,
						 0x01,0xB6,0x02,0x00,0x02,0x3B,0x02,0x3D,0x02,0x75,
						 0x02,0xB1,0x02,0xD5,0x03,0x09,0x03,0x28,0x03,0x52,
						 0x03,0x6B,0x03,0x8D,0x03,0xA2,0x03,0xBB,0x03,0xC1,
						 0x03,0xC1}},
	{REGFLAG_DELAY, 10, {}},

	//#Enable Page0
	{0xF0,	5,	{0x55,0xAA,0x52,0x08,0x00}},
	{REGFLAG_DELAY, 1, {}},
	
	//# Color Enhance Setting
	//{0xB4,	1,	{0x00}},//diable
	//{REGFLAG_DELAY, 3, {}},
	{0xB4,	1,	{0x01}},//enable
	{REGFLAG_DELAY, 1, {}},
	
	//# RGB I/F Setting
	{0xB0,	5,	{0x00,0x05,0x02,0x05,0x02}},
	{REGFLAG_DELAY, 1, {}},
	
	//## SDT:
	{0xB6,	1,	{0x07}},//{0x05}
	{REGFLAG_DELAY, 1, {}},
	
	{0xB7,	2,	{0x71,0x71}},//{0x70,0x70}
	{REGFLAG_DELAY, 1, {}},
	
	{0xB8,	4,	{0x01,0x0A,0x0A,0x0A}},//{0x01,0x05,0x05,0x05}
	{REGFLAG_DELAY, 1, {}},
	
	//# Inversion: Column
	{0xBC,	3,	{0x00,0x00,0x00}},//{0x05,0x05,0x05}
	{REGFLAG_DELAY, 1, {}},
	
	//# BOE's Setting (default)
	{0xCC,	3,	{0x03,0x50,0x50}},
	{REGFLAG_DELAY, 1, {}},
	
	//# Display Timing:
	{0xBD,	5,	{0x01,0x84,0x07,0x31,0x00}},//{0x01,0x00,0x07,0x31,0x00}
	{REGFLAG_DELAY, 1, {}},
	//{0xBE,	5,	{0x01,0x84,0x07,0x31,0x00}},
	//{REGFLAG_DELAY, 3, {}},
	//{0xBF,	5,	{0x01,0x84,0x07,0x31,0x00}},
	//{REGFLAG_DELAY, 3, {}},

	{0xF0,	4,	{0xAA,0x55,0x25,0x01}},
	{REGFLAG_DELAY, 1, {}},
	
	{0x3A,	1,	{0x77}},
	{REGFLAG_DELAY, 1, {}},
	
	{0x11,	0,	{0x00}},
	{REGFLAG_DELAY, 120, {}},
	{0x29,	0,	{0x00}},
	{REGFLAG_DELAY, 10, {}},

	//#Enable Page 0
	{0xF0,	5,	{0x55,0xAA,0x52,0x08,0x00}},
	{REGFLAG_DELAY, 1, {}},
	
	//#VScan

	//(vodeo mode&command mode)
	//{0xB1,	2,	{0xF8,0x00}},
	//0512-87172138-7122, 15950021248
	//(only command mode)
	{0xB1,	2,	{0xE8,0x00}},
	{REGFLAG_DELAY, 1, {}},
	// Note
	// Strongly recommend not to set Sleep out / Display On here. That will cause messed frame to be shown as later the backlight is on.


	// Setting ending by predefined flag
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

#else
static struct LCM_setting_table lcm_initialization_setting[] = {
	
	/*
	Note :

	Data ID will depends on the following rule.
	
		count of parameters > 1	=> Data ID = 0x39
		count of parameters = 1	=> Data ID = 0x15
		count of parameters = 0	=> Data ID = 0x05

	Structure Format :

	{DCS command, count of parameters, {parameter list}}
	{REGFLAG_DELAY, milliseconds of time, {}},

	...

	Setting ending by predefined flag
	
	{REGFLAG_END_OF_TABLE, 0x00, {}}
	*/

	//#Enable Page1
	{0xF0,	5,	{0x55,0xAA,0x52,0x08,0x01}},
	{REGFLAG_DELAY, 1, {}},
	//# VGMP/VGMN/VOOM Setting, VGMP=4.8V  #VGSP=0.6125V
	{0xBC,	3,	{0x00,0x78,0x1A}},
	{REGFLAG_DELAY, 1, {}},
	//# VGMN=-4.8V  #VGSN=-0.6125V
	{0xBD,	3,	{0x00,0x78,0x1A}},
	{REGFLAG_DELAY, 1, {}},
	//#VCOM=
	{0xBE,	2,	{0x00,0x49}},
	{REGFLAG_DELAY, 1, {}},

	//#R+
	{0xD1,	52,	{0x00,0x00,0x00,0x13,0x00,0x27,0x00,0x46,0x00,0x6A,
			0x00,0xA4,0x00,0xD5,0x01,0x1E,0x01,0x53,0x01,0x9B,
			0x01,0xCB,0x02,0x16,0x02,0x4E,0x02,0x4F,0x02,0x7F,
			0x02,0xB3,0x02,0xCF,0x02,0xEE,0x03,0x01,0x03,0x1B,
			0x03,0x2A,0x03,0x40,0x03,0x50,0x03,0x67,0x03,0xA8,
			0x03,0xD8}},
	{REGFLAG_DELAY, 1, {}},
	//#G+
	{0xD2,	52,	{0x00,0x00,0x00,0x13,0x00,0x27,0x00,0x46,0x00,0x6A,
			0x00,0xA4,0x00,0xD5,0x01,0x1E,0x01,0x53,0x01,0x9B,
			0x01,0xCB,0x02,0x16,0x02,0x4E,0x02,0x4F,0x02,0x7F,
			0x02,0xB3,0x02,0xCF,0x02,0xEE,0x03,0x01,0x03,0x1B,
			0x03,0x2A,0x03,0x40,0x03,0x50,0x03,0x67,0x03,0xA8,
			0x03,0xD8}},
	{REGFLAG_DELAY, 1, {}},
	//#B+
	{0xD3,	52,	{0x00,0x00,0x00,0x13,0x00,0x27,0x00,0x46,0x00,0x6A,
			0x00,0xA4,0x00,0xD5,0x01,0x1E,0x01,0x53,0x01,0x9B,
			0x01,0xCB,0x02,0x16,0x02,0x4E,0x02,0x4F,0x02,0x7F,
			0x02,0xB3,0x02,0xCF,0x02,0xEE,0x03,0x01,0x03,0x1B,
			0x03,0x2A,0x03,0x40,0x03,0x50,0x03,0x67,0x03,0xA8,
			0x03,0xD8}},
	{REGFLAG_DELAY, 1, {}},
	//#R-
	{0xD4,	52,	{0x00,0x00,0x00,0x13,0x00,0x27,0x00,0x46,0x00,0x6A,
			0x00,0xA4,0x00,0xD5,0x01,0x1E,0x01,0x53,0x01,0x9B,
			0x01,0xCB,0x02,0x16,0x02,0x4E,0x02,0x4F,0x02,0x7F,
			0x02,0xB3,0x02,0xCF,0x02,0xEE,0x03,0x01,0x03,0x1B,
			0x03,0x2A,0x03,0x40,0x03,0x50,0x03,0x67,0x03,0xA8,
			0x03,0xD8}},
	{REGFLAG_DELAY, 1, {}},
	//#G-
	{0xD5,	52,	{0x00,0x00,0x00,0x13,0x00,0x27,0x00,0x46,0x00,0x6A,
			0x00,0xA4,0x00,0xD5,0x01,0x1E,0x01,0x53,0x01,0x9B,
			0x01,0xCB,0x02,0x16,0x02,0x4E,0x02,0x4F,0x02,0x7F,
			0x02,0xB3,0x02,0xCF,0x02,0xEE,0x03,0x01,0x03,0x1B,
			0x03,0x2A,0x03,0x40,0x03,0x50,0x03,0x67,0x03,0xA8,
			0x03,0xD8}},
	{REGFLAG_DELAY, 1, {}},
	//#B-
	{0xD6,	52,	{0x00,0x00,0x00,0x13,0x00,0x27,0x00,0x46,0x00,0x6A,
			0x00,0xA4,0x00,0xD5,0x01,0x1E,0x01,0x53,0x01,0x9B,
			0x01,0xCB,0x02,0x16,0x02,0x4E,0x02,0x4F,0x02,0x7F,
			0x02,0xB3,0x02,0xCF,0x02,0xEE,0x03,0x01,0x03,0x1B,
			0x03,0x2A,0x03,0x40,0x03,0x50,0x03,0x67,0x03,0xA8,
			0x03,0xD8}},
	{REGFLAG_DELAY, 10, {}},	
	{0xB0,	3,	{0x00,0x00,0x00}},
	{REGFLAG_DELAY, 1, {}},
	//# AVDD: manual,
	{0xB6,	3,	{0x36,0x36,0x36}},
	{REGFLAG_DELAY, 1, {}},
	//#Power Control for VCL
	{0xB8,	3,	{0x26,0x26,0x26}},
	{REGFLAG_DELAY, 1, {}},
	{0xB1,	3,	{0x00,0x00,0x00}},
	{REGFLAG_DELAY, 1, {}},
	//# AVEE: manual, ©\6V
	{0xB7,	3,	{0x26,0x26,0x26}},
	{REGFLAG_DELAY, 1, {}},
	{0xB9,	3,	{0x34,0x34,0x34}},//{0x34,0x34,0x34}
	{REGFLAG_DELAY, 1, {}},
	//# VGL(LVGL):
	{0xBA,	3,	{0x16,0x16,0x16}},//{0x14,0x14,0x14}
	{REGFLAG_DELAY, 1, {}},
	
	//#Enable Page0
	{0xF0,	5,	{0x55,0xAA,0x52,0x08,0x00}},
	{REGFLAG_DELAY, 1, {}},
	
	{0xB1,	1,	{0xFC}},
	{REGFLAG_DELAY, 1, {}},

	{0xB4,	1,	{0x10}},
	{REGFLAG_DELAY, 1, {}},
	
	//## SDT:
	{0xB6,	1,	{0x05}},//{0x05}
	{REGFLAG_DELAY, 1, {}},	
	
	{0xB7,	2,	{0x70,0x70}},//{0x70,0x70}
	{REGFLAG_DELAY, 1, {}},
	
	{0xB8,	4,	{0x01,0x0A,0x0A,0x0A}},//{0x01,0x05,0x05,0x05}
	{REGFLAG_DELAY, 1, {}},

	//# VGMP/VGSP:
	{0xBC,	3,	{0x05,0x05,0x05}},//{0x00,0x80,0x00}
	{REGFLAG_DELAY, 1, {}},
	
	//# VGMN/VGSN
	{0xBD,	5,	{0x01,0x84,0x07,0x31,0x00}},//{0x00,0x80,0x00}
	{REGFLAG_DELAY, 1, {}},
	
	//# VCOM=©\0.1
	{0xBE,	5,	{0x01,0x84,0x07,0x31,0x00}},//{0x00,0x80,0x00}
	{REGFLAG_DELAY, 1, {}},

	//# VCOM=©\0.1
	{0xBE,	5,	{0x01,0x84,0x07,0x31,0x00}},//{0x00,0x80,0x00}
	{REGFLAG_DELAY, 1, {}},

//#TE ON                       	
	{0x35,	1,	{0x00}},
	{REGFLAG_DELAY, 1, {}},

	{0x36,	1,	{0x00}},
	{REGFLAG_DELAY, 1, {}},
	
	{0x11,	0,	{0x00}},
	{REGFLAG_DELAY, 120, {}},
	
	{0x29,	0,	{0x00}},
	{REGFLAG_DELAY, 10, {}},

	{0x3A,	1,	{0x77}},
	{REGFLAG_DELAY, 10, {}},
	
	//#Enable Page 0
	{0xF0,	5,	{0x55,0xAA,0x52,0x08,0x00}},
	{REGFLAG_DELAY, 1, {}},

	{0xC7,	1,	{0x02}},
	{REGFLAG_DELAY, 10, {}},

	{0xC9,	1,	{0x11}},
	{REGFLAG_DELAY, 10, {}},
	
	{0x21,	1,	{0x00}},
	{REGFLAG_DELAY, 10, {}},
	
	//#VScan

	//(vodeo mode&command mode)
	//{0xB1,	2,	{0xF8,0x00}},
	//0512-87172138-7122, 15950021248
	//(only command mode)
//Ivan	{0xB1,	2,	{0xE8,0x00}},
//Ivan	{REGFLAG_DELAY, 1, {}},
	// Note
	// Strongly recommend not to set Sleep out / Display On here. That will cause messed frame to be shown as later the backlight is on.


	// Setting ending by predefined flag
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};
#endif

static struct LCM_setting_table lcm_set_window[] = {
	{0x2A,	4,	{0x00, 0x00, (FRAME_WIDTH>>8), (FRAME_WIDTH&0xFF)}},
	{0x2B,	4,	{0x00, 0x00, (FRAME_HEIGHT>>8), (FRAME_HEIGHT&0xFF)}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

static struct LCM_setting_table lcm_sleep_out_setting[] = {
	// Sleep Out
	{0x11, 1, {0x00}},
	{REGFLAG_DELAY, 120, {}},

	// Display ON
	{0x29, 1, {0x00}},
	{REGFLAG_DELAY, 20, {}},
	
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

static struct LCM_setting_table lcm_deep_sleep_mode_in_setting[] = {
	// Display off sequence
	{0x28, 1, {0x00}},
	{REGFLAG_DELAY, 10, {}},

	// Sleep Mode On
	{0x10, 1, {0x00}},
	{REGFLAG_DELAY, 120, {}},

	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

static struct LCM_setting_table lcm_backlight_level_setting[] = {
	{0x51, 1, {0xFF}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};


static struct LCM_setting_table lcm_sw_reset_setting[] = {
	// Sleep Out
	{0x01, 1, {0x00}},
	{REGFLAG_DELAY, 120, {}},	
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};


static void dsi_send_cmdq_tinno(unsigned cmd, unsigned char count, unsigned char *para_list, unsigned char force_update)
{
	unsigned int item[16];
	unsigned char dsi_cmd = (unsigned char)cmd;
	unsigned char dc;
	int index = 0, length = 0;
	
	memset(item,0,sizeof(item));
	if(count+1 > 60)
	{
		LCM_DBG("Exceed 16 entry\n");
		return;
	}
/*
	Data ID will depends on the following rule.
	
		count of parameters > 1	=> Data ID = 0x39
		count of parameters = 1	=> Data ID = 0x15
		count of parameters = 0	=> Data ID = 0x05
*/	
	if(count == 0)
	{
		item[0] = 0x0500 | (dsi_cmd<<16);
		length = 1;
	}
	else if(count == 1)
	{
		item[0] = 0x1500 | (dsi_cmd<<16) | (para_list[0]<<24);
		length = 1;
	}
	else
	{
		item[0] = 0x3902 | ((count+1)<<16);//Count include command.
		++length;
		while(1)
		{
			if (index == count+1)
				break;
			if ( 0 == index ){
				dc = cmd;
			}else{
				dc = para_list[index-1];
			}
			item[index/4+1] |= (dc<<(8*(index%4)));
			if ( index%4 == 0 ) ++length;
			++index;
		}
	}
	
	dsi_set_cmdq(&item, length, force_update);

}

static void push_table(struct LCM_setting_table *table, unsigned int count, unsigned char force_update)
{
	unsigned int i;

	for(i = 0; i < count; i++) {
		
	    unsigned cmd;
	    cmd = table[i].cmd;
		
	    switch (cmd) {
			
	        case REGFLAG_DELAY :
	            MDELAY(table[i].count);
	            break;
				
	        case REGFLAG_END_OF_TABLE :
	            break;
				
	        default:
//				dsi_send_cmdq_tinno(cmd, table[i].count, table[i].para_list, force_update);
				dsi_set_cmdq_V2(cmd, table[i].count, table[i].para_list, force_update);
	   	}
	}

}

// ---------------------------------------------------------------------------
//  LCM Driver Implementations
// ---------------------------------------------------------------------------

static void lcm_set_util_funcs(const LCM_UTIL_FUNCS *util)
{
	LCM_DBG();
	memcpy(&lcm_util, util, sizeof(LCM_UTIL_FUNCS));
}

static void lcm_get_params(LCM_PARAMS *params)
{
	LCM_DBG();
	memset(params, 0, sizeof(LCM_PARAMS));

	params->type   = LCM_TYPE_DSI;

	params->width  = FRAME_WIDTH;
	params->height = FRAME_HEIGHT;

	// enable tearing-free
	params->dbi.te_mode 				= LCM_DBI_TE_MODE_DISABLED;
	params->dbi.te_edge_polarity		= LCM_POLARITY_RISING;

	params->dsi.mode   = CMD_MODE;

	// DSI
	/* Command mode setting */
	params->dsi.LANE_NUM				= LCM_TWO_LANE;
	//The following defined the fomat for data coming from LCD engine.
	params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
	params->dsi.data_format.trans_seq   = LCM_DSI_TRANS_SEQ_MSB_FIRST;
	params->dsi.data_format.padding     = LCM_DSI_PADDING_ON_LSB;
	params->dsi.data_format.format      = LCM_DSI_FORMAT_RGB888;
	params->dsi.LPX      = 13;

	// Highly depends on LCD driver capability.
	params->dsi.packet_size=256;

	// Video mode setting		
	params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;

	params->dsi.word_count=480*3;	
	params->dsi.vertical_sync_active=2;
	params->dsi.vertical_backporch=2; //0x07
	params->dsi.vertical_frontporch=2; //0x31
	params->dsi.vertical_active_line=800;

	params->dsi.line_byte=2180;		// 2256 = 752*3
	params->dsi.horizontal_sync_active_byte=26;
	params->dsi.horizontal_backporch_byte=206;
	params->dsi.horizontal_frontporch_byte=206;	
	params->dsi.rgb_byte=(480*3+6);	

	params->dsi.horizontal_sync_active_word_count=20;	
	params->dsi.horizontal_backporch_word_count=200;
	params->dsi.horizontal_frontporch_word_count=200;

	// Bit rate calculation
	params->dsi.pll_div1=0x1A;//38;		// fref=26MHz, fvco=fref*(div1+1)	(div1=0~63, fvco=500MHZ~1GHz)
	params->dsi.pll_div2=1;			// div2=0~15: fout=fvo/(2*div2)

}

static unsigned int lcm_compare_id(void)
{
	unsigned int id = 0, id2 = 0;
	unsigned char buffer[4];
	unsigned int array[16];

//	push_table(lcm_compare_id_setting, sizeof(lcm_compare_id_setting) / sizeof(struct LCM_setting_table), 1);
	
//Ivan
// 00 = MTK Config = Type 0 Instruction = Short Packet read/write
// 37 = DSI Data type = Set Maximum return packet size
// 0003 = data length = 3 bytes data to read
//Data = 00/11/22/33	
//	config_gpio();
/*
	SET_RESET_PIN(1);  //NOTE:should reset LCM firstly
	MDELAY(10);
	SET_RESET_PIN(0);
	MDELAY(10);
	SET_RESET_PIN(1);
	MDELAY(120);	

	array[0] = 0x00013700;// read id return two byte,version and id
	dsi_set_cmdq(array, 1, 1);
	MDELAY(10);	
	
	read_reg_v2(0xDB, buffer, 1);
	id = buffer[0]; //we only need ID
*/

	SET_RESET_PIN(1);  //NOTE:should reset LCM firstly
	MDELAY(10);
	SET_RESET_PIN(0);
	MDELAY(10);
	SET_RESET_PIN(1);
	MDELAY(120);	
	buffer[0] = 0;
	buffer[1] = 0;
	buffer[2] = 0;
	buffer[3] = 0;	
	
	array[0]=0x00063902;
	array[1]=0x52AA55F0;
	array[2]=0x00000108;
	dsi_set_cmdq(&array, 3, 1);
	MDELAY(10); 

	array[0] = 0x00023700;// read id return two byte,version and id
	dsi_set_cmdq(array, 1, 1);
	MDELAY(10); 
	
	read_reg_v2(0xC5, buffer, 2);
	id = buffer[0]; //we only need ID
	id2= buffer[1]; //we test buffer 1	

	LCM_DBG("ID1 = %x, ID2 = %x, ID3 = %x, ID4 = %x", buffer[0],buffer[1],buffer[2],buffer[3]);	
//	return (LCM_ID == id)?1:0;
	return 1;
}

static void lcm_init(void)
{
	LCM_DBG();
//	config_gpio();
//	lcm_compare_id();

	SET_RESET_PIN(1);
	MDELAY(1);
	SET_RESET_PIN(0);
	MDELAY(10);
	SET_RESET_PIN(1);
	MDELAY(150);

//	push_table(lcm_sw_reset_setting, sizeof(lcm_sw_reset_setting) / sizeof(struct LCM_setting_table), 1);

	push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);
	
	MDELAY(25);
}

static void lcm_suspend(void)
{
	LCM_DBG();
	push_table(lcm_deep_sleep_mode_in_setting, sizeof(lcm_deep_sleep_mode_in_setting) / sizeof(struct LCM_setting_table), 1);
}

static void lcm_resume(void)
{
	LCM_DBG();
	push_table(lcm_sleep_out_setting, sizeof(lcm_sleep_out_setting) / sizeof(struct LCM_setting_table), 1);
}

static void lcm_update(unsigned int x, unsigned int y,
                       unsigned int width, unsigned int height)
{
	unsigned int x0 = x;
	unsigned int y0 = y;
	unsigned int x1 = x0 + width - 1;
	unsigned int y1 = y0 + height - 1;

	unsigned char x0_MSB = ((x0>>8)&0xFF);
	unsigned char x0_LSB = (x0&0xFF);
	unsigned char x1_MSB = ((x1>>8)&0xFF);
	unsigned char x1_LSB = (x1&0xFF);
	unsigned char y0_MSB = ((y0>>8)&0xFF);
	unsigned char y0_LSB = (y0&0xFF);
	unsigned char y1_MSB = ((y1>>8)&0xFF);
	unsigned char y1_LSB = (y1&0xFF);

	unsigned int data_array[16];
//Ivan
// 02 = MTK Config = Type 2 Instruction
// 39 = DSI Data type = DCS Long write
// 0005 = data length = 5 bytes data follow
//Data = 00/11/22/33	
	
	data_array[0]= 0x00053902;		
	data_array[1]= (x1_MSB<<24)|(x0_LSB<<16)|(x0_MSB<<8)|0x2a;
	data_array[2]= (x1_LSB);
	data_array[3]= 0x00053902;
	data_array[4]= (y1_MSB<<24)|(y0_LSB<<16)|(y0_MSB<<8)|0x2b;
	data_array[5]= (y1_LSB);
	data_array[6]= 0x002c3909;

	dsi_set_cmdq(&data_array, 7, 0);
}

static void lcm_setbacklight(unsigned int level)
{
	LCM_DBG("level=%d", level);

	
	// Refresh value of backlight level.
	lcm_backlight_level_setting[0].para_list[0] = level;

	push_table(lcm_backlight_level_setting, sizeof(lcm_backlight_level_setting) / sizeof(struct LCM_setting_table), 1);
	
	light_lcd_on( level );
}

static void lcm_setpwm(unsigned int divider)
{
	LCM_DBG();
	// TBD
}

static unsigned int lcm_getpwm(unsigned int divider)
{
	// ref freq = 15MHz, B0h setting 0x80, so 80.6% * freq is pwm_clk;
	// pwm_clk / 255 / 2(lcm_setpwm() 6th params) = pwm_duration = 23706
	unsigned int pwm_clk = 23706 / (1<<divider);	
	LCM_DBG();
	return pwm_clk;
}

LCM_DRIVER nt35510_dsi_lcm_drv = 
{
    .name			= "nt35510_dsi",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params     = lcm_get_params,
	.init           = lcm_init,
	.suspend        = lcm_suspend,
	.resume         = lcm_resume,
	.update         = lcm_update,
	.set_backlight = lcm_setbacklight,
	.compare_id    = lcm_compare_id,	
//	.set_pwm = lcm_setpwm,
//	.get_pwm = lcm_getpwm,
};

