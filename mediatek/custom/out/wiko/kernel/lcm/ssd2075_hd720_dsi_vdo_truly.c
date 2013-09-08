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

#ifndef BUILD_LK
#include <linux/string.h>
#endif
#include "lcm_drv.h"

#ifdef BUILD_LK
	#include <platform/mt_gpio.h>
	#define LCM_PRINT printf
#elif defined(BUILD_UBOOT)
	#include <asm/arch/mt_gpio.h>
#else
	#include <linux/kernel.h>
	#include <mach/mt_gpio.h>
	#define LCM_PRINT printk
#endif
#define LCM_DBG
#if  defined(LCM_DBG)
#define LCM_DBG(fmt, arg...) \
	LCM_PRINT("[LCM-ssd0275-DSI] %s (line:%d) :" fmt "\r\n", __func__, __LINE__, ## arg)
#else
#define LCM_DBG(fmt, arg...) do {} while (0)
#endif
// ---------------------------------------------------------------------------
//  Local Constants
// ---------------------------------------------------------------------------

#define FRAME_WIDTH  (720)
#define FRAME_HEIGHT (1280)
#define ACTIVE_WIDTH  										(66.7)
#define ACTIVE_HEIGHT 										(120.4)
#define REGFLAG_DELAY             							0XFE
#define REGFLAG_END_OF_TABLE      							0xFA   // END OF REGISTERS MARKER

#define LCM_ID_SSD2075 (0x2075)

#ifndef TRUE
    #define TRUE 1
#endif

#ifndef FALSE
    #define FALSE 0
#endif

static unsigned int lcm_esd_test = FALSE;      ///only for ESD test

#define LCM_DEBUG
// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------

static LCM_UTIL_FUNCS lcm_util = {0};

#define SET_RESET_PIN(v)    (lcm_util.set_reset_pin((v)))

#define UDELAY(n) (lcm_util.udelay(n))
#define MDELAY(n) (lcm_util.mdelay(n))


// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------

#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)	        lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)		lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)										lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)					lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg(cmd)											lcm_util.dsi_dcs_read_lcm_reg(cmd)
#define read_reg_v2(cmd, buffer, buffer_size)   				lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)   


#define   LCM_DSI_CMD_MODE							0

struct LCM_setting_table {
    unsigned cmd;
    unsigned char count;
    unsigned char para_list[64];
};

static struct LCM_setting_table lcm_sleep_out_setting[] = {
 //    Sleep Out
	{0x11, 1, {0x00}},
	{REGFLAG_DELAY, 150, {}},

//  Display ON
	{0x29, 1, {0x00}},
	{REGFLAG_DELAY, 20, {}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
	
};

static struct LCM_setting_table lcm_initialization_setting[] = {
	{0xE1,	1,	{0xA3}},

	{0xB3,	1,	{0x00}},
	{0xB6,	4,	{0x16,0x0F,0x00,0x00}},
	{0xB8,	8,	{0x00,0x06,0x08,0x00,0x07,0x09,0x23,0x04}},
	{0xB9,	6,	{0x04,0x08,0x22,0xff,0xff,0x0f}},
	{0xBA,	8,	{0x0e,0x0e,0x10,0x10,0x0a,0x0a,0x0c,0x0c}},
	{0xBB,	8,	{0xa1,0xa1,0xa1,0xa1,0xa1,0xa1,0xa1,0xa1}},
	{0xBC,	8,	{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},
	{0xBD,	8,	{0x0f,0x0f,0x11,0x11,0x0b,0x0b,0x0d,0x0d}},
	{0xBE,	8,	{0xa1,0xa1,0xa1,0xa1,0xa1,0xa1,0xa1,0xa1}},
	{0xBF,	8,	{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},
	{0xB1,	3,	{0x16,0x1e,0x12}},

	{0xE0,	5,	{0x01,0x03,0x02,0x00,0x00}},

	{0xD0,	6,	{0x00,0x00,0x10,0x1E,0x22,0x2e}},
	{0xD1,	5,	{0x26,0x2b,0x23,0x1b,0x0a}},
	{0xD2,	6,	{0x00,0x00,0x10,0x1E,0x22,0x2e}},
	{0xD3,	5,	{0x26,0x2b,0x23,0x1b,0x0a}},
	{0xD4,	6,	{0x00,0x00,0x10,0x1E,0x22,0x2e}},
	{0xD5,	5,	{0x26,0x2b,0x23,0x1b,0x0a}},
	{0xD6,	6,	{0x00,0x00,0x10,0x1E,0x22,0x2e}},
	{0xD7,	5,	{0x26,0x2b,0x23,0x1b,0x0a}},
	{0xD8,	6,	{0x00,0x00,0x10,0x1E,0x22,0x2e}},
	{0xD9,	5,	{0x26,0x2b,0x23,0x1b,0x0a}},
	{0xDA,	6,	{0x00,0x00,0x10,0x1E,0x22,0x2e}},
	{0xDB,	5,	{0x26,0x2b,0x23,0x1b,0x0a}},

	{0x70,	4,	{0xd8,0x00,0xff,0x80}},
	{0xFF,	1,	{0x01}},

	//diff with truly{0xEC	1,	{0x12}},
	{0xC6,	2,	{0x99,0x33}},
	{0xDE,	2,	{0x9d,0x30}},
	{0x14,	1,	{0x00}},
	//diff with truly

	{0xE9,	1,	{0x07}},
	{0xED,	2,	{0x60,0x10}},
	{0xEC,	1,	{0x12}},

	{0xCD,	4,	{0x77,0x7B,0x34,0x08}},
	{0xC3,	7,	{0x03,0x05,0x34,0x05,0x01,0x44,0x54}},
	{0xC4,	5,	{0x02,0x03,0x58,0x58,0x5a}},
	{0xCB,	3,	{0xdf,0x80,0x00}},

	{0xEA,	2,	{0x15,0x28}},
	{0xF0,	4,	{0x38,0x00,0x00,0x00}},
	{0xC9,	3,	{0x60,0x00,0x82}},
	{0xB5,	8,	{0x00,0x05,0x05,0x1e,0x04,0x40,0x20,0xfc}},

	{0x36,	1,	{0x08}},

	{0x11,	0,	{0x00}},
	{0x29,	0,	{0x00}},
	{0xF0,	4,	{0x18,0xff,0xff,0x00}},
};



static void init_lcm_registers(void)
{
	unsigned int data_array[16];

    	data_array[0] = 0x00023902;                          
	data_array[1] = 0x0000A3E1;  //0x000093E1           //wangyanhui modify     
    dsi_set_cmdq(data_array, 2, 1);
	//MDELAY(1);
	
	data_array[0] = 0x00023902;                          
    data_array[1] = 0x000000B3;                 
    dsi_set_cmdq(data_array, 2, 1);
	//MDELAY(1);

	
	data_array[0] = 0x00053902;                          
    data_array[1] = 0x000F16B6; 
	data_array[2] = 0x00000000; 
    dsi_set_cmdq(data_array, 3, 1); 
    //MDELAY(1);
	 
	data_array[0] = 0x00093902;                          
    data_array[1] = 0x080600B8; 
	data_array[2] = 0x23090700; 
	data_array[3] = 0x00000004; 
    dsi_set_cmdq(data_array, 4, 1);
    //MDELAY(1);
	 
	data_array[0] = 0x00073902;                          
    data_array[1] = 0x220804B9; 
	data_array[2] = 0x000FFFFF;  //0x00FFFFFF
    dsi_set_cmdq(data_array, 3, 1); 
    //MDELAY(1);
	 
	data_array[0] = 0x00093902;                          
    data_array[1] = 0x100E0EBA; 
	data_array[2] = 0x0C0A0A10; 
	data_array[3] = 0x0000000C; 
    dsi_set_cmdq(data_array, 4, 1);
	//MDELAY(1);
	
	data_array[0] = 0x00093902;                          
    data_array[1] = 0xA1A1A1BB; 
	data_array[2] = 0xA1A1A1A1; 
	data_array[3] = 0x000000A1; 
    dsi_set_cmdq(data_array, 4, 1);
	//MDELAY(1);

	
	data_array[0] = 0x00093902;                          
    data_array[1] = 0x000000BC; 
	data_array[2] = 0x00000000; 
	data_array[3] = 0x00000000; 
    dsi_set_cmdq(data_array, 4, 1);
	//MDELAY(1);

	data_array[0] = 0x00093902;                          
    data_array[1] = 0x110F0FBD; 
	data_array[2] = 0x0D0B0B11; 
	data_array[3] = 0x0000000D; 
    dsi_set_cmdq(data_array, 4, 1);
	//MDELAY(1);

	 
	data_array[0] = 0x00093902;                          
    data_array[1] = 0xA1A1A1BE; 
	data_array[2] = 0xA1A1A1A1; 
	data_array[3] = 0x000000A1; 
    dsi_set_cmdq(data_array, 4, 1);
	//MDELAY(1);

	
	data_array[0] = 0x00093902;                          
    data_array[1] = 0x000000BF; 
	data_array[2] = 0x00000000; 
	data_array[3] = 0x00000000; 
    dsi_set_cmdq(data_array, 4, 1);
	//MDELAY(1);

	
	data_array[0] = 0x00043902;                          
    data_array[1] = 0x121E16B1;  //modified  14+3             
    dsi_set_cmdq(data_array, 2, 1); 
	//MDELAY(1);

	
	data_array[0] = 0x00063902;                          
    data_array[1] = 0x020301E0; //VSA
	data_array[2] = 0x00000100; 
    dsi_set_cmdq(data_array, 3, 1); 
	//MDELAY(1);

	
	data_array[0] = 0x00073902;                          
    data_array[1] = 0x100000D0; 
	data_array[2] = 0x002E221E; 
    dsi_set_cmdq(data_array, 3, 1); 
	//MDELAY(1);

	
	data_array[0] = 0x00063902;                          
    data_array[1] = 0x232B26D1; 
	data_array[2] = 0x00000A1B; 
    dsi_set_cmdq(data_array, 3, 1);
	//MDELAY(1);

	
	data_array[0] = 0x00073902;                          
    data_array[1] = 0x100000D2; 
	data_array[2] = 0x002E221E; 
    dsi_set_cmdq(data_array, 3, 1); 
	//MDELAY(1);

	
	data_array[0] = 0x00063902;                          
    data_array[1] = 0x232B26D3; 
	data_array[2] = 0x00000A1B; 
    dsi_set_cmdq(data_array, 3, 1);
	//MDELAY(1);

	
	data_array[0] = 0x00073902;                          
    data_array[1] = 0x100000D4; 
	data_array[2] = 0x002E221E; 
    dsi_set_cmdq(data_array, 3, 1); 
	//MDELAY(1);

	
	data_array[0] = 0x00063902;                          
    data_array[1] = 0x232B26D5; 
	data_array[2] = 0x00000A1B; 
    dsi_set_cmdq(data_array, 3, 1);  
	//MDELAY(1);

	data_array[0] = 0x00073902;                          
    data_array[1] = 0x100000D6; 
	data_array[2] = 0x002E221E; 
    dsi_set_cmdq(data_array, 3, 1);
	//MDELAY(1);

	data_array[0] = 0x00063902;                          
    data_array[1] = 0x232B26D7; 
	data_array[2] = 0x00000A1B; 
    dsi_set_cmdq(data_array, 3, 1); 
	//MDELAY(1);

	data_array[0] = 0x00073902;                          
    data_array[1] = 0x100000D8; 
	data_array[2] = 0x002E221E; 
    dsi_set_cmdq(data_array, 3, 1); 
	//MDELAY(1);

	data_array[0] = 0x00063902;                          
    data_array[1] = 0x232B26D9; 
	data_array[2] = 0x00000A1B; 
    dsi_set_cmdq(data_array, 3, 1); 
	//MDELAY(1);

	data_array[0] = 0x00073902;                          
    data_array[1] = 0x100000DA; 
	data_array[2] = 0x002E221E; 
    dsi_set_cmdq(data_array, 3, 1); 
	//MDELAY(1);
 
	data_array[0] = 0x00063902;                          
    data_array[1] = 0x232B26DB; 
	data_array[2] = 0x00000A1B; 
    dsi_set_cmdq(data_array, 3, 1); 
	//MDELAY(1);

	data_array[0] = 0x00053902;                          
    data_array[1] = 0xFF00D870; 
	data_array[2] = 0x00000080; 
    dsi_set_cmdq(data_array, 3, 1); 
	//MDELAY(1);

	data_array[0] = 0x00023902;                          
    data_array[1] = 0x000001FF;                 
    dsi_set_cmdq(data_array, 2, 1); 
	MDELAY(1);

#if 1											//diff with truly    lishengli   20121213  begin  
// add cmd-c6 
	data_array[0] = 0x00033902; 						 
	data_array[1] = 0x003399C6; 				
	dsi_set_cmdq(data_array, 2, 1); 
	MDELAY(1);
//add end

//legen modify 
	data_array[0] = 0x00033902;                          
    data_array[1] = 0x00309DDE; //00309DDE
	//data_array[2] = 0x00130D0C; 
    dsi_set_cmdq(data_array, 2, 1);


	data_array[0] = 0x00023902;                          
    data_array[1] = 0x00000014; 
	//data_array[2] = 0x00130D0C; 
    dsi_set_cmdq(data_array, 2, 1);
//legen modify end							//diff with truly    lishengli   20121213  end
#endif

	data_array[0] = 0x00023902;                          
    data_array[1] = 0x000007E9;                 
    dsi_set_cmdq(data_array, 2, 1); 
	MDELAY(1);

	data_array[0] = 0x00033902;                          
    data_array[1] = 0x001060ED;                 
    dsi_set_cmdq(data_array, 2, 1); 
	//MDELAY(1);

	data_array[0] = 0x00023902;                          
    data_array[1] = 0x000012EC;                 
    dsi_set_cmdq(data_array, 2, 1); 
	//MDELAY(1);
 
	data_array[0] = 0x00053902;                          
    data_array[1] = 0x347B77CD; 
	data_array[2] = 0x00000008; 
    dsi_set_cmdq(data_array, 3, 1);
	//MDELAY(1);
 
	data_array[0] = 0x00083902;                          
	data_array[1] = 0x340503C3;     //0x340503C3
	data_array[2] = 0x54440105; 
    dsi_set_cmdq(data_array, 3, 1);
	//MDELAY(1);

	data_array[0] = 0x00063902; 
	data_array[1] = 0x580302C4;  //0x701303c4 //0x700302C4
	data_array[2] = 0x00005A58;  //0x00005C70
    dsi_set_cmdq(data_array, 3, 1);
	//MDELAY(1);

    
	
	data_array[0] = 0x00043902;                          
	 
	data_array[1] = 0x0080DFCB;   //0x0080DACB     //line<20130109>wangyanhui           
    dsi_set_cmdq(data_array, 2, 1); 
	//MDELAY(1);

	data_array[0] = 0x00033902;                          
    data_array[1] = 0x002815EA;                 
    dsi_set_cmdq(data_array, 2, 1); 
	//MDELAY(1);
 
	data_array[0] = 0x00053902;                          
    data_array[1] = 0x000038F0; 
	data_array[2] = 0x00000000; 
    dsi_set_cmdq(data_array, 3, 1);
	//MDELAY(1);

	data_array[0] = 0x00043902;                          
    data_array[1] = 0x820060C9;                 
    dsi_set_cmdq(data_array, 2, 1);
	//MDELAY(1);

	data_array[0] = 0x00093902;                          
    data_array[1] = 0x050500B5;						//diff with truly    lishengli   20121213	
	data_array[2] = 0x2040041E;                          
    data_array[3] = 0x000000FC;                 
    dsi_set_cmdq(data_array, 4, 1);
	//MDELAY(1);

	data_array[0] = 0x00023902;                          
    data_array[1] = 0x00000836;                 
    dsi_set_cmdq(data_array, 2, 1);
    MDELAY(1);//wait for PLL to lock 						//diff with truly    lishengli   20121213

    //1 Do not delete 0x11, 0x29 here
	data_array[0] = 0x00110500; // Sleep Out
	dsi_set_cmdq(data_array, 1, 1);
	MDELAY(120);
	
	data_array[0] = 0x00290500; // Display On
	dsi_set_cmdq(data_array, 1, 1); 
	data_array[0] = 0x00053902;                          
	data_array[1] = 0xFFFF18F0;  
	data_array[2] = 0x00000000;  	
	dsi_set_cmdq(data_array, 3, 1);
	
}


//edit by Magnum 2012-12-18
static void dsi_send_cmdq_tinno(unsigned cmd, unsigned char count, unsigned 
char *para_list, unsigned char force_update)
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
			// an item make up of 4data. 
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
				dsi_send_cmdq_tinno(cmd, table[i].count, table[i].para_list, force_update);
		//	dsi_set_cmdq_V2(cmd, table[i].count, table[i].para_list, force_update);
       	}
    }
	
}


// ---------------------------------------------------------------------------
//  LCM Driver Implementations
// ---------------------------------------------------------------------------

static void lcm_set_util_funcs(const LCM_UTIL_FUNCS *util)
{
    memcpy(&lcm_util, util, sizeof(LCM_UTIL_FUNCS));
}


static void lcm_get_params(LCM_PARAMS *params)
{
		memset(params, 0, sizeof(LCM_PARAMS));
	
		params->type   = LCM_TYPE_DSI;

		params->width  = FRAME_WIDTH;
		params->height = FRAME_HEIGHT;
		params->active_width= ACTIVE_WIDTH;
		params->active_height= ACTIVE_HEIGHT;


       //1 SSD2075 has no TE Pin
		// enable tearing-free
		params->dbi.te_mode 				= LCM_DBI_TE_MODE_DISABLED;
		params->dbi.te_edge_polarity		= LCM_POLARITY_RISING;

        #if (LCM_DSI_CMD_MODE)
		params->dsi.mode   = CMD_MODE;
        #else
		//params->dsi.mode   = SYNC_PULSE_VDO_MODE;
		params->dsi.mode   = BURST_VDO_MODE;
		//params->dsi.mode   = SYNC_EVENT_VDO_MODE; 
		
        #endif
	
		// DSI
		/* Command mode setting */
		//1 Three lane or Four lane
		params->dsi.LANE_NUM				= LCM_FOUR_LANE;			//LCM_THREE_LANE;
		//The following defined the fomat for data coming from LCD engine.
		params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
		params->dsi.data_format.trans_seq   = LCM_DSI_TRANS_SEQ_MSB_FIRST;
		params->dsi.data_format.padding     = LCM_DSI_PADDING_ON_LSB;
		params->dsi.data_format.format      = LCM_DSI_FORMAT_RGB888;

		// Highly depends on LCD driver capability.
		// Not support in MT6573
		params->dsi.packet_size=256;

		// Video mode setting		
		params->dsi.intermediat_buffer_num = 0;//because DSI/DPI HW design change, this parameters should be 0 when video mode in MT658X; or memory leakage

		params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;
		params->dsi.word_count=720*3;	

		
		params->dsi.vertical_sync_active				= 3;  //---3
		params->dsi.vertical_backporch					= 12 ; //---14   12	zhuxiankun 12
		params->dsi.vertical_frontporch					= 8;  //----8
		params->dsi.vertical_active_line				= FRAME_HEIGHT; 

		params->dsi.horizontal_sync_active				= 2;  //----2
		params->dsi.horizontal_backporch				= 28; //----28
		params->dsi.horizontal_frontporch				= 50; //----50
		params->dsi.horizontal_active_pixel				= FRAME_WIDTH;


		params->dsi.HS_PRPR=3;
		params->dsi.CLK_HS_POST = 22;
		params->dsi.DA_HS_EXIT =35;
	    	//params->dsi.LPX=8; 
		//params->dsi.HS_PRPR=5;
		//params->dsi.HS_TRAIL=13;

		// Bit rate calculation
		//1 Every lane speed
//		params->dsi.pll_div1=0;		// div1=0,1,2,3;div1_real=1,2,4,4 ----0: 546Mbps  1:273Mbps
//		params->dsi.pll_div2=1;		// div2=0,1,2,3;div1_real=1,2,4,4	
//		params->dsi.fbk_div =19;    // fref=26MHz, fvco=fref*(fbk_div+1)*2/(div1_real*div2_real)	
		params->dsi.PLL_CLOCK = LCM_DSI_6589_PLL_CLOCK_214_5;

		params->dsi.noncont_clock = TRUE;
		params->dsi.noncont_clock_period = 2;	// Unit : frames
}

static void lcm_init(void)
{

	SET_RESET_PIN(1);
	MDELAY(5);    //>1ms
	SET_RESET_PIN(0);
	MDELAY(40);  // >30ms
	
	SET_RESET_PIN(1);
	MDELAY(80);    // >60ms      

	//init_lcm_registers();
	push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);

}



static void lcm_suspend(void)
{
	unsigned int data_array[16];

	SET_RESET_PIN(1);
	MDELAY(5);
	SET_RESET_PIN(0);
	MDELAY(10);
	SET_RESET_PIN(1);
	MDELAY(120);
	
	data_array[0] = 0x00023902;                          
	data_array[1] = 0x000001FF;                 
	dsi_set_cmdq(data_array, 2, 1);
	
	data_array[0] = 0x00073902;                          
	data_array[1] = 0x111815DE;
	data_array[2] = 0x00180F10;   
	dsi_set_cmdq(data_array, 3, 1);

	data_array[0] = 0x00083902;                          
	data_array[1] = 0x340600C3;
	data_array[2] = 0x54440105;   
	dsi_set_cmdq(data_array, 3, 1); 

	data_array[0] = 0x00033902;                          
	data_array[1] = 0x00100CCE; 
	dsi_set_cmdq(data_array, 2, 1);
}


static void lcm_resume(void)
{
   //1 do lcm init again to solve some display issue
	SET_RESET_PIN(1);
   	MDELAY(2);   // > 1ms
	SET_RESET_PIN(0);
	MDELAY(40); // >30ms
	
	SET_RESET_PIN(1);
	MDELAY(80);  // >60ma      

	init_lcm_registers();

}
         
#if (LCM_DSI_CMD_MODE)
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

	data_array[0]= 0x00053902;
	data_array[1]= (x1_MSB<<24)|(x0_LSB<<16)|(x0_MSB<<8)|0x2a;
	data_array[2]= (x1_LSB);
	dsi_set_cmdq(&data_array, 3, 1);
	
	data_array[0]= 0x00053902;
	data_array[1]= (y1_MSB<<24)|(y0_LSB<<16)|(y0_MSB<<8)|0x2b;
	data_array[2]= (y1_LSB);
	dsi_set_cmdq(&data_array, 3, 1);

	data_array[0]= 0x00290508; //HW bug, so need send one HS packet
	dsi_set_cmdq(&data_array, 1, 1);
	
	data_array[0]= 0x002c3909;
	dsi_set_cmdq(&data_array, 1, 0);

}
#endif

static unsigned int lcm_compare_id(void)
{


    unsigned int id0,id1,id=0;
	unsigned char buffer[2];
	unsigned int array[16];  

    SET_RESET_PIN(1);
    SET_RESET_PIN(0);
    MDELAY(1);
    SET_RESET_PIN(1);
    MDELAY(10);

	array[0] = 0x00023700;// return byte number
	dsi_set_cmdq(&array, 1, 1);
	MDELAY(10);

	read_reg_v2(0xA1, buffer, 2);
	id0 = buffer[0]; 
	id1 = buffer[1];
	id=(id0<<8)|id1;
	
    #ifdef BUILD_LK
	printf("%s, LK ssd2075 id0 = 0x%08x\n", __func__, id0);
	printf("%s, LK ssd2075 id1 = 0x%08x\n", __func__, id1);
	printf("%s, LK ssd2075 id = 0x%08x\n", __func__, id);
   #else
	printk("%s, Kernel ssd2075 id0 = 0x%08x\n", __func__, id0);
	printk("%s, Kernel ssd2075 id1 = 0x%08x\n", __func__, id1);
	printk("%s, Kernel ssd2075 id = 0x%08x\n", __func__, id);
   #endif

  return (LCM_ID_SSD2075 == id)?1:1;


}

#if 1
static unsigned int lcm_esd_check(void)
{

    #ifndef BUILD_LK
      char  buffer[4] = {0xff, 0xff, 0xff , 0xff};
      int   array[4];

      printk("[DISP] lcm_esd_check  ");

      if(lcm_esd_test)
       {
           lcm_esd_test = FALSE;
           return TRUE;
        }

        array[0] = 0x00083700;// read id return two byte,version and id
        dsi_set_cmdq(array, 1, 1);

        read_reg_v2(0xF5, buffer, 3);
        printk(" [DISP] lcm_esd_check   buffer[0] = 0x%x  ,buffer[1] = 0x%x,buffer[2] = 0x%x \n",buffer[0],buffer[1],buffer[2]);

        
        if(((buffer[0]&0xff)==0) && ((buffer[1]&0xf0) ==0)&&((buffer[2]&0x0f)== 0))
        {
                return FALSE;
        }
        else
        {
                return TRUE;
        }
     #endif
	 
}


static unsigned int lcm_esd_recover(void)
{
	//lcm_init();
	lcm_resume();

	return TRUE;
}
#endif

#ifdef LCM_DEBUG
char * magnum_strsep(char **s, const char *ct)
{
	char *sbegin = *s, *end;

	if (sbegin == NULL)
		return NULL;

	end = strpbrk(sbegin, ct);
	if (end)
		*end++ = '\0';
	*s = end;

	return sbegin;
}
static struct LCM_setting_table  lcm_debug_params[100];
static int register_count = 0;
static char  tempbuf[100][500];

//just for register and, register length is 2
static int ver_2_num(char* ver)
{
	LCM_DBG("ver ====== %s",ver);
    unsigned long var=0;
    unsigned long t;
    int len = strlen(ver);
//	LCM_DBG("data length %d",len);
    if (var > 8) //\u93c8\u20ac\u95c0?\u6d63?
 	 return -1;
	 int i = 0;
	//for (i; i!=2; i++)
	for (i; ver[i] !='\0'; i++)
	 {
	     if(i==2)
		 	break;
	 	 if (ver[i]>='a' && ver[i] <='f')
		   t = ver[i]-87;
	 	 else if (ver[i]>='A' && ver[i] <='F')
		   t = ver[i]-55;
		 else if(ver[i] < 47)
		  	 continue;
	 	 else
	 	 t = ver[i]-48;
//		 LCM_DBG("t ====== %x",t);
	 	 var<<=4;
	  	 var|=t;
     }
	return var; 
}

static void GetDebugInfo(char * cmd, char * data,int reg_count)
{
	unsigned tempcmd = ver_2_num(cmd);
	int data_count = 0;
	lcm_debug_params[reg_count].cmd = tempcmd;
	char *p = NULL;
	char delim2[] = ",";
	//p = magnum_strsep(&data,delim2);   // split to  para_list
	LCM_DBG("data  = %s",data);
	 unsigned int datatemp;
	 while((p=magnum_strsep(&data,delim2))){
	 	     datatemp = ver_2_num(p);
			 LCM_DBG("data temp = %x",datatemp);
	        lcm_debug_params[reg_count].para_list[data_count] = datatemp;
        	data_count++;	
        }
      
    lcm_debug_params[reg_count].count = data_count;
   	LCM_DBG("[Magnum] GetDebugInfo...cmd=%x, count = %d \n",lcm_debug_params[reg_count].cmd,lcm_debug_params[reg_count].count);
	int j = 0;
	while(j!= data_count){
		LCM_DBG("para_list=%x\n",lcm_debug_params[reg_count].para_list[j]);
		j++;
	}
	LCM_DBG("\n");
	
}
static void handle_lcm_debug_buff(char * buf)
{

	LCM_DBG("[MAGNUM]get lcm debug buffer == %s \n",buf);
	int length = strlen(buf);
	if(length <= 0){
		LCM_DBG("RETURN .............. \n");
		return;
		}
	char delim[]="+";
    char *p=NULL;
    char *p1=NULL;
    char * pCmd = NULL;
	char * pData = NULL;
	char delim1[]=";";
	
	//p = magnum_strsep(&buf,delim);   // split to  LCM_setting_table
//	 while(NULL != (p=magnum_strsep(&buf,delim))){
	while(p=magnum_strsep(&buf,delim)){
        	strcpy(tempbuf[register_count],p);
        //	tempbuf[register_count] = p;
        	LCM_DBG("tempbuf == %s\n",tempbuf[register_count]);
        	register_count++;	
        }	
	LCM_DBG("register_count ==[%d]\n",register_count);
    int i = 0;
    for( i;i<register_count;i++)
    {
		p1 = tempbuf[i];
		pCmd = magnum_strsep(&p1,delim1); // split to cmd & data;
		LCM_DBG("lcm_debug_params[%d]: ",i);
		LCM_DBG("cmd == %s  && ",pCmd);
		pData=magnum_strsep(&p1,delim1);
		LCM_DBG("data == %s && length ==%d\n",pData,strlen(pData));
		GetDebugInfo(pCmd,pData,i);
    }
//	lcm_debug_params[register_count].cmd = 255;
 //   lcm_debug_params[register_count].count = 0;
   // lcm_debug_params[register_count].count
   i = 0;
 //  while(++i < (register_count +1))
   while(++i < (register_count ))
   {
   		LCM_DBG("lcm_debug_params[%d]: ",i);
		LCM_DBG("cmd == %x  && ",lcm_debug_params[i].cmd);
		LCM_DBG("count == %d  && ",lcm_debug_params[i].count);
		char * para = lcm_debug_params[i].para_list;
		LCM_DBG("data == ");
		int j = 0;
		while(j!= lcm_debug_params[i].count){
			LCM_DBG(" %x ,",lcm_debug_params[i].para_list[j]);
			j++;
		}
		LCM_DBG("\n");
   }
	
}

static void lcm_debug(char * buf)
{
	//edit by Magnum 2012-11-1
	//static struct LCM_setting_table  lcm_debug_params[];
	     //  lcm_debug[0] = {cmd ,count,};
	//     lcm_debug_params[0].cmd = cmd;
	//	 lcm_debug_params[0].count = count;
	//	 strcpy(lcm_debug_params[0].para_list,para_list);
	//char * p = strdup(buf);
	LCM_DBG("[Magnum] lcm_debug %s\n",buf);
//	if(enableCE(buf))
	//	return;
	handle_lcm_debug_buff(buf);
	
	push_table(lcm_debug_params, register_count, 1);
	register_count = 0;
	memset(lcm_debug_params,0,sizeof(lcm_debug_params) / sizeof(struct LCM_setting_table));
	LCM_DBG("[Magnum] lcm_debug %x\n",lcm_debug_params[0].cmd);
}
#endif


LCM_DRIVER ssd2075_hd720_dsi_vdo_truly_lcm_drv = 
{
    .name			= "ssd2075_hd720_dsi_vdo_truly",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params     = lcm_get_params,
	.init           = lcm_init,
	.suspend        = lcm_suspend,
	.resume         = lcm_resume,
	.compare_id    = lcm_compare_id,
//	.esd_check = lcm_esd_check,
//	.esd_recover = lcm_esd_recover,
    #if (LCM_DSI_CMD_MODE)
    .update         = lcm_update,
    #endif
    #ifdef LCM_DEBUG
	.m_debug	        = lcm_debug,
    #endif
    };
