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
	LCM_PRINT("[LCM-HX8394A-DSI] %s (line:%d) :" fmt "\r\n", __func__, __LINE__, ## arg)
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
#define REGFLAG_DELAY             								0XFE
#define REGFLAG_END_OF_TABLE      							0xFA   // END OF REGISTERS MARKER
#define LCM_ID_HX8394 (0x94)
//#define LCM_DEBUG			

#ifndef TRUE
    #define TRUE 1
#endif

#ifndef FALSE
    #define FALSE 0
#endif

static unsigned int lcm_esd_test = FALSE;      ///only for ESD test

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
static struct LCM_setting_table lcm_initialization_setting[] = {
	{0x11,	1,     {0x00}}, 
	{REGFLAG_DELAY, 50, {}},
	{0xB9,	3,	{0xFF, 0x83, 0x94}},
	{0xBA,	17,	{0x13,0x83,0x00,0x16,0xA6,0x11,0x08,0xFF,
				  0x0F,0x24,0x03,0x21,0x24,0x25,0x20,0x02,	
				  0x10}},  
	{0xB1,	15,	{0x7C,0x00,0x24,0x89,0x01,0x11,0x11,0x36,
			  	  0x3E,0x26,0x26,0x57,0x1A,0x0E,0xE6}},
	{0xB2,	6,	{0x0F,0xC8,0x04,0x04,0x00,0x81}},
	{0xBF,	2,     {0x06,0x10}}, 
	{0xB4,	18,	{0x00,0x00,0x00,0x05,0x06,0x41,0x42,0x02,
				  0x41,0x42,0x43,0x47,0x19,0x58,0x58,0x08,
				  0x85,0x10}},  
	{0xD5,	24,	{0x4C,0x01,0x00,0x01,0xCD,0x23,0xEF,0x45,
				  0x67,0x89,0xAB,0x11,0x00,0xDC,0x10,0xFE,
				  0x32,0xBA,0x98,0x76,0x54,0x00,0x11,0x40}},
	{0xE0,	34,	{0x24,0x33,0x36,0x3F,0x3F,0x3F,0x3C,0x56,
				  0x05,0x0C,0x0E,0x11,0x13,0x12,0x14,0x12,
				  0x1E,0x24,0x33,0x36,0x3F,0x3F,0x3F,0x3C,
				  0x56,0x05,0x0C,0x0E,0x11,0x13,0x12,0x14,
				  0x12,0x1E}},//GAMMA
	{REGFLAG_DELAY, 5, {}},
	{0xE3,	1,	{0x01}}, //CE ENABLE
	{0xE0,	10,	{0x00,0x00,0x04,0x04,0x02,0x00,0x80,0x20,
				  0x00,0x20,0x00,0x00,0x08,0x06,0x04,0x00,
				  0x80,0x0E}},  //CE PARAMS
	{REGFLAG_DELAY, 1, {}},
	{0xC7,	2,	{0x00,0X30}}, //SET TCON_OPT
	{0xB6,	1,	{0x2A}},//SET VCOM
	{0xCC,	1,	{0x09}},//SET PANEL
	
	{0x29,	1,     {0x00}}, 
	{REGFLAG_DELAY, 120, {}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

static void init_lcm_registers(void)
{
	unsigned int data_array[16];

#if 1
	data_array[0] = 0x00110500; // Sleep Out
	dsi_set_cmdq(data_array, 1, 1);
	MDELAY(200);

	data_array[0] = 0x00043902;                          
      data_array[1] = 0x9483ffb9;                 
      dsi_set_cmdq(data_array, 2, 1);
	
	//{0xB9,	3,	{0xFF,0x83,0x94}},			//set password

	data_array[0] = 0x00123902;                          
      data_array[1] = 0x008313ba;      
      data_array[2] = 0x0811a616;   
      data_array[3] = 0x03240fff;   
      data_array[4] = 0x20252421;   
      data_array[5] = 0x00001002;   	  
      dsi_set_cmdq(data_array, 6, 1);
	  
  	//{0xBA,	17,	{0x13, 0x83, 0x00, 0x16,		// set mipi control
  				//  0xA6, 0x11, 0x08, 0xFF,
  				 // 0x0F, 0x24, 0x03, 0x21,
  				 // 0x24, 0x25, 0x20, 0x02,
  				 // 0x10}},    

     data_array[0] = 0x00103902;                          
      data_array[1] = 0x24007cb1;      
      data_array[2] = 0x11110109;   
      data_array[3] = 0x26263e36;   
      data_array[4] = 0xe6011a57;   //0a   	  
      dsi_set_cmdq(data_array, 5, 1);
	  
  	//{0xB1,	15,	{0x7C, 0x00, 0x24, 0x09,		// set power
  				  //0x01, 0x11, 0x11, 0x36,
  				  //0x3E, 0x26, 0x26, 0x57,
  				  //0x0A, 0x01, 0xE6}},    

      data_array[0]=0x00200500; 					//line <> <20121215> lishengli  
	dsi_set_cmdq(&data_array, 1, 1);


      //data_array[0] = 0x00023902;                     	//line <> <20121215> lishengli     
      //data_array[1] = 0x00000a36;          	  
      //dsi_set_cmdq(data_array, 2, 1);    //wangyanhui delete

      data_array[0] = 0x00023902;                     	//line <> <20121215> lishengli     
      data_array[1] = 0x000009cc;          	  
      dsi_set_cmdq(data_array, 2, 1);    //wangyanhui delete

      //data_array[0] = 0x00023902;		   //wangyanhui add 
      //data_array[1] = 0x000009cc;          	  
      //dsi_set_cmdq(data_array, 2, 1);  
      
      data_array[0] = 0x00073902;                          
      data_array[1] = 0x04c80fb2;      
      data_array[2] = 0x00810004;     	  
      dsi_set_cmdq(data_array, 3, 1);
	  
  	//{0xB2,	6,	{0x0F, 0xC8, 0x04, 0x04,
  				 // 0x00, 0x81}},    

      data_array[0] = 0x00033902;                          
      data_array[1] = 0x001006bf;      	  
      dsi_set_cmdq(data_array, 2, 1);  
  	//{0xBF,	2,	{0x06, 0x10}},    				// increase driving abilty


     data_array[0] = 0x00133902;                          
      data_array[1] = 0x000000b4;      
      data_array[2] = 0x42410605;   
      data_array[3] = 0x43424102;   
      data_array[4] = 0x60581947;   
      data_array[5] = 0x00108508;   	  
      dsi_set_cmdq(data_array, 6, 1);
	  
  	//{0xB4,	18,	{0x00, 0x00, 0x00, 0x05,		// SET CYC 
  				 // 0x06, 0x41, 0x42, 0x02,
  				  //0x41, 0x42, 0x43, 0x47,
  				  //0x19, 0x58, 0x60, 0x08,
  				  //0x85, 0x10}},    
    
     data_array[0] = 0x00193902;                          
      data_array[1] = 0x07014cd5;      
      data_array[2] = 0xef23cd01;   
      data_array[3] = 0xab896745;   
      data_array[4] = 0x10dc0011;   
      data_array[5] = 0x98ba32fe;   
      data_array[6] = 0x11005476;   	
      data_array[7] = 0x00000040;   		  
      dsi_set_cmdq(data_array, 8, 1);
	  
	//{0xD5,	24,	{0x4C, 0x01, 0x07, 0x01,		// SET GIP
				 //0xCD, 0x23, 0xEF, 0x45,
				 //0x67, 0x89, 0xAB, 0x11,
				 //0x00, 0xDC, 0x10, 0xFE,
				 //0x32, 0xBA, 0x98, 0x76,
				 //0x54, 0x00, 0x11, 0x40}},
     data_array[0] = 0x00233902;                          
      data_array[1] = 0x363324e0;      
      data_array[2] = 0x3c3f3f3f;   
      data_array[3] = 0x0e0c0556;   
      data_array[4] = 0x14121311;   
      data_array[5] = 0x33241e12;   
      data_array[6] = 0x3f3f3f36;   	
      data_array[7] = 0x0c05563c;   	
      data_array[8] = 0x1213110e; 	
      data_array[9] = 0x001e1214; 	  
      dsi_set_cmdq(data_array, 10, 1);
	  
	//{0xE0, 	34,	{0x24, 0x33, 0x36, 0x3F,		// R Gamma
				  //0x3f, 0x3f, 0x3c, 0x56,
				  //0x05, 0x0c, 0x0e, 0x11,
				  //0x13, 0x12, 0x14, 0x12,
				  //0x1e, 0x24, 0x33, 0x36,
				 // 0x3f, 0x3f, 0x3f, 0x3c,
				  //0x56, 0x05, 0x0c, 0x0e,
				  //0x11, 0x13, 0x12, 0x14, 
				  //0x12, 0x1e}},
	
      data_array[0] = 0x00023902;                          
      data_array[1] = 0x000001e3;         	  
      dsi_set_cmdq(data_array, 2, 1);  
	//{0xE3,	1, {0x01}},						// ENABLE CE
	//{REGFLAG_DELAY, 5, {}},
	MDELAY(5);
	data_array[0] = 0x00133902;                          
      data_array[1] = 0x040000e5;      
      data_array[2] = 0x80000204;   
      data_array[3] = 0x00200020;   
      data_array[4] = 0x04060800;   
      data_array[5] = 0x000e8000;   	  
      dsi_set_cmdq(data_array, 6, 1);
	  
	//{0xE5,	18,	{0x00, 0x00, 0x04, 0x04, 		// CE PARAMETER	
				 // 0x02, 0x00, 0x80, 0x20, 
				  //0x00, 0x20, 0x00, 0x00, 
				  //0x08, 0x06, 0x04, 0x00, 
				  //0x80, 0x0E}}, 
	//{REGFLAG_DELAY, 5, {}},
	

      data_array[0] = 0x00033902;                          
      data_array[1] = 0x003000c7;          	  
      dsi_set_cmdq(data_array, 2, 1);  
	//{0xC7, 	2,	{0x00, 0x30}},				// Set TCON_OPT 


// add cmd-c6 
	data_array[0] = 0x00043902; 						 
	data_array[1] = 0x800004C6; 				
	dsi_set_cmdq(data_array, 2, 1); 
	MDELAY(1);
//add end //wangyanhui add 

      data_array[0] = 0x00023902;                          
      data_array[1] = 0x00002ab6;       
      dsi_set_cmdq(data_array, 2, 1);  
	  
	//{0xB6, 	1,	{0x2A}},						//Set VCOM
	data_array[0] = 0x00110500; // Sleep Out
	dsi_set_cmdq(data_array, 1, 1);
	MDELAY(120);
	
	data_array[0] = 0x00290500; // Display On
	dsi_set_cmdq(data_array, 1, 1);
	MDELAY(120);
	//{0x29,	0,	{}},
	//{REGFLAG_DELAY, 120, {}},
#endif	

}


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
				dsi_send_cmdq_tinno(cmd, table[i].count, table[i].para_list, force_update);
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
		params->dsi.LANE_NUM				= LCM_FOUR_LANE;
		
		//The following defined the fomat for data coming from LCD engine.
		params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
		params->dsi.data_format.trans_seq   = LCM_DSI_TRANS_SEQ_MSB_FIRST;
		params->dsi.data_format.padding     = LCM_DSI_PADDING_ON_LSB;
		params->dsi.data_format.format      = LCM_DSI_FORMAT_RGB888;
		
		// Video mode setting		
		
		// Highly depends on LCD driver capability.
		// Not support in MT6573
		params->dsi.packet_size=256;

		// Video mode setting		
		params->dsi.intermediat_buffer_num = 0;//because DSI/DPI HW design change, this parameters should be 0 when video mode in MT658X; or memory leakage

		params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;
		params->dsi.word_count=720*3;	

	#if 0   //from hx8369
		params->dsi.vertical_sync_active				= 3;
		params->dsi.vertical_backporch					= 12;
		params->dsi.vertical_frontporch					= 2;
		params->dsi.vertical_active_line				= FRAME_HEIGHT; 

		params->dsi.horizontal_sync_active				= 10;
		params->dsi.horizontal_backporch				= 50;
		params->dsi.horizontal_frontporch				= 50;
		params->dsi.horizontal_active_pixel				= FRAME_WIDTH;
	#endif
	
		params->dsi.vertical_sync_active				= 3;  //---3
		params->dsi.vertical_backporch					= 12; //---14
		params->dsi.vertical_frontporch					= 9;  //----8
		params->dsi.vertical_active_line				= FRAME_HEIGHT; 

		params->dsi.horizontal_sync_active				= 11;//26;		// 2;  //----2
		params->dsi.horizontal_backporch				= 90;//146;			// 28; //----28
		params->dsi.horizontal_frontporch				= 90;//146;			// 50; //----50
		params->dsi.horizontal_active_pixel				= FRAME_WIDTH;


        //	params->dsi.HS_PRPR=6;
	//    params->dsi.LPX=8; 
		//params->dsi.HS_PRPR=5;
		//params->dsi.HS_TRAIL=13;

		// Bit rate calculation
		//1 Every lane speed
	//	params->dsi.pll_div1=	0;		// div1=0,1,2,3;div1_real=1,2,4,4 ----0: 546Mbps  1:273Mbps
	//	params->dsi.pll_div2=	1;		// div2=0,1,2,3;div1_real=1,2,4,4	
	//	params->dsi.fbk_div =	19;    // fref=26MHz, fvco=fref*(fbk_div+1)*2/(div1_real*div2_real)	
		params->dsi.PLL_CLOCK = LCM_DSI_6589_PLL_CLOCK_227_5;//LCM_DSI_6589_PLL_CLOCK_234;
    #ifdef BUILD_LK
	  printf("[LK]---hx8394a----%s------\n",__func__);
    #else
	  printk("[KERNEL]---hx8394a----%s------\n",__func__);
    #endif	
}

static void lcm_init(void)
{

	SET_RESET_PIN(1);
	SET_RESET_PIN(0);
	MDELAY(1);
	
	SET_RESET_PIN(1);
	MDELAY(20);      

//	init_lcm_registers();
	push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);
    #ifdef BUILD_LK
	  printf("[LK]---hx8394a----%s------\n",__func__);
    #else
	  printk("[KERNEL]---hx8394a----%s------\n",__func__);
    #endif	
}



static void lcm_suspend(void)
{
	unsigned int data_array[16];

	data_array[0] = 0x00100500; // Sleep In
	dsi_set_cmdq(&data_array, 1, 1);
	MDELAY(200);
    #ifdef BUILD_LK
	  printf("[LK]---hx8394a----%s------\n",__func__);
    #else
	  printk("[KERNEL]---hx8394a----%s------\n",__func__);
    #endif	
}


static void lcm_resume(void)
{
   //1 do lcm init again to solve some display issue

	SET_RESET_PIN(1);
	SET_RESET_PIN(0);
	MDELAY(1);
	
	SET_RESET_PIN(1);
	MDELAY(20);      

	init_lcm_registers();
    #ifdef BUILD_LK
	  printf("[LK]---hx8394a----%s------\n",__func__);
    #else
	  printk("[KERNEL]---hx8394a----%s------\n",__func__);
    #endif	
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

      array[0] = 0x00043902;                          
      array[1] = 0x9483ffb9;                 
      dsi_set_cmdq(array, 2, 1);
	  
	MDELAY(10);

    memset(buffer, 0, sizeof(buffer));
	read_reg_v2(0xf4, buffer, 1);
	id=buffer[0];
	
    #ifdef BUILD_LK
	printf("%s, LK hx8394a id0 = 0x%08x\n", __func__, id0);
	printf("%s, LK hx8394a id1 = 0x%08x\n", __func__, id1);
	printf("%s, LK hx8394a id = 0x%08x\n", __func__, id);
   #else
	printk("%s, Kernel hx8394a id0 = 0x%08x\n", __func__, id0);
	printk("%s, Kernel hx8394a id1 = 0x%08x\n", __func__, id1);
	printk("%s, Kernel hx8394a id = 0x%08x\n", __func__, id);
   #endif

  return (LCM_ID_HX8394 == id)?1:0;


}

#if 1
static unsigned int lcm_esd_check(void)
{
#ifndef BUILD_LK
	char  buffer[3];
	//int   array[4];

	if(lcm_esd_test)
	{
		lcm_esd_test = FALSE;
		return TRUE;
	}

	/// please notice: the max return packet size is 1
	/// if you want to change it, you can refer to the following marked code
	/// but read_reg currently only support read no more than 4 bytes....
	/// if you need to read more, please let BinHan knows.
	/*
			unsigned int data_array[16];
			unsigned int max_return_size = 1;
			
			data_array[0]= 0x00003700 | (max_return_size << 16);	
			
			dsi_set_cmdq(&data_array, 1, 1);
	*/
	/*
		test ic power state 0Ah == 0xa0 
		test ic Read Display Status 09h== 0xa0 , 0xbb , 0xbb , 0xc0

		test ic power state 0Ah == 0x1c 
		test ic Read Display Status 09h== 0x80, 0x73 , 0x0 , 0x0

	*/
	 unsigned int normal=0;
        unsigned char buffer1[1];
	unsigned char buffer2[4];
	unsigned int array[16]; 
	array[0] = 0x00013700;// set return byte number
	dsi_set_cmdq(array, 1, 1);
	read_reg_v2(0x0A, &buffer1, 1);  //hx8394a power state
	LCM_DBG("test ic power state 0Ah == 0x%x \n",buffer1[0]);
	if(buffer1[0] == 0x1c)
	{
   		LCM_DBG("esd check OK..\n");
	 	return 0;
	}else{
		LCM_DBG("esd check error..\n");
		return 1;
	}

#if 0	     //  read  display status will return bad value some times , advice from Himax that delete these code
	array[0] = 0x00043700;// set return byte number
	dsi_set_cmdq(array, 1, 1);
	read_reg_v2(0x09, &buffer2, 4);//hx8394a display state
	LCM_DBG("test ic Read Display Status 09h== 0x%x , 0x%x , 0x%x , 0x%x\n",buffer2[0],buffer2[1],buffer2[2],buffer2[3]);
    if(buffer2[0] == 0x80 && buffer2[1] == 0x73 && buffer2[2] == 0x00 && buffer2[3] == 0x00)
    	{
   		LCM_DBG("esd check OK..\n");
	 	return 0;
	}else{
		LCM_DBG("esd check error..\n");
		return 1;
	}
#endif
#endif

}

static unsigned int lcm_esd_recover(void)
{
	lcm_init();
	lcm_resume();

	return TRUE;
}
#endif

#ifdef LCM_DEBUG
static char * magnum_strsep(char **s, const char *ct)
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
static int ver_2_num(char* ver)
{
	LCM_DBG("ver ====== %s",ver);
    unsigned long var=0;
    unsigned long t;
    int len = strlen(ver);
    if (var > 8) //\u93c8\u20ac\u95c0?\u6d63?
 	 return -1;
	 int i = 0;
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
	while(p=magnum_strsep(&buf,delim)){
        	strcpy(tempbuf[register_count],p);
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
   i = 0;
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
	LCM_DBG("[Magnum] lcm_debug %s\n",buf);
	handle_lcm_debug_buff(buf);
	push_table(lcm_debug_params, register_count, 1);
	register_count = 0;
	memset(lcm_debug_params,0,sizeof(lcm_debug_params) / sizeof(struct LCM_setting_table));
	LCM_DBG("[Magnum] lcm_debug %x\n",lcm_debug_params[0].cmd);
}
#endif
LCM_DRIVER hx8394a_dsi_vdo_lcm_drv = 
{
    .name			= "hx8394a_hd720_dsi_vdo_boe",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params     = lcm_get_params,
	.init           = lcm_init,
	.suspend        = lcm_suspend,
	.resume         = lcm_resume,
	.compare_id    = lcm_compare_id,
	//.esd_check = lcm_esd_check,
	//.esd_recover = lcm_esd_recover,
    #if (LCM_DSI_CMD_MODE)
    .update         = lcm_update,
    #endif
    #ifdef LCM_DEBUG
	.m_debug	        =  lcm_debug,
    #endif
    };
