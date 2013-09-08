/*****************************************************************************
*  Copyright Statement:
*  --------------------
*  This software is protected by Copyright and the information contained
*  herein is confidential. The software may not be copied and the information
*  contained herein may not be used or disclosed except with the written
*  permission of MediaTek Inc. (C) 2005
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

/*****************************************************************************
 *
 * Filename:
 * ---------
 *   sensor.h
 *
 * Project:
 * --------
 *   DUMA
 *
 * Description:
 * ------------
 *   CMOS sensor header file
 *
 ****************************************************************************/
#ifndef _OV8831_SENSOR_H
#define _OV8831_SENSOR_H

#define OV8831_DEBUG
#define OV8831_DRIVER_TRACE
//#define OV8831_TEST_PATTEM

#ifdef OV8831_DEBUG
//#define SENSORDB printk
#else
//#define SENSORDB(x,...)
#endif

//#define OV8831_2_LANE  // if you use 2 lane setting on MT6589, please define it


#define OV8831_FACTORY_START_ADDR 0
#define OV8831_ENGINEER_START_ADDR 10

//#define MIPI_INTERFACE

 
typedef enum OV8831_group_enum
{
  OV8831_PRE_GAIN = 0,
  OV8831_CMMCLK_CURRENT,
  OV8831_FRAME_RATE_LIMITATION,
  OV8831_REGISTER_EDITOR,
  OV8831_GROUP_TOTAL_NUMS
} OV8831_FACTORY_GROUP_ENUM;

typedef enum OV8831_register_index
{
  OV8831_SENSOR_BASEGAIN = OV8831_FACTORY_START_ADDR,
  OV8831_PRE_GAIN_R_INDEX,
  OV8831_PRE_GAIN_Gr_INDEX,
  OV8831_PRE_GAIN_Gb_INDEX,
  OV8831_PRE_GAIN_B_INDEX,
  OV8831_FACTORY_END_ADDR
} OV8831_FACTORY_REGISTER_INDEX;

typedef enum OV8831_engineer_index
{
  OV8831_CMMCLK_CURRENT_INDEX = OV8831_ENGINEER_START_ADDR,
  OV8831_ENGINEER_END
} OV8831_FACTORY_ENGINEER_INDEX;

typedef struct _sensor_data_struct
{
  SENSOR_REG_STRUCT reg[OV8831_ENGINEER_END];
  SENSOR_REG_STRUCT cct[OV8831_FACTORY_END_ADDR];
} sensor_data_struct;

/* SENSOR PREVIEW/CAPTURE VT CLOCK */
//#define OV8831_PREVIEW_CLK                     69333333  //48100000
//#define OV8831_CAPTURE_CLK                     69333333  //48100000

#define OV8831_PREVIEW_CLK     138666667  //65000000
#define OV8831_CAPTURE_CLK     134333333  //117000000  //69333333
#ifdef OV8831_2_LANE
#define OV8831_VIDEO_CLK       208000000
#else
#define OV8831_VIDEO_CLK       221000000
#endif
#define OV8831_ZSD_PRE_CLK     134333333 //117000000  

#define OV8831_COLOR_FORMAT                    SENSOR_OUTPUT_FORMAT_RAW_R //SENSOR_OUTPUT_FORMAT_RAW_R, SENSOR_OUTPUT_FORMAT_RAW_B,SENSOR_OUTPUT_FORMAT_RAW_Gr

#define OV8831_MIN_ANALOG_GAIN				1	/* 1x */
#define OV8831_MAX_ANALOG_GAIN				32	/* 32x */


/* FRAME RATE UNIT */
#define OV8831_FPS(x)                          (10 * (x))

/* SENSOR PIXEL/LINE NUMBERS IN ONE PERIOD */
//#define OV8831_FULL_PERIOD_PIXEL_NUMS          2700 /* 9 fps */
#ifdef OV8831_2_LANE
#define OV8831_FULL_PERIOD_PIXEL_NUMS          (3608 + 200)  //3055 /* 8 fps */
#else
#define OV8831_FULL_PERIOD_PIXEL_NUMS          3608  //3055 /* 8 fps */
#endif
#define OV8831_FULL_PERIOD_LINE_NUMS           2484  //1968
#define OV8831_PV_PERIOD_PIXEL_NUMS            3608  //1630 /* 30 fps */
#define OV8831_PV_PERIOD_LINE_NUMS             1260  //984
#define OV8831_VIDEO_PERIOD_PIXEL_NUMS         3688  //1630 /* 30 fps */
#define OV8831_VIDEO_PERIOD_LINE_NUMS          1994  //984

#ifdef OV8831_2_LANE
#define OV8831_3D_FULL_PERIOD_PIXEL_NUMS       (3608 + 200) /* 15 fps */
#else
#define OV8831_3D_FULL_PERIOD_PIXEL_NUMS       3608 /* 15 fps */
#endif
#define OV8831_3D_FULL_PERIOD_LINE_NUMS        2484
#define OV8831_3D_PV_PERIOD_PIXEL_NUMS         3608 /* 30 fps */
#define OV8831_3D_PV_PERIOD_LINE_NUMS          1260
#define OV8831_3D_VIDEO_PERIOD_PIXEL_NUMS      3608 /* 30 fps */
#define OV8831_3D_VIDEO_PERIOD_LINE_NUMS       1260


/* SENSOR START/END POSITION */
#define OV8831_FULL_X_START                    9
#define OV8831_FULL_Y_START                    11
#define OV8831_IMAGE_SENSOR_FULL_WIDTH         (3264 - 64) /* 2560 */
#define OV8831_IMAGE_SENSOR_FULL_HEIGHT        (2448 - 48) /* 1920 */

#define OV8831_PV_X_START                      5 //5
#define OV8831_PV_Y_START                      5
#define OV8831_IMAGE_SENSOR_PV_WIDTH           (1600)  //    (1280 - 16) /* 1264 */
#define OV8831_IMAGE_SENSOR_PV_HEIGHT          (1200)  //(960 - 12) /* 948 */

#define OV8831_VIDEO_X_START                   9//9
#define OV8831_VIDEO_Y_START                   11
#define OV8831_IMAGE_SENSOR_VIDEO_WIDTH        (3264 - 64) /* 1264 */
#define OV8831_IMAGE_SENSOR_VIDEO_HEIGHT       (1836 - 48) /* 948 */

#define OV8831_3D_FULL_X_START                 9   //(1+16+6)
#define OV8831_3D_FULL_Y_START                 11  //(1+12+4)
#define OV8831_IMAGE_SENSOR_3D_FULL_WIDTH      (3264 - 32) //(2592 - 16) /* 2560 */
#define OV8831_IMAGE_SENSOR_3D_FULL_HEIGHT     (2448 - 24) //(1944 - 12) /* 1920 */
#define OV8831_3D_PV_X_START                   5
#define OV8831_3D_PV_Y_START                   5
#define OV8831_IMAGE_SENSOR_3D_PV_WIDTH        (1600) /* 1600 */
#define OV8831_IMAGE_SENSOR_3D_PV_HEIGHT       (1200) /* 1200 */
#define OV8831_3D_VIDEO_X_START                5
#define OV8831_3D_VIDEO_Y_START                5
#define OV8831_IMAGE_SENSOR_3D_VIDEO_WIDTH     (1600) /* 1600 */
#define OV8831_IMAGE_SENSOR_3D_VIDEO_HEIGHT    (1200) /* 1200 */


/* SENSOR READ/WRITE ID */
#define OV8831_SLAVE_WRITE_ID_1   (0x6c)
#define OV8831_SLAVE_WRITE_ID_2   (0x20)

#define OV8831_WRITE_ID   (0x20)  //(0x6c)
#define OV8831_READ_ID    (0x21)  //(0x6d)

/* SENSOR ID */
//#define OV8831_SENSOR_ID						(0x5647)


/************OTP Feature*********************/
//#define OV8831_USE_OTP
#if defined(OV8831_USE_OTP)

struct ov8831_otp_struct
{
    kal_uint16 customer_id;
	kal_uint16 module_integrator_id;
	kal_uint16 lens_id;
	kal_uint16 rg_ratio;
	kal_uint16 bg_ratio;
	kal_uint16 light_rg;
	kal_uint16 light_bg;
	kal_uint16 user_data[5];
	kal_uint16 lenc[62];//63
};
#define RG_TYPICAL 0x147
#define BG_TYPICAL 0x15c
#endif
/************OTP Feature*********************/

/* SENSOR PRIVATE STRUCT */
typedef struct OV8831_sensor_STRUCT
{
  MSDK_SENSOR_CONFIG_STRUCT cfg_data;
  sensor_data_struct eng; /* engineer mode */
  MSDK_SENSOR_ENG_INFO_STRUCT eng_info;
  kal_uint8 mirror;
  kal_bool pv_mode;
  kal_bool video_mode;  
  //kal_bool NightMode;
  kal_bool is_zsd;
  kal_bool is_zsd_cap;
  kal_bool is_autofliker;
  //kal_uint16 normal_fps; /* video normal mode max fps */
  //kal_uint16 night_fps; /* video night mode max fps */  
  kal_uint16 FixedFps;
  kal_uint16 shutter;
  kal_uint16 gain;
  kal_uint32 pv_pclk;
  kal_uint32 cap_pclk;
  kal_uint32 pclk;
  kal_uint16 frame_height;
  kal_uint16 line_length;  
  kal_uint16 write_id;
  kal_uint16 read_id;
  kal_uint16 dummy_pixels;
  kal_uint16 dummy_lines;
} OV8831_sensor_struct;

//export functions
UINT32 OV8831Open(void);
UINT32 OV8831Control(MSDK_SCENARIO_ID_ENUM ScenarioId, MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *pImageWindow, MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData);
UINT32 OV8831FeatureControl(MSDK_SENSOR_FEATURE_ENUM FeatureId, UINT8 *pFeaturePara,UINT32 *pFeatureParaLen);
UINT32 OV8831GetInfo(MSDK_SCENARIO_ID_ENUM ScenarioId, MSDK_SENSOR_INFO_STRUCT *pSensorInfo, MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData);
UINT32 OV8831GetResolution(MSDK_SENSOR_RESOLUTION_INFO_STRUCT *pSensorResolution);
UINT32 OV8831Close(void);

#define Sleep(ms) mdelay(ms)

#endif 
