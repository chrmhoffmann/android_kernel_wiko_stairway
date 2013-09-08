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

/********************************************************************************************
 *     LEGAL DISCLAIMER
 *
 *     (Header of MediaTek Software/Firmware Release or Documentation)
 *
 *     BY OPENING OR USING THIS FILE, BUYER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES
 *     THAT THE SOFTWARE/FIRMWARE AND ITS DOCUMENTATIONS ("MEDIATEK SOFTWARE") RECEIVED
 *     FROM MEDIATEK AND/OR ITS REPRESENTATIVES ARE PROVIDED TO BUYER ON AN "AS-IS" BASIS
 *     ONLY. MEDIATEK EXPRESSLY DISCLAIMS ANY AND ALL WARRANTIES, EXPRESS OR IMPLIED,
 *     INCLUDING BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR
 *     A PARTICULAR PURPOSE OR NONINFRINGEMENT. NEITHER DOES MEDIATEK PROVIDE ANY WARRANTY
 *     WHATSOEVER WITH RESPECT TO THE SOFTWARE OF ANY THIRD PARTY WHICH MAY BE USED BY,
 *     INCORPORATED IN, OR SUPPLIED WITH THE MEDIATEK SOFTWARE, AND BUYER AGREES TO LOOK
 *     ONLY TO SUCH THIRD PARTY FOR ANY WARRANTY CLAIM RELATING THERETO. MEDIATEK SHALL ALSO
 *     NOT BE RESPONSIBLE FOR ANY MEDIATEK SOFTWARE RELEASES MADE TO BUYER'S SPECIFICATION
 *     OR TO CONFORM TO A PARTICULAR STANDARD OR OPEN FORUM.
 *
 *     BUYER'S SOLE AND EXCLUSIVE REMEDY AND MEDIATEK'S ENTIRE AND CUMULATIVE LIABILITY WITH
 *     RESPECT TO THE MEDIATEK SOFTWARE RELEASED HEREUNDER WILL BE, AT MEDIATEK'S OPTION,
TO REVISE OR REPLACE THE MEDIATEK SOFTWARE AT ISSUE, OR REFUND ANY SOFTWARE LICENSE
 *     FEES OR SERVICE CHARGE PAID BY BUYER TO MEDIATEK FOR SUCH MEDIATEK SOFTWARE AT ISSUE.
 *
 *     THE TRANSACTION CONTEMPLATED HEREUNDER SHALL BE CONSTRUED IN ACCORDANCE WITH THE LAWS
 *     OF THE STATE OF CALIFORNIA, USA, EXCLUDING ITS CONFLICT OF LAWS PRINCIPLES.
 ************************************************************************************************/
#include <utils/Log.h>
#include <fcntl.h>
#include <math.h>

#include "camera_custom_nvram.h"
#include "camera_custom_sensor.h"
#include "image_sensor.h"
#include "kd_imgsensor_define.h"
#include "camera_AE_PLineTable_imx135raw.h"
#include "camera_info_imx135raw.h"
#include "camera_custom_AEPlinetable.h"

const NVRAM_CAMERA_ISP_PARAM_STRUCT CAMERA_ISP_DEFAULT_VALUE =
{{
    //Version
    Version: NVRAM_CAMERA_PARA_FILE_VERSION,

    //SensorId
    SensorId: SENSOR_ID,
    ISPComm:{
        {
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    	}
    },
    ISPPca: {
        #include INCLUDE_FILENAME_ISP_PCA_PARAM
    },
    ISPRegs:{
        #include INCLUDE_FILENAME_ISP_REGS_PARAM
    },
    ISPMfbMixer:{{
        {//00: MFB mixer for ISO 100
            0x00000000, 0x00000000
        },
        {//01: MFB mixer for ISO 200
            0x00000000, 0x00000000
        },
        {//02: MFB mixer for ISO 400
            0x00000000, 0x00000000
        },
        {//03: MFB mixer for ISO 800
            0x00000000, 0x00000000
        },
        {//04: MFB mixer for ISO 1600
            0x00000000, 0x00000000
        },
        {//05: MFB mixer for ISO 2400
            0x00000000, 0x00000000
        },
        {//06: MFB mixer for ISO 3200
            0x00000000, 0x00000000
    }
    }},
    ISPCcmPoly22:{
        71975,    // i4R_AVG
        16048,    // i4R_STD
        109575,    // i4B_AVG
        25427,    // i4B_STD
        {  // i4P00[9]
            5817500, -3360000, 102500, -1112500, 4225000, -552500, -60000, -3015000, 5635000
        },
        {  // i4P10[9]
            1616520, -2016307, 412097, -50620, -37688, 88308, 4354, -171361, 164814
        },
        {  // i4P01[9]
            1348515, -1744679, 411386, -315078, -67856, 382934, -178112, -1173235, 1357367
        },
        {  // i4P20[9]
            0, 0, 0, 0, 0, 0, 0, 0, 0
        },
        {  // i4P11[9]
            0, 0, 0, 0, 0, 0, 0, 0, 0
        },
        {  // i4P02[9]
            0, 0, 0, 0, 0, 0, 0, 0, 0
        }
    }
}};

const NVRAM_CAMERA_3A_STRUCT CAMERA_3A_NVRAM_DEFAULT_VALUE =
{
    NVRAM_CAMERA_3A_FILE_VERSION, // u4Version
    SENSOR_ID, // SensorId

    // AE NVRAM
    {
        // rDevicesInfo
        {
            1136,    // u4MinGain, 1024 base = 1x
            8192,    // u4MaxGain, 16x
            85,    // u4MiniISOGain, ISOxx  
            128,    // u4GainStepUnit, 1x/8 
            21,    // u4PreExpUnit 
            31,    // u4PreMaxFrameRate
            21,    // u4VideoExpUnit  
            31,    // u4VideoMaxFrameRate 
            1024,    // u4Video2PreRatio, 1024 base = 1x 
            22,    // u4CapExpUnit 
            15,    // u4CapMaxFrameRate
            1024,    // u4Cap2PreRatio, 1024 base = 1x
            22,    // u4LensFno, Fno = 2.8
            350    // u4FocusLength_100x
         },
         // rHistConfig
        {
            4, // 2,   // u4HistHighThres
            40,  // u4HistLowThres
            2,   // u4MostBrightRatio
            1,   // u4MostDarkRatio
            160, // u4CentralHighBound
            20,  // u4CentralLowBound
            {240, 230, 220, 210, 200}, // u4OverExpThres[AE_CCT_STRENGTH_NUM]
            {82, 108, 100, 148, 170},  // u4HistStretchThres[AE_CCT_STRENGTH_NUM]
            {18, 22, 65, 30, 34}    // u4BlackLightThres[AE_CCT_STRENGTH_NUM]
        },
        // rCCTConfig
        {
            TRUE,            // bEnableBlackLight
            TRUE,            // bEnableHistStretch
            FALSE,           // bEnableAntiOverExposure
            TRUE,            // bEnableTimeLPF
            TRUE,            // bEnableCaptureThres
            TRUE,            // bEnableVideoThres
            TRUE,            // bEnableStrobeThres
            47,                // u4AETarget
            47,                // u4StrobeAETarget

            50,                // u4InitIndex
            10,                 // u4BackLightWeight
            32,                // u4HistStretchWeight
            4,                 // u4AntiOverExpWeight
            2,                 // u4BlackLightStrengthIndex
            2,    // u4HistStretchStrengthIndex
            2,                 // u4AntiOverExpStrengthIndex
            2,                 // u4TimeLPFStrengthIndex
            {1, 3, 5, 7, 8}, // u4LPFConvergeTable[AE_CCT_STRENGTH_NUM]
            90,                // u4InDoorEV = 9.0, 10 base
            -8,    // i4BVOffset delta BV = value/10 
            64,                 // u4PreviewFlareOffset
            64,                 // u4CaptureFlareOffset
            5,    // u4CaptureFlareThres
            64,                 // u4VideoFlareOffset
            5,    // u4VideoFlareThres
            32,                 // u4StrobeFlareOffset
            2,                 // u4StrobeFlareThres
            160,                 // u4PrvMaxFlareThres
            0,                 // u4PrvMinFlareThres
            160,                 // u4VideoMaxFlareThres
            0,                 // u4VideoMinFlareThres            
            18,                // u4FlatnessThres              // 10 base for flatness condition.
            75                 // u4FlatnessStrength
         }
    },

    // AWB NVRAM
{								
	// AWB calibration data							
	{							
		// rCalGain (calibration gain: 1.0 = 512)						
		{						
			0,	// u4R				
			0,	// u4G				
			0	// u4B				
		},						
		// rDefGain (Default calibration gain: 1.0 = 512)						
		{						
			0,	// u4R				
			0,	// u4G				
			0	// u4B				
		},						
		// rDefGain (Default calibration gain: 1.0 = 512)						
		{						
			0,	// u4R				
			0,	// u4G				
			0	// u4B				
		},						
		// rD65Gain (D65 WB gain: 1.0 = 512)						
		{						
                951,    // i4R
                512,    // i4G
                753    // i4B
		}						
	},							
	// Original XY coordinate of AWB light source							
	{							
		// Strobe						
		{						
                -24,    // i4X
                -426    // i4Y
		},						
		// Horizon						
		{						
                -414,    // i4X
                -396    // i4Y
		},						
		// A						
		{						
                -311,    // i4X
                -411    // i4Y
            },
            // TL84
            {
                -208,    // i4X
                -394    // i4Y
            },
            // CWF
            {
                -166,    // i4X
                -464    // i4Y
            },
            // DNP
            {
                -55,    // i4X
                -411    // i4Y
            },
            // D65
            {
                86,    // i4X
                -371    // i4Y
            },
            // DF
            {
                22,    // i4X
                -425    // i4Y
            }
        },
        // Rotated XY coordinate of AWB light source
        {
            // Strobe
            {
                -54,    // i4X
                -423    // i4Y
            },
            // Horizon
            {
                -440,    // i4X
                -365    // i4Y
            },
            // A
            {
                -339,    // i4X
                -388    // i4Y
            },
            // TL84
            {
                -235,    // i4X
                -378    // i4Y
            },
            // CWF
            {
                -198,    // i4X
                -451    // i4Y
            },
            // DNP
            {
                -84,    // i4X
                -406    // i4Y
            },
            // D65
            {
                60,    // i4X
                -376    // i4Y
            },
            // DF
            {
                -8,    // i4X
                -440    // i4Y //-425
            }
        },
        // AWB gain of AWB light source
        {
            // Strobe 
            {
                883,    // i4R
                512,    // i4G
                941    // i4B
            },
            // Horizon 
            {
                512,    // i4R
                525,    // i4G
                1570    // i4B
            },
            // A 
            {
                586,    // i4R
                512,    // i4G
                1362    // i4B
            },
            // TL84 
            {
                659,    // i4R
                512,    // i4G
                1156    // i4B
            },
            // CWF 
            {
                767,    // i4R
                512,    // i4G
                1200    // i4B
            },
            // DNP 
            {
                829,    // i4R
                512,    // i4G
                962    // i4B
            },
            // D65 
            {
                951,    // i4R
                512,    // i4G
                753    // i4B
            },
            // DF 
            {
                937,    // i4R
                512,    // i4G
                883    // i4B
            }
        },
    	// Rotation matrix parameter
    	{
            4,    // i4RotationAngle
            255,    // i4Cos
            18    // i4Sin
        },
        // Daylight locus parameter
        {
            -147,    // i4SlopeNumerator
            128    // i4SlopeDenominator
        },
        // AWB light area
        {
            // Strobe:FIXME
            {
            0,    // i4RightBound
            0,    // i4LeftBound
            0,    // i4UpperBound
            0    // i4LowerBound
            },
            // Tungsten
            {
            -285,    // i4RightBound
            -935,    // i4LeftBound
            -311,    // i4UpperBound
            -426    // i4LowerBound
            },
            // Warm fluorescent
            {
            -285,    // i4RightBound
            -935,    // i4LeftBound
            -426,    // i4UpperBound
            -546    // i4LowerBound
            },
            // Fluorescent
            {
            -134,    // i4RightBound
            -285,    // i4LeftBound
            -311,    // i4UpperBound
            -414    // i4LowerBound
            },
            // CWF
            {
            -134,    // i4RightBound
            -285,    // i4LeftBound
            -414,    // i4UpperBound
            -501    // i4LowerBound
            },
            // Daylight
            {
            85,    // i4RightBound
            -134,    // i4LeftBound
            -296,    // i4UpperBound
            -456    // i4LowerBound
            },
            // Shade
            {
            445,    // i4RightBound
            85,    // i4LeftBound
            -296,    // i4UpperBound
            -456    // i4LowerBound
            },
            // Daylight Fluorescent
            {
            85,    // i4RightBound
            -134,    // i4LeftBound
            -456,    // i4UpperBound
            -546    // i4LowerBound
            }
        },
        // PWB light area
        {
            // Reference area
            {
            445,    // i4RightBound
            -935,    // i4LeftBound
            0,    // i4UpperBound
            -546    // i4LowerBound
            },
            // Daylight
            {
            110,    // i4RightBound
            -134,    // i4LeftBound
            -296,    // i4UpperBound
            -456    // i4LowerBound
            },
            // Cloudy daylight
            {
            210,    // i4RightBound
            35,    // i4LeftBound
            -296,    // i4UpperBound
            -456    // i4LowerBound
            },
            // Shade
            {
            310,    // i4RightBound
            35,    // i4LeftBound
            -296,    // i4UpperBound
            -456    // i4LowerBound
            },
            // Twilight
            {
            -134,    // i4RightBound
            -294,    // i4LeftBound
            -296,    // i4UpperBound
            -456    // i4LowerBound
            },
            // Fluorescent
            {
            110,    // i4RightBound
            -335,    // i4LeftBound
            -326,    // i4UpperBound
            -501    // i4LowerBound
            },
            // Warm fluorescent
            {
            -239,    // i4RightBound
            -439,    // i4LeftBound
            -326,    // i4UpperBound
            -501    // i4LowerBound
            },
            // Incandescent
            {
            -239,    // i4RightBound
            -439,    // i4LeftBound
            -296,    // i4UpperBound
            -456    // i4LowerBound
            },
            // Gray World
            {
            5000,    // i4RightBound
            -5000,    // i4LeftBound
            5000,    // i4UpperBound
            -5000    // i4LowerBound
            }
        },
        // PWB default gain	
        {
            // Daylight
            {
            869,    // i4R
            512,    // i4G
            836    // i4B
            },
            // Cloudy daylight
            {
            1029,    // i4R
            512,    // i4G
            688    // i4B
            },
            // Shade
            {
            1096,    // i4R
            512,    // i4G
            640    // i4B
            },
            // Twilight
            {
            674,    // i4R
            512,    // i4G
            1120    // i4B
            },
            // Fluorescent
            {
            809,    // i4R
            512,    // i4G
            1013    // i4B
            },
            // Warm fluorescent
            {
            608,    // i4R
            512,    // i4G
            1407    // i4B
            },
            // Incandescent
            {
            576,    // i4R
            512,    // i4G
            1342    // i4B
            },
            // Gray World
            {
            512,    // i4R
            512,    // i4G
            512    // i4B
            }
        },
        // AWB preference color	
        {
            // Tungsten
            {
            0,    // i4SliderValue
            5941    // i4OffsetThr
            },
            // Warm fluorescent	
            {
            0,    // i4SliderValue
            5712    // i4OffsetThr
            },
            // Shade
            {
            52,    // i4SliderValue
            301    // i4OffsetThr
            },
            // Daylight WB gain
            {
            854,    // i4R
            512,    // i4G
            600    // i4B //590 mj
            },
            // Preference gain: strobe
            {
            512,    // i4R
            512,    // i4G
            512    // i4B
            },
            // Preference gain: tungsten
            {
            485,    // i4R
            512,    // i4G
            512    // i4B //520 mj
            },
            // Preference gain: warm fluorescent
            {
            485,    // i4R
            512,    // i4G
            532    // i4B
            },
            // Preference gain: fluorescent
            {
            485,    // i4R
            512,    // i4G
            512    // i4B
            },
            // Preference gain: CWF
            {
            516,    // i4R
            512,    // i4G
            516    // i4B
            },
            // Preference gain: daylight
            {
            530,    // i4R
            512,    // i4G
            520   // i4B //512 mj 520
            },
            // Preference gain: shade
            {
            512,    // i4R
            512,    // i4G
            512    // i4B
            },
            // Preference gain: daylight fluorescent
            {
            512,    // i4R
            512,    // i4G
            512    // i4B
            }
        },
        {// CCT estimation
            {// CCT
                2300,    // i4CCT[0]
                2850,    // i4CCT[1]
                4100,    // i4CCT[2]
                5100,    // i4CCT[3]
                6500    // i4CCT[4]
            },
            {// Rotated X coordinate
                -500,    // i4RotatedXCoordinate[0]
                -399,    // i4RotatedXCoordinate[1]
                -295,    // i4RotatedXCoordinate[2]
                -144,    // i4RotatedXCoordinate[3]
                0    // i4RotatedXCoordinate[4]
            }
        }


    },
	{0}
};

#include INCLUDE_FILENAME_ISP_LSC_PARAM
//};  //  namespace


typedef NSFeature::RAWSensorInfo<SENSOR_ID> SensorInfoSingleton_T;


namespace NSFeature {
template <>
UINT32
SensorInfoSingleton_T::
impGetDefaultData(CAMERA_DATA_TYPE_ENUM const CameraDataType, VOID*const pDataBuf, UINT32 const size) const
{
    UINT32 dataSize[CAMERA_DATA_TYPE_NUM] = {sizeof(NVRAM_CAMERA_ISP_PARAM_STRUCT),
                                             sizeof(NVRAM_CAMERA_3A_STRUCT),
                                             sizeof(NVRAM_CAMERA_SHADING_STRUCT),
                                             sizeof(NVRAM_LENS_PARA_STRUCT),
                                             sizeof(AE_PLINETABLE_T)};

    if (CameraDataType > CAMERA_DATA_AE_PLINETABLE || NULL == pDataBuf || (size < dataSize[CameraDataType]))
    {
        return 1;
    }

    switch(CameraDataType)
    {
        case CAMERA_NVRAM_DATA_ISP:
            memcpy(pDataBuf,&CAMERA_ISP_DEFAULT_VALUE,sizeof(NVRAM_CAMERA_ISP_PARAM_STRUCT));
            break;
        case CAMERA_NVRAM_DATA_3A:
            memcpy(pDataBuf,&CAMERA_3A_NVRAM_DEFAULT_VALUE,sizeof(NVRAM_CAMERA_3A_STRUCT));
            break;
        case CAMERA_NVRAM_DATA_SHADING:
            memcpy(pDataBuf,&CAMERA_SHADING_DEFAULT_VALUE,sizeof(NVRAM_CAMERA_SHADING_STRUCT));
            break;
        case CAMERA_DATA_AE_PLINETABLE:
            memcpy(pDataBuf,&g_PlineTableMapping,sizeof(AE_PLINETABLE_T));
            break;
        default:
            break;
    }
    return 0;
}};  //  NSFeature


