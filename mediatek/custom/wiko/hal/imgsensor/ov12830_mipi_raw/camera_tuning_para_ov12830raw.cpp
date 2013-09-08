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
#include "camera_AE_PLineTable_ov12830raw.h"
#include "camera_info_ov12830raw.h"
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
        64100,    // i4R_AVG
        14876,    // i4R_STD
        101275,    // i4B_AVG
        20366,    // i4B_STD
        { // i4P00[9]
            4805000, -2480000, 235000, -557500, 3465000, -350000, -12500, -2857500, 5432500
        },
        { // i4P10[9]
            1876973, -2279073, 402100, 64595, -350906, 270883, 225683, -405619, 181413
        },
        { // i4P01[9]
            1593261, -1923858, 330597, -100888, -481687, 566707, 52137, -1183056, 1135666
        },
        { // i4P20[9]
            0, 0, 0, 0, 0, 0, 0, 0, 0
        },
        { // i4P11[9]
            0, 0, 0, 0, 0, 0, 0, 0, 0
        },
        { // i4P02[9]
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
            1195,   // u4MinGain, 1024 base =  1x
            16384,  // u4MaxGain, 16x
            100,     // u4MiniISOGain, ISOxx
            128,    // u4GainStepUnit, 1x/8
            33,     // u4PreExpUnit
            30,     // u4PreMaxFrameRate
            33,     // u4VideoExpUnit
            30,     // u4VideoMaxFrameRate
            1024,   // u4Video2PreRatio, 1024 base = 1x
            17,    // u4CapExpUnit 
            18,    // u4CapMaxFrameRate
            1024,   // u4Cap2PreRatio, 1024 base = 1x
            24,    // u4LensFno, Fno = 2.8
            330    // u4FocusLength_100x
         },
         // rHistConfig
        {
            2,   // u4HistHighThres
            40,  // u4HistLowThres
            2,   // u4MostBrightRatio
            1,   // u4MostDarkRatio
            160, // u4CentralHighBound
            20,  // u4CentralLowBound
            {240, 230, 220, 210, 200}, // u4OverExpThres[AE_CCT_STRENGTH_NUM]
            {86, 108, 128, 148, 170},  // u4HistStretchThres[AE_CCT_STRENGTH_NUM]
            {18, 22, 26, 30, 34}       // u4BlackLightThres[AE_CCT_STRENGTH_NUM]
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
            4,                 // u4BackLightWeight
            32,                // u4HistStretchWeight
            4,                 // u4AntiOverExpWeight
            0,    // u4BlackLightStrengthIndex
            4,    // u4HistStretchStrengthIndex
            2,                 // u4AntiOverExpStrengthIndex
            2,                 // u4TimeLPFStrengthIndex
            {1, 3, 5, 7, 8}, // u4LPFConvergeTable[AE_CCT_STRENGTH_NUM]
            90,                // u4InDoorEV = 9.0, 10 base
            -3,    // i4BVOffset delta BV = value/10 
            64,                 // u4PreviewFlareOffset
            64,                 // u4CaptureFlareOffset
            3,                 // u4CaptureFlareThres
            64,                 // u4VideoFlareOffset
            3,                 // u4VideoFlareThres
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
            // rUnitGain (unit gain: 1.0 = 512)
		{						
                0,    // i4R
                0,    // i4G
                0    // i4B
		},						
            // rGoldenGain (golden sample gain: 1.0 = 512)
		{						
                0,    // i4R
                0,    // i4G
                0    // i4B
		},						
            // rTuningUnitGain (Tuning sample unit gain: 1.0 = 512)
		{						
                0,    // i4R
                0,    // i4G
                0    // i4B
		},						
		// rD65Gain (D65 WB gain: 1.0 = 512)						
		{						
                869,    // i4R
                512,    // i4G
                698    // i4B
		}						
	},							
	// Original XY coordinate of AWB light source							
	{							
		// Strobe						
		{						
                133,    // i4X
                -301    // i4Y
		},						
		// Horizon						
		{						
                -415,    // i4X
                -333    // i4Y
		},						
		// A						
		{						
                -300,    // i4X
                -349    // i4Y
		},						
		// TL84						
		{						
                -191,    // i4X
                -314    // i4Y
		},						
		// CWF						
		{						
                -165,    // i4X
                -364    // i4Y
		},						
		// DNP						
		{						
                -63,    // i4X
                -322    // i4Y
		},						
		// D65						
		{						
                81,    // i4X
                -310    // i4Y
		},						
		// DF						
		{						
                0,    // i4X
                0    // i4Y
		}						
	},							
	// Rotated XY coordinate of AWB light source							
	{							
		// Strobe						
		{						
                118,    // i4X
                -308    // i4Y
		},						
		// Horizon						
		{						
                -432,    // i4X
                -312    // i4Y
		},						
		// A						
		{						
                -318,    // i4X
                -334    // i4Y
		},						
		// TL84						
		{						
                -207,    // i4X
                -304    // i4Y
		},						
		// CWF						
		{						
                -183,    // i4X
                -356    // i4Y
		},						
		// DNP						
		{						
                -79,    // i4X
                -319    // i4Y 
		},						
		// D65						
		{						
                65,    // i4X
                -314    // i4Y
		},						
		// DF						
		{						
                0,    // i4X
                0    // i4Y
		}						
	},							
	// AWB gain of AWB light source							
	{							
		// Strobe						
		{						
                921,    // i4R
                512,    // i4G
                642    // i4B
		},						
		// Horizon						
		{						
                512,    // i4R
                572,    // i4G
                1574    // i4B
		},						
		// A						
		{						
                547,    // i4R
                512,    // i4G
                1231    // i4B
		},						
		// TL84						
		{						
                604,    // i4R
                512,    // i4G
                1014    // i4B
		},						
		// CWF						
		{						
                670,    // i4R
                512,    // i4G
                1047    // i4B
		},						
		// DNP						
		{						
                727,    // i4R
                512,    // i4G
                862    // i4B
		},						
		// D65						
		{						
                869,    // i4R
                512,    // i4G
                698    // i4B
		},						
		// DF						
		{						
                512,    // i4R
                512,    // i4G
                512    // i4B
		}						
	},							
	// Rotation matrix parameter							
	{							
            3,    // i4RotationAngle
            256,    // i4Cos
            13    // i4Sin
	},							
	// Daylight locus parameter							
	{							
            -144,    // i4SlopeNumerator
		128	// i4SlopeDenominator					
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
            -257,    // i4RightBound
            -907,    // i4LeftBound
            -273,    // i4UpperBound
            -373    // i4LowerBound
		},						
		// Warm fluorescent						
		{						
            -257,    // i4RightBound
            -907,    // i4LeftBound
            -373,    // i4UpperBound
            -493    // i4LowerBound
		},						
		// Fluorescent						
		{						
            -129,    // i4RightBound
            -257,    // i4LeftBound
            -239,    // i4UpperBound
            -330    // i4LowerBound
		},						
		// CWF						
		{						
            -129,    // i4RightBound
            -257,    // i4LeftBound
            -330,    // i4UpperBound
            -406    // i4LowerBound
		},						
		// Daylight						
		{						
            90,    // i4RightBound
            -129,    // i4LeftBound
            -234,    // i4UpperBound
            -394    // i4LowerBound
		},						
		// Shade						
		{						
            450,    // i4RightBound
            90,    // i4LeftBound
            -234,    // i4UpperBound
            -394    // i4LowerBound
		},						
		// Daylight Fluorescent						
		{						
            90,    // i4RightBound
            -129,    // i4LeftBound
            -394,    // i4UpperBound
            -524    // i4LowerBound
		}						
	},							
	// PWB light area							
	{							
		// Reference area						
		{						
            450,    // i4RightBound
            -907,    // i4LeftBound
            0,    // i4UpperBound
            -524    // i4LowerBound
		},						
		// Daylight						
		{						
            115,    // i4RightBound
            -129,    // i4LeftBound
            -234,    // i4UpperBound
            -394    // i4LowerBound
		},						
		// Cloudy daylight						
		{						
            215,    // i4RightBound
            40,    // i4LeftBound
            -234,    // i4UpperBound
            -394    // i4LowerBound
		},						
		// Shade						
		{						
            315,    // i4RightBound
            40,    // i4LeftBound
            -234,    // i4UpperBound
            -394    // i4LowerBound
		},						
		// Twilight						
		{						
            -129,    // i4RightBound
            -289,    // i4LeftBound
            -234,    // i4UpperBound
            -394    // i4LowerBound
		},						
		// Fluorescent						
		{						
            115,    // i4RightBound
            -307,    // i4LeftBound
            -254,    // i4UpperBound
            -406    // i4LowerBound
		},						
		// Warm fluorescent						
		{						
            -218,    // i4RightBound
            -418,    // i4LeftBound
            -254,    // i4UpperBound
            -406    // i4LowerBound
		},						
		// Incandescent						
		{						
            -218,    // i4RightBound
            -418,    // i4LeftBound
            -234,    // i4UpperBound
            -394    // i4LowerBound
		},						
		// Gray World						
		{						
			5000,	// i4RightBound				
			-5000,	// i4LeftBound				
			5000,	// i4UpperBound				
			-5000	// i4LowerBound				
		}						
	},							
	// PWB default gain							
	{							
		// Daylight						
		{						
            792,    // i4R
            512,    // i4G
            773    // i4B
		},						
		// Cloudy daylight						
		{						
            941,    // i4R
            512,    // i4G
            639    // i4B
		},						
		// Shade						
		{						
            1004,    // i4R
            512,    // i4G
            595    // i4B
		},						
		// Twilight						
		{						
            612,    // i4R
            512,    // i4G
            1030    // i4B
		},						
		// Fluorescent						
		{						
            723,    // i4R
            512,    // i4G
            896    // i4B
		},						
		// Warm fluorescent						
		{						
            544,    // i4R
            512,    // i4G
            1227    // i4B
		},						
		// Incandescent						
		{						
            532,    // i4R
            512,    // i4G
            1202    // i4B
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
            5782    // i4OffsetThr
		},						
		// Warm fluorescent						
		{						
            0,    // i4SliderValue
            5394    // i4OffsetThr
		},						
		// Shade						
		{						
            50,    // i4SliderValue
            341    // i4OffsetThr
		},						
		// Daylight WB gain						
		{						
            722,    // i4R
            512,    // i4G
            857    // i4B
		},						
		// Preference gain: strobe						
		{						
            512,    // i4R
            512,    // i4G
            512    // i4B
		},						
		// Preference gain: tungsten						
		{						
            512,    // i4R
            512,    // i4G
            512    // i4B
		},						
		// Preference gain: warm fluorescent						
		{						
            512,    // i4R
            512,    // i4G
            512    // i4B
		},						
		// Preference gain: fluorescent						
		{						
            512,    // i4R
            512,    // i4G
            512    // i4B
		},						
		// Preference gain: CWF						
		{						
            512,    // i4R
            512,    // i4G
            512    // i4B
		},						
		// Preference gain: daylight						
		{						
            512,    // i4R
            512,    // i4G
            512    // i4B
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
			2300,	// i4CCT[0]				
			2850,	// i4CCT[1]				
			4100,	// i4CCT[2]				
			5100,	// i4CCT[3]				
			6500 	// i4CCT[4]				
		},						
            {// Rotated X coordinate
                -497,    // i4RotatedXCoordinate[0]
                -383,    // i4RotatedXCoordinate[1]
                -272,    // i4RotatedXCoordinate[2]
                -144,    // i4RotatedXCoordinate[3]
			0 	// i4RotatedXCoordinate[4]				
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


