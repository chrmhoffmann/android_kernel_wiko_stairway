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
#include "camera_AE_PLineTable_ov5646mipiraw.h"
#include "camera_info_ov5646mipiraw.h"
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
        69125,    // i4R_AVG
        14412,    // i4R_STD
        90700,    // i4B_AVG
        24790,    // i4B_STD
        {  // i4P00[9]
            5012500, -1827500, -630000, -805000, 3547500, -185000, 90000, -2072500, 4542500
        },
        {  // i4P10[9]
            758891, -748455, -6780, -43386, 33802, 2060, -29546, 421159, -391613
        },
        {  // i4P01[9]
            489414, -497629, 15563, -166838, -35171, 194963, -19049, -362634, 381683
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
            1136,   // u4MinGain, 1024 base =  1x
            8192,  // u4MaxGain, 16x  //
            106,     // u4MiniISOGain, ISOxx
            128,    // u4GainStepUnit, 1x/8
            34,     // u4PreExpUnit
            30,     // u4PreMaxFrameRate
            30,     // u4VideoExpUnit
            30,     // u4VideoMaxFrameRate
            1024,   // u4Video2PreRatio, 1024 base = 1x
            35,     // u4CapExpUnit
            14,     // u4CapMaxFrameRate
            1024,   // u4Cap2PreRatio, 1024 base = 1x
            24,      // u4LensFno, Fno = 2.8
            350     // u4FocusLength_100x
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
            {86, 108, 128, 148, 170},    // u4HistStretchThres[AE_CCT_STRENGTH_NUM] 
            {18, 22, 26, 30, 34}       // u4BlackLightThres[AE_CCT_STRENGTH_NUM]
        },
        // rCCTConfig
        {
            TRUE,            // bEnableBlackLight
            TRUE,            // bEnableHistStretch
            FALSE,           // bEnableAntiOverExposure
            TRUE,            // bEnableTimeLPF
            FALSE,    // bEnableCaptureThres
            FALSE,    // bEnableVideoThres
            FALSE,    // bEnableStrobeThres
            47,                // u4AETarget
            0,                // u4StrobeAETarget  //47-->65

            50,    // u4InitIndex
            4,                 // u4BackLightWeight
            32,                // u4HistStretchWeight
            4,                 // u4AntiOverExpWeight
            2,    // u4BlackLightStrengthIndex
            2,    // u4HistStretchStrengthIndex
            2,                 // u4AntiOverExpStrengthIndex
            2,                 // u4TimeLPFStrengthIndex
            {1, 3, 5, 7, 8}, // u4LPFConvergeTable[AE_CCT_STRENGTH_NUM]
            90,                // u4InDoorEV = 9.0, 10 base
            -12,    // i4BVOffset delta BV = value/10 
            64,                 // u4PreviewFlareOffset   //12bit
            64,                 // u4CaptureFlareOffset  //12bit
            5,                 // u4CaptureFlareThres
            64,                 // u4VideoFlareOffset   //12bit
            5,                 // u4VideoFlareThres
            32,                 // u4StrobeFlareOffset  //12bit
            2,                 // u4StrobeFlareThres
            8,                 // u4PrvMaxFlareThres
            2,                 // u4PrvMinFlareThres
            8,                 // u4VideoMaxFlareThres
            2,                 // u4VideoMinFlareThres            
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
    			0,	// i4R
    			0,	// i4G
    			0	// i4B
    		},
    		// rGoldenGain (golden sample gain: 1.0 = 512)
    		{
	            0,	// i4R
	            0,	// i4G
	            0	// i4B
            },
    		// rTuningUnitGain (Tuning sample unit gain: 1.0 = 512)
    		{
	            0,	// i4R
	            0,	// i4G
	            0	// i4B
            },
            // rD65Gain (D65 WB gain: 1.0 = 512)
            {
                857,    // i4R
                512,    // i4G
                558    // i4B
            }
        },
        // Original XY coordinate of AWB light source
        {
           // Strobe
            {
                159,    // i4X
                -222    // i4Y
            },
            // Horizon
            {
                -402,    // i4X
                -311    // i4Y
            },
            // A
            {
                -306,    // i4X
                -323    // i4Y
            },
            // TL84
            {
                -135,    // i4X
                -315    // i4Y
            },
            // CWF
            {
                -85,    // i4X
                -423    // i4Y
            },
            // DNP
            {
                19,    // i4X
                -262    // i4Y
            },
            // D65
            {
                159,    // i4X
                -222    // i4Y
            },
            // DF
            {
                109,    // i4X
                -334    // i4Y
            }
        },
        // Rotated XY coordinate of AWB light source
        {
            // Strobe
            {
                113,    // i4X
                -248    // i4Y
            },
            // Horizon
            {
                -454,    // i4X
                -228    // i4Y
            },
            // A
            {
                -362,    // i4X
                -258    // i4Y
            },
            // TL84
            {
                -193,    // i4X
                -283    // i4Y
            },
            // CWF
            {
                -164,    // i4X
                -398    // i4Y
            },
            // DNP
            {
                -32,    // i4X
                -261    // i4Y
            },
            // D65
            {
                113,    // i4X
                -248    // i4Y
            },
            // DF
            {
                43,    // i4X
                -348    // i4Y
            }
        },
        // AWB gain of AWB light source
        {
            // Strobe 
            {
                857,    // i4R
                512,    // i4G
                558    // i4B
            },
            // Horizon 
            {
                512,    // i4R
                579,    // i4G
                1521    // i4B
            },
            // A 
            {
                524,    // i4R
                512,    // i4G
                1199    // i4B
            },
            // TL84 
            {
                654,    // i4R
                512,    // i4G
                942    // i4B
            },
            // CWF 
            {
                808,    // i4R
                512,    // i4G
                1019    // i4B
            },
            // DNP 
            {
                749,    // i4R
                512,    // i4G
                711    // i4B
            },
            // D65 
            {
                857,    // i4R
                512,    // i4G
                558    // i4B
            },
            // DF 
            {
                934,    // i4R
                512,    // i4G
                695    // i4B
            }
        },
        // Rotation matrix parameter
        {
            11,    // i4RotationAngle
            251,    // i4Cos
            49    // i4Sin
        },
        // Daylight locus parameter
        {
            -191,    // i4SlopeNumerator
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
            -243,    // i4RightBound
            -893,    // i4LeftBound
            -180,    // i4UpperBound
            -293    // i4LowerBound
            },
            // Warm fluorescent
            {
            -243,    // i4RightBound
            -893,    // i4LeftBound
            -293,    // i4UpperBound
            -448    // i4LowerBound
            },
            // Fluorescent
            {
            -82,    // i4RightBound
            -243,    // i4LeftBound
            -180,    // i4UpperBound
            -340    // i4LowerBound
            },
            // CWF
            {
            -82,    // i4RightBound
            -243,    // i4LeftBound
            -340,    // i4UpperBound
            -448    // i4LowerBound
            },
            // Daylight
            {
            138,    // i4RightBound
            -82,    // i4LeftBound
            -168,    // i4UpperBound
            -328    // i4LowerBound
            },
            // Shade
            {
            498,    // i4RightBound
            138,    // i4LeftBound
            -168,    // i4UpperBound
            -328    // i4LowerBound
            },
            // Daylight Fluorescent
            {
            138,    // i4RightBound
            -82,    // i4LeftBound
            -328,    // i4UpperBound
            -448    // i4LowerBound
            }
        },
        // PWB light area
        {
            // Reference area
            {
            498,    // i4RightBound
            -893,    // i4LeftBound
            0,    // i4UpperBound
            -448    // i4LowerBound
            },
            // Daylight
            {
            163,    // i4RightBound
            -82,    // i4LeftBound
            -168,    // i4UpperBound
            -328    // i4LowerBound
            },
            // Cloudy daylight
            {
            263,    // i4RightBound
            88,    // i4LeftBound
            -168,    // i4UpperBound
            -328    // i4LowerBound
            },
            // Shade
            {
            363,    // i4RightBound
            88,    // i4LeftBound
            -168,    // i4UpperBound
            -328    // i4LowerBound
            },
            // Twilight
            {
            -82,    // i4RightBound
            -242,    // i4LeftBound
            -168,    // i4UpperBound
            -328    // i4LowerBound
            },
            // Fluorescent
            {
            163,    // i4RightBound
            -293,    // i4LeftBound
            -198,    // i4UpperBound
            -448    // i4LowerBound
            },
            // Warm fluorescent
            {
            -262,    // i4RightBound
            -462,    // i4LeftBound
            -198,    // i4UpperBound
            -448    // i4LowerBound
            },
            // Incandescent
            {
            -262,    // i4RightBound
            -462,    // i4LeftBound
            -168,    // i4UpperBound
            -328    // i4LowerBound
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
            793,    // i4R
            512,    // i4G
            626    // i4B
            },
            // Cloudy daylight
            {
            916,    // i4R
            512,    // i4G
            505    // i4B
            },
            // Shade
            {
            967,    // i4R
            512,    // i4G
            467    // i4B
            },
            // Twilight
            {
            639,    // i4R
            512,    // i4G
            864    // i4B
            },
            // Fluorescent
            {
            798,    // i4R
            512,    // i4G
            802    // i4B
            },
            // Warm fluorescent
            {
            581,    // i4R
            512,    // i4G
            1286    // i4B
            },
            // Incandescent
            {
            515,    // i4R
            512,    // i4G
            1187    // i4B
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
            7703    // i4OffsetThr
            },
            // Warm fluorescent	
            {
            0,    // i4SliderValue
            5947    // i4OffsetThr
            },
            // Shade
            {
            50,    // i4SliderValue
            347    // i4OffsetThr
            },
            // Daylight WB gain
            {
            721,    // i4R
            512,    // i4G
            535    // i4B
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
            508    // i4B
            },
            // Preference gain: warm fluorescent
            {
            485,    // i4R
            512,    // i4G
            508    // i4B
            },
            // Preference gain: fluorescent
            {
            508,    // i4R
            512,    // i4G
            495    // i4B
            },
            // Preference gain: CWF
            {
            508,    // i4R
            512,    // i4G
            512    // i4B
            },
            // Preference gain: daylight
            {
            512,    // i4R
            512,    // i4G
            490    // i4B
            },
            // Preference gain: shade
            {
            512,    // i4R
            512,    // i4G
            490    // i4B
            },
            // Preference gain: daylight fluorescent
            {
            512,    // i4R
            512,    // i4G
            508    // i4B
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
                -567,    // i4RotatedXCoordinate[0]
                -475,    // i4RotatedXCoordinate[1]
                -306,    // i4RotatedXCoordinate[2]
                -145,    // i4RotatedXCoordinate[3]
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


