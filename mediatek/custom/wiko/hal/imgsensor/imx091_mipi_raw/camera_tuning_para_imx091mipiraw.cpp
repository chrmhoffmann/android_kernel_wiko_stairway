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
#include "camera_AE_PLineTable_imx091mipiraw.h"
#include "camera_info_imx091mipiraw.h"
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
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
        }
    },
    ISPPca:{
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
        73750,    // i4R_AVG
        15257,    // i4R_STD
        99025,    // i4B_AVG
        22609,    // i4B_STD
        {  // i4P00[9]
            5017500, -2620000, 162500, -797500, 3747500, -397500, 40000, -2430000, 4950000
        },
        {  // i4P10[9]
            1513337, -1643557, 130220, 12258, -25355, 16093, 28642, -500955, 472313
        },
        {  // i4P01[9]
            1340062, -1404539, 64478, -251836, 77978, 180508, -78002, -1366752, 1444754
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
            1144,    // u4MinGain, 1024 base = 1x
            8192,    // u4MaxGain, 16x
            118,    // u4MiniISOGain, ISOxx  
            128,    // u4GainStepUnit, 1x/8 
            20478,    // u4PreExpUnit 
            30,    // u4PreMaxFrameRate
            20478,    // u4VideoExpUnit  
            30,    // u4VideoMaxFrameRate 
            1024,    // u4Video2PreRatio, 1024 base = 1x 
            20910,    // u4CapExpUnit 
            15,    // u4CapMaxFrameRate
            1024,    // u4Cap2PreRatio, 1024 base = 1x
            24,    // u4LensFno, Fno = 2.8
            350    // u4FocusLength_100x
        },
        // rHistConfig
        {
            4,    // u4HistHighThres
            40,    // u4HistLowThres
            2,    // u4MostBrightRatio
            1,    // u4MostDarkRatio
            160,    // u4CentralHighBound
            20,    // u4CentralLowBound
            {240, 230, 220, 210, 200},    // u4OverExpThres[AE_CCT_STRENGTH_NUM] 
            {82, 108, 128, 148, 170},    // u4HistStretchThres[AE_CCT_STRENGTH_NUM] 
            {18, 22, 26, 30, 34}    // u4BlackLightThres[AE_CCT_STRENGTH_NUM] 
        },
        // rCCTConfig
        {
            TRUE,    // bEnableBlackLight
            TRUE,    // bEnableHistStretch
            FALSE,    // bEnableAntiOverExposure
            TRUE,    // bEnableTimeLPF
            TRUE,    // bEnableCaptureThres
            TRUE,    // bEnableVideoThres
            TRUE,    // bEnableStrobeThres
            47,    // u4AETarget
            47,    // u4StrobeAETarget
            50,    // u4InitIndex
            4,    // u4BackLightWeight
            32,    // u4HistStretchWeight
            4,    // u4AntiOverExpWeight
            2,    // u4BlackLightStrengthIndex
            0,    // u4HistStretchStrengthIndex
            2,    // u4AntiOverExpStrengthIndex
            2,    // u4TimeLPFStrengthIndex
            {1, 3, 5, 7, 8},    // u4LPFConvergeTable[AE_CCT_STRENGTH_NUM] 
            90,    // u4InDoorEV = 9.0, 10 base 
            -28,    // i4BVOffset delta BV = value/10 
            64,    // u4PreviewFlareOffset
            64,    // u4CaptureFlareOffset
            4,    // u4CaptureFlareThres
            64,    // u4VideoFlareOffset
            4,    // u4VideoFlareThres
            32,    // u4StrobeFlareOffset
            2,    // u4StrobeFlareThres
            160,    // u4PrvMaxFlareThres
            0,    // u4PrvMinFlareThres
            160,    // u4VideoMaxFlareThres
            0,    // u4VideoMinFlareThres
            18,    // u4FlatnessThres    // 10 base for flatness condition.
            75    // u4FlatnessStrength
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
                965,    // i4R
                512,    // i4G
                680    // i4B
            }
        },
        // Original XY coordinate of AWB light source
        {
           // Strobe
            {
                76,    // i4X
                -376    // i4Y
            },
            // Horizon
            {
                -383,    // i4X
                -385    // i4Y
            },
            // A
            {
                -266,    // i4X
                -390    // i4Y
            },
            // TL84
            {
                -144,    // i4X
                -366    // i4Y
            },
            // CWF
            {
                -92,    // i4X
                -411    // i4Y
            },
            // DNP
            {
                -22,    // i4X
                -382    // i4Y
            },
            // D65
            {
                129,    // i4X
                -339    // i4Y
            },
            // DF
            {
                79,    // i4X
                -395    // i4Y
            }
        },
        // Rotated XY coordinate of AWB light source
        {
            // Strobe
            {
                36,    // i4X
                -383    // i4Y
            },
            // Horizon
            {
                -422,    // i4X
                -343    // i4Y
            },
            // A
            {
                -306,    // i4X
                -360    // i4Y
            },
            // TL84
            {
                -182,    // i4X
                -349    // i4Y
            },
            // CWF
            {
                -135,    // i4X
                -400    // i4Y
            },
            // DNP
            {
                -62,    // i4X
                -378    // i4Y
            },
            // D65
            {
                93,    // i4X
                -351    // i4Y
            },
            // DF
            {
                37,    // i4X
                -402    // i4Y
            }
        },
        // AWB gain of AWB light source
        {
            // Strobe 
            {
                944,    // i4R
                512,    // i4G
                768    // i4B
            },
            // Horizon 
            {
                514,    // i4R
                512,    // i4G
                1448    // i4B
            },
            // A 
            {
                606,    // i4R
                512,    // i4G
                1245    // i4B
            },
            // TL84 
            {
                692,    // i4R
                512,    // i4G
                1022    // i4B
            },
            // CWF 
            {
                788,    // i4R
                512,    // i4G
                1011    // i4B
            },
            // DNP 
            {
                834,    // i4R
                512,    // i4G
                885    // i4B
            },
            // D65 
            {
                965,    // i4R
                512,    // i4G
                680    // i4B
            },
            // DF 
            {
                973,    // i4R
                512,    // i4G
                785    // i4B
            }
        },
        // Rotation matrix parameter
        {
            6,    // i4RotationAngle
            255,    // i4Cos
            27    // i4Sin
        },
        // Daylight locus parameter
        {
            -158,    // i4SlopeNumerator
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
            -232,    // i4RightBound
            -882,    // i4LeftBound
            -301,    // i4UpperBound
            -401    // i4LowerBound
            },
            // Warm fluorescent
            {
            -232,    // i4RightBound
            -882,    // i4LeftBound
            -401,    // i4UpperBound
            -521    // i4LowerBound
            },
            // Fluorescent
            {
            -112,    // i4RightBound
            -232,    // i4LeftBound
            -284,    // i4UpperBound
            -374    // i4LowerBound
            },
            // CWF
            {
            -112,    // i4RightBound
            -232,    // i4LeftBound
            -374,    // i4UpperBound
            -500    // i4LowerBound
            },
            // Daylight
            {
            118,    // i4RightBound
            -112,    // i4LeftBound
            -271,    // i4UpperBound
            -431    // i4LowerBound
            },
            // Shade
            {
            478,    // i4RightBound
            118,    // i4LeftBound
            -271,    // i4UpperBound
            -431    // i4LowerBound
            },
            // Daylight Fluorescent
            {
            118,    // i4RightBound
            -112,    // i4LeftBound
            -431,    // i4UpperBound
            -510    // i4LowerBound
            }
        },
        // PWB light area
        {
            // Reference area
            {
            478,    // i4RightBound
            -882,    // i4LeftBound
            0,    // i4UpperBound
            -521    // i4LowerBound
            },
            // Daylight
            {
            143,    // i4RightBound
            -112,    // i4LeftBound
            -271,    // i4UpperBound
            -431    // i4LowerBound
            },
            // Cloudy daylight
            {
            243,    // i4RightBound
            68,    // i4LeftBound
            -271,    // i4UpperBound
            -431    // i4LowerBound
            },
            // Shade
            {
            343,    // i4RightBound
            68,    // i4LeftBound
            -271,    // i4UpperBound
            -431    // i4LowerBound
            },
            // Twilight
            {
            -112,    // i4RightBound
            -272,    // i4LeftBound
            -271,    // i4UpperBound
            -431    // i4LowerBound
            },
            // Fluorescent
            {
            143,    // i4RightBound
            -282,    // i4LeftBound
            -299,    // i4UpperBound
            -450    // i4LowerBound
            },
            // Warm fluorescent
            {
            -206,    // i4RightBound
            -406,    // i4LeftBound
            -299,    // i4UpperBound
            -450    // i4LowerBound
            },
            // Incandescent
            {
            -206,    // i4RightBound
            -406,    // i4LeftBound
            -271,    // i4UpperBound
            -431    // i4LowerBound
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
            879,    // i4R
            512,    // i4G
            763    // i4B
            },
            // Cloudy daylight
            {
            1040,    // i4R
            512,    // i4G
            620    // i4B
            },
            // Shade
            {
            1104,    // i4R
            512,    // i4G
            575    // i4B
            },
            // Twilight
            {
            685,    // i4R
            512,    // i4G
            1038    // i4B
            },
            // Fluorescent
            {
            822,    // i4R
            512,    // i4G
            890    // i4B
            },
            // Warm fluorescent
            {
            618,    // i4R
            512,    // i4G
            1265    // i4B
            },
            // Incandescent
            {
            597,    // i4R
            512,    // i4G
            1230    // i4B
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
            6140    // i4OffsetThr
            },
            // Warm fluorescent	
            {
            0,    // i4SliderValue
            5451    // i4OffsetThr
            },
            // Shade
            {
            50,    // i4SliderValue
            342    // i4OffsetThr
            },
            // Daylight WB gain
            {
            801,    // i4R
            512,    // i4G
            856    // i4B
            },
            // Preference gain: strobe
            {
            512,    // i4R
            512,    // i4G
            512    // i4B
            },
            // Preference gain: tungsten
            {
            495,    // i4R
            512,    // i4G
            520    // i4B
            },
            // Preference gain: warm fluorescent
            {
            490,    // i4R
            512,    // i4G
            518    // i4B
            },
            // Preference gain: fluorescent
            {
            496,    // i4R
            512,    // i4G
            512    // i4B
            },
            // Preference gain: CWF
            {
            508,    // i4R
            512,    // i4G
            528    // i4B
            },
            // Preference gain: daylight
            {
            516,    // i4R
            512,    // i4G
            508    // i4B
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
                -515,    // i4RotatedXCoordinate[0]
                -399,    // i4RotatedXCoordinate[1]
                -275,    // i4RotatedXCoordinate[2]
                -155,    // i4RotatedXCoordinate[3]
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
}}; // NSFeature


