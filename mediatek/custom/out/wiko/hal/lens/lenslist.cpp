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
#include <utils/Errors.h>
#include <fcntl.h>
#include <math.h>

#include "MediaHal.h"

//#include "lens_custom_cfg.h"
//#include "msdk_lens_exp.h"
#include "camera_custom_lens.h"
//#include "lens.h"
//nclude "image_sensor.h"
#include "kd_imgsensor.h"

extern PFUNC_GETLENSDEFAULT pDummy_getDefaultData;

#if defined(SENSORDRIVE)
extern PFUNC_GETLENSDEFAULT pSensorDrive_getDefaultData;
#endif

#if defined(FM50AF)
extern PFUNC_GETLENSDEFAULT pFM50AF_getDefaultData;
#endif


#if defined(OV8825AF)
extern PFUNC_GETLENSDEFAULT pOV8825AF_getDefaultData;
#endif

#if defined(OV8830AF)
extern PFUNC_GETLENSDEFAULT pOV8830AF_getDefaultData;
#endif

#if defined(OV8831AF)
extern PFUNC_GETLENSDEFAULT pOV8831AF_getDefaultData;
#endif

#if defined(OV12830AF)
extern PFUNC_GETLENSDEFAULT pOV12830AF_getDefaultData;
#endif

#if defined(IMX135AF)
extern PFUNC_GETLENSDEFAULT pIMX135AF_getDefaultData;
#endif

#if defined(IMX091AF)
extern PFUNC_GETLENSDEFAULT pIMX091AF_getDefaultData;
#endif

#if defined(IMX092AF)
extern PFUNC_GETLENSDEFAULT pIMX092AF_getDefaultData;
#endif

MSDK_LENS_INIT_FUNCTION_STRUCT LensList[MAX_NUM_OF_SUPPORT_LENS] =
{
	{DUMMY_SENSOR_ID, DUMMY_LENS_ID, "Dummy", pDummy_getDefaultData},

#if defined(SENSORDRIVE)
	{DUMMY_SENSOR_ID, SENSOR_DRIVE_LENS_ID, "kd_camera_hw", pSensorDrive_getDefaultData},	

    //  for backup lens, need assign correct SensorID
    //{OV5642_SENSOR_ID, SENSOR_DRIVE_LENS_ID, "kd_camera_hw", pSensorDrive_getDefaultData},
#endif

#if defined(OV8825AF)
		{OV8825_SENSOR_ID, OV8825AF_LENS_ID, "OV8825AF", pOV8825AF_getDefaultData},
#endif
#if defined(OV8830AF)
		{OV8830_SENSOR_ID, OV8830AF_LENS_ID, "OV8830AF", pOV8830AF_getDefaultData},
#endif
#if defined(OV8831AF)
		{OV8831_SENSOR_ID, OV8831AF_LENS_ID, "OV8831AF", pOV8831AF_getDefaultData},
#endif
#if defined(OV12830AF)
		{OV12830_SENSOR_ID, OV12830AF_LENS_ID, "OV12830AF", pOV12830AF_getDefaultData},
#endif
#if defined(IMX135AF)
		{IMX135_SENSOR_ID, IMX135AF_LENS_ID, "IMX135AF", pIMX135AF_getDefaultData},
#endif
#if defined(IMX091AF)
		{IMX091MIPI_SENSOR_ID, IMX091AF_LENS_ID, "IMX091AF", pIMX091AF_getDefaultData},
#endif
#if defined(IMX092AF)
		{IMX092MIPI_SENSOR_ID, IMX092AF_LENS_ID, "IMX092AF", pIMX092AF_getDefaultData},
#endif
#if defined(FM50AF)
	{DUMMY_SENSOR_ID, FM50AF_LENS_ID, "FM50AF", pFM50AF_getDefaultData},
#endif

    //  for new added lens, need assign correct SensorID

};

UINT32 GetLensInitFuncList(PMSDK_LENS_INIT_FUNCTION_STRUCT pLensList)
{
    memcpy(pLensList, &LensList[0], sizeof(MSDK_LENS_INIT_FUNCTION_STRUCT)* MAX_NUM_OF_SUPPORT_LENS);
    return MHAL_NO_ERROR;
} // GetLensInitFuncList()






