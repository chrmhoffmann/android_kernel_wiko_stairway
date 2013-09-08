#ifndef _IMX091AF_H
#define _IMX091AF_H

#include <linux/ioctl.h>
//#include "kd_imgsensor.h"

#define IMX091AF_MAGIC 'A'
//IOCTRL(inode * ,file * ,cmd ,arg )


//Structures
typedef struct {
//current position
unsigned long u4CurrentPosition;
//macro position
unsigned long u4MacroPosition;
//Infiniti position
unsigned long u4InfPosition;
//Motor Status
bool          bIsMotorMoving;
//Motor Open?
bool          bIsMotorOpen;
//Support SR?
bool          bIsSupportSR;
} stIMX091AF_MotorInfo;

//Control commnad
//S means "set through a ptr"
//T means "tell by a arg value"
//G means "get by a ptr"             
//Q means "get by return a value"
//X means "switch G and S atomically"
//H means "switch T and Q atomically"
#define IMX091AFIOC_G_MOTORINFO _IOR(IMX091AF_MAGIC,0,stIMX091AF_MotorInfo)

#define IMX091AFIOC_T_MOVETO _IOW(IMX091AF_MAGIC,1,unsigned long)

#define IMX091AFIOC_T_SETINFPOS _IOW(IMX091AF_MAGIC,2,unsigned long)

#define IMX091AFIOC_T_SETMACROPOS _IOW(IMX091AF_MAGIC,3,unsigned long)

#else
#endif
