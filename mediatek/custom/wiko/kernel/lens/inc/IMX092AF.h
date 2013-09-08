#ifndef _IMX092AF_H
#define _IMX092AF_H

#include <linux/ioctl.h>
//#include "kd_imgsensor.h"

#define IMX092AF_MAGIC 'A'
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
} stIMX092AF_MotorInfo;

//Control commnad
//S means "set through a ptr"
//T means "tell by a arg value"
//G means "get by a ptr"             
//Q means "get by return a value"
//X means "switch G and S atomically"
//H means "switch T and Q atomically"
#define IMX092AFIOC_G_MOTORINFO _IOR(IMX092AF_MAGIC,0,stIMX092AF_MotorInfo)

#define IMX092AFIOC_T_MOVETO _IOW(IMX092AF_MAGIC,1,unsigned long)

#define IMX092AFIOC_T_SETINFPOS _IOW(IMX092AF_MAGIC,2,unsigned long)

#define IMX092AFIOC_T_SETMACROPOS _IOW(IMX092AF_MAGIC,3,unsigned long)

#else
#endif
