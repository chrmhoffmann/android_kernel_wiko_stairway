/*
 * MD218A voice coil motor driver
 *
 *
 */

#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <asm/atomic.h>
#include "IMX092AF.h"
#include "../camera/kd_camera_hw.h"

#define LENS_I2C_BUSNUM 1
static struct i2c_board_info __initdata kd_lens_dev={ I2C_BOARD_INFO("IMX092AF", 0x1D)};


#define IMX092AF_DRVNAME "IMX092AF"
#define IMX092AF_VCM_WRITE_ID           0x18

#define IMX092AF_DEBUG
#ifdef IMX092AF_DEBUG
#define IMX092AFDB printk
#else
#define IMX092AFDB(x,...)
#endif

static spinlock_t g_IMX092AF_SpinLock;

static struct i2c_client * g_pstIMX092AF_I2Cclient = NULL;

static dev_t g_IMX092AF_devno;
static struct cdev * g_pIMX092AF_CharDrv = NULL;
static struct class *actuator_class = NULL;

static int  g_s4IMX092AF_Opened = 0;
static long g_i4MotorStatus = 0;
static long g_i4Dir = 0;
static unsigned long g_u4IMX092AF_INF = 0;
static unsigned long g_u4IMX092AF_MACRO = 1023;
static unsigned long g_u4TargetPosition = 0;
static unsigned long g_u4CurrPosition   = 0;

static int g_sr = 3;

extern s32 mt_set_gpio_mode(u32 u4Pin, u32 u4Mode);
extern s32 mt_set_gpio_out(u32 u4Pin, u32 u4PinOut);
extern s32 mt_set_gpio_dir(u32 u4Pin, u32 u4Dir);


static int s4IMX092AF_ReadReg(unsigned short * a_pu2Result)
{
    int  i4RetValue = 0;
    char pBuff[2];

    i4RetValue = i2c_master_recv(g_pstIMX092AF_I2Cclient, pBuff , 2);

    if (i4RetValue < 0) 
    {
        IMX092AFDB("[IMX092AF] I2C read failed!! \n");
        return -1;
    }

    //*a_pu2Result = (((u16)pBuff[0]) << 4) + (pBuff[1] >> 4);
    *a_pu2Result = (((u16)(pBuff[0] & 0x03)) << 8) + pBuff[1];
    return 0;
}

static int s4IMX092AF_WriteReg(u16 a_u2Data)
{
    int  i4RetValue = 0;

    //char puSendCmd[2] = {(char)(a_u2Data >> 4) , (char)(((a_u2Data & 0xF) << 4)+g_sr)};
    char puSendCmd[2] = {(char)(((a_u2Data >> 8) & 0x03) | 0xc0), (char)(a_u2Data & 0xff)};


    //IMX092AFDB("[IMX092AF] g_sr %d, write %d \n", g_sr, a_u2Data);
    g_pstIMX092AF_I2Cclient->ext_flag |= I2C_A_FILTER_MSG;
    i4RetValue = i2c_master_send(g_pstIMX092AF_I2Cclient, puSendCmd, 2);
	
    if (i4RetValue < 0) 
    {
        IMX092AFDB("[IMX092AF] I2C send failed!! \n");
        return -1;
    }

    return 0;
}

inline static int getIMX092AFInfo(__user stIMX092AF_MotorInfo * pstMotorInfo)
{
    stIMX092AF_MotorInfo stMotorInfo;
    stMotorInfo.u4MacroPosition   = g_u4IMX092AF_MACRO;
    stMotorInfo.u4InfPosition     = g_u4IMX092AF_INF;
    stMotorInfo.u4CurrentPosition = g_u4CurrPosition;
    stMotorInfo.bIsSupportSR      = TRUE;

	if (g_i4MotorStatus == 1)	{stMotorInfo.bIsMotorMoving = 1;}
	else						{stMotorInfo.bIsMotorMoving = 0;}

	if (g_s4IMX092AF_Opened >= 1)	{stMotorInfo.bIsMotorOpen = 1;}
	else						{stMotorInfo.bIsMotorOpen = 0;}

    if(copy_to_user(pstMotorInfo , &stMotorInfo , sizeof(stIMX092AF_MotorInfo)))
    {
        IMX092AFDB("[IMX092AF] copy to user failed when getting motor information \n");
    }

    return 0;
}

inline static int moveIMX092AF(unsigned long a_u4Position)
{
    int ret = 0;
    
    if((a_u4Position > g_u4IMX092AF_MACRO) || (a_u4Position < g_u4IMX092AF_INF))
    {
        IMX092AFDB("[IMX092AF] out of range \n");
        return -EINVAL;
    }

    if (g_s4IMX092AF_Opened == 1)
    {
        unsigned short InitPos;
        ret = s4IMX092AF_ReadReg(&InitPos);
	    
        spin_lock(&g_IMX092AF_SpinLock);
        if(ret == 0)
        {
            IMX092AFDB("[IMX092AF] Init Pos %6d \n", InitPos);
            g_u4CurrPosition = (unsigned long)InitPos;
        }
        else
        {		
            g_u4CurrPosition = 0;
        }
        g_s4IMX092AF_Opened = 2;
        spin_unlock(&g_IMX092AF_SpinLock);
    }

    if (g_u4CurrPosition < a_u4Position)
    {
        spin_lock(&g_IMX092AF_SpinLock);	
        g_i4Dir = 1;
        spin_unlock(&g_IMX092AF_SpinLock);	
    }
    else if (g_u4CurrPosition > a_u4Position)
    {
        spin_lock(&g_IMX092AF_SpinLock);	
        g_i4Dir = -1;
        spin_unlock(&g_IMX092AF_SpinLock);			
    }
    else										{return 0;}

    spin_lock(&g_IMX092AF_SpinLock);    
    g_u4TargetPosition = a_u4Position;
    spin_unlock(&g_IMX092AF_SpinLock);	

    //IMX092AFDB("[IMX092AF] move [curr] %d [target] %d\n", g_u4CurrPosition, g_u4TargetPosition);

            spin_lock(&g_IMX092AF_SpinLock);
            g_sr = 3;
            g_i4MotorStatus = 0;
            spin_unlock(&g_IMX092AF_SpinLock);	
		
            if(s4IMX092AF_WriteReg((unsigned short)g_u4TargetPosition) == 0)
            {
                spin_lock(&g_IMX092AF_SpinLock);		
                g_u4CurrPosition = (unsigned long)g_u4TargetPosition;
                spin_unlock(&g_IMX092AF_SpinLock);				
            }
            else
            {
                IMX092AFDB("[IMX092AF] set I2C failed when moving the motor \n");			
                spin_lock(&g_IMX092AF_SpinLock);
                g_i4MotorStatus = -1;
                spin_unlock(&g_IMX092AF_SpinLock);				
            }

    return 0;
}

inline static int setIMX092AFInf(unsigned long a_u4Position)
{
    spin_lock(&g_IMX092AF_SpinLock);
    g_u4IMX092AF_INF = a_u4Position;
    spin_unlock(&g_IMX092AF_SpinLock);	
    return 0;
}

inline static int setIMX092AFMacro(unsigned long a_u4Position)
{
    spin_lock(&g_IMX092AF_SpinLock);
    g_u4IMX092AF_MACRO = a_u4Position;
    spin_unlock(&g_IMX092AF_SpinLock);	
    return 0;	
}

////////////////////////////////////////////////////////////////
static long IMX092AF_Ioctl(
struct file * a_pstFile,
unsigned int a_u4Command,
unsigned long a_u4Param)
{
    long i4RetValue = 0;

    switch(a_u4Command)
    {
        case IMX092AFIOC_G_MOTORINFO :
            i4RetValue = getIMX092AFInfo((__user stIMX092AF_MotorInfo *)(a_u4Param));
        break;

        case IMX092AFIOC_T_MOVETO :
            i4RetValue = moveIMX092AF(a_u4Param);
        break;
 
        case IMX092AFIOC_T_SETINFPOS :
            i4RetValue = setIMX092AFInf(a_u4Param);
        break;

        case IMX092AFIOC_T_SETMACROPOS :
            i4RetValue = setIMX092AFMacro(a_u4Param);
        break;
		
        default :
      	    IMX092AFDB("[IMX092AF] No CMD \n");
            i4RetValue = -EPERM;
        break;
    }

    return i4RetValue;
}

//Main jobs:
// 1.check for device-specified errors, device not ready.
// 2.Initialize the device if it is opened for the first time.
// 3.Update f_op pointer.
// 4.Fill data structures into private_data
//CAM_RESET
static int IMX092AF_Open(struct inode * a_pstInode, struct file * a_pstFile)
{
    IMX092AFDB("[IMX092AF] IMX092AF_Open - Start\n");

    spin_lock(&g_IMX092AF_SpinLock);

    if(g_s4IMX092AF_Opened)
    {
        spin_unlock(&g_IMX092AF_SpinLock);
        IMX092AFDB("[IMX092AF] the device is opened \n");
        return -EBUSY;
    }

    g_s4IMX092AF_Opened = 1;
		
    spin_unlock(&g_IMX092AF_SpinLock);

    IMX092AFDB("[IMX092AF] IMX092AF_Open - End\n");

    return 0;
}

//Main jobs:
// 1.Deallocate anything that "open" allocated in private_data.
// 2.Shut down the device on last close.
// 3.Only called once on last time.
// Q1 : Try release multiple times.
static int IMX092AF_Release(struct inode * a_pstInode, struct file * a_pstFile)
{
    IMX092AFDB("[IMX092AF] IMX092AF_Release - Start\n");

    if (g_s4IMX092AF_Opened)
    {
        IMX092AFDB("[IMX092AF] feee \n");
        g_sr = 5;
	    s4IMX092AF_WriteReg(200);
        msleep(10);
	    s4IMX092AF_WriteReg(100);
        msleep(10);
            	            	    	    
        spin_lock(&g_IMX092AF_SpinLock);
        g_s4IMX092AF_Opened = 0;
        spin_unlock(&g_IMX092AF_SpinLock);

    }

    IMX092AFDB("[IMX092AF] IMX092AF_Release - End\n");

    return 0;
}

static const struct file_operations g_stIMX092AF_fops = 
{
    .owner = THIS_MODULE,
    .open = IMX092AF_Open,
    .release = IMX092AF_Release,
    .unlocked_ioctl = IMX092AF_Ioctl
};

inline static int Register_IMX092AF_CharDrv(void)
{
    struct device* vcm_device = NULL;

    IMX092AFDB("[IMX092AF] Register_IMX092AF_CharDrv - Start\n");

    //Allocate char driver no.
    if( alloc_chrdev_region(&g_IMX092AF_devno, 0, 1,IMX092AF_DRVNAME) )
    {
        IMX092AFDB("[IMX092AF] Allocate device no failed\n");

        return -EAGAIN;
    }

    //Allocate driver
    g_pIMX092AF_CharDrv = cdev_alloc();

    if(NULL == g_pIMX092AF_CharDrv)
    {
        unregister_chrdev_region(g_IMX092AF_devno, 1);

        IMX092AFDB("[IMX092AF] Allocate mem for kobject failed\n");

        return -ENOMEM;
    }

    //Attatch file operation.
    cdev_init(g_pIMX092AF_CharDrv, &g_stIMX092AF_fops);

    g_pIMX092AF_CharDrv->owner = THIS_MODULE;

    //Add to system
    if(cdev_add(g_pIMX092AF_CharDrv, g_IMX092AF_devno, 1))
    {
        IMX092AFDB("[IMX092AF] Attatch file operation failed\n");

        unregister_chrdev_region(g_IMX092AF_devno, 1);

        return -EAGAIN;
    }

    actuator_class = class_create(THIS_MODULE, "actuatordrv6");
    if (IS_ERR(actuator_class)) {
        int ret = PTR_ERR(actuator_class);
        IMX092AFDB("Unable to create class, err = %d\n", ret);
        return ret;            
    }

    vcm_device = device_create(actuator_class, NULL, g_IMX092AF_devno, NULL, IMX092AF_DRVNAME);

    if(NULL == vcm_device)
    {
        return -EIO;
    }
    
    IMX092AFDB("[IMX092AF] Register_IMX092AF_CharDrv - End\n");    
    return 0;
}

inline static void Unregister_IMX092AF_CharDrv(void)
{
    IMX092AFDB("[IMX092AF] Unregister_IMX092AF_CharDrv - Start\n");

    //Release char driver
    cdev_del(g_pIMX092AF_CharDrv);

    unregister_chrdev_region(g_IMX092AF_devno, 1);
    
    device_destroy(actuator_class, g_IMX092AF_devno);

    class_destroy(actuator_class);

    IMX092AFDB("[IMX092AF] Unregister_IMX092AF_CharDrv - End\n");    
}

//////////////////////////////////////////////////////////////////////

static int IMX092AF_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int IMX092AF_i2c_remove(struct i2c_client *client);
static const struct i2c_device_id IMX092AF_i2c_id[] = {{IMX092AF_DRVNAME,0},{}};   
struct i2c_driver IMX092AF_i2c_driver = {                       
    .probe = IMX092AF_i2c_probe,                                   
    .remove = IMX092AF_i2c_remove,                           
    .driver.name = IMX092AF_DRVNAME,                 
    .id_table = IMX092AF_i2c_id,                             
};  

#if 0 
static int IMX092AF_i2c_detect(struct i2c_client *client, int kind, struct i2c_board_info *info) {         
    strcpy(info->type, IMX092AF_DRVNAME);                                                         
    return 0;                                                                                       
}      
#endif 
static int IMX092AF_i2c_remove(struct i2c_client *client) {
    return 0;
}

/* Kirby: add new-style driver {*/
static int IMX092AF_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    int i4RetValue = 0;

    IMX092AFDB("[IMX092AF] IMX092AF_i2c_probe\n");

    /* Kirby: add new-style driver { */
    g_pstIMX092AF_I2Cclient = client;
    
    //g_pstIMX092AF_I2Cclient->addr = g_pstIMX092AF_I2Cclient->addr >> 1;
    g_pstIMX092AF_I2Cclient->addr = IMX092AF_VCM_WRITE_ID >> 1;
    
    //Register char driver
    i4RetValue = Register_IMX092AF_CharDrv();

    if(i4RetValue){

        IMX092AFDB("[IMX092AF] register char device failed!\n");

        return i4RetValue;
    }

    spin_lock_init(&g_IMX092AF_SpinLock);

    IMX092AFDB("[IMX092AF] Attached!! \n");

    return 0;
}

static int IMX092AF_probe(struct platform_device *pdev)
{
    return i2c_add_driver(&IMX092AF_i2c_driver);
}

static int IMX092AF_remove(struct platform_device *pdev)
{
    i2c_del_driver(&IMX092AF_i2c_driver);
    return 0;
}

static int IMX092AF_suspend(struct platform_device *pdev, pm_message_t mesg)
{
    return 0;
}

static int IMX092AF_resume(struct platform_device *pdev)
{
    return 0;
}

// platform structure
static struct platform_driver g_stIMX092AF_Driver = {
    .probe		= IMX092AF_probe,
    .remove	= IMX092AF_remove,
    .suspend	= IMX092AF_suspend,
    .resume	= IMX092AF_resume,
    .driver		= {
        .name	= "lens_actuator6",
        .owner	= THIS_MODULE,
    }
};

static struct platform_device actuator_dev6 = {
	.name		  = "lens_actuator6",
	.id		  = -1,
};

static int __init IMX092AF_i2C_init(void)
{
    i2c_register_board_info(LENS_I2C_BUSNUM, &kd_lens_dev, 1);
    platform_device_register(&actuator_dev6);	
    if(platform_driver_register(&g_stIMX092AF_Driver)){
        IMX092AFDB("failed to register IMX092AF driver\n");
        return -ENODEV;
    }

    return 0;
}

static void __exit IMX092AF_i2C_exit(void)
{
	platform_driver_unregister(&g_stIMX092AF_Driver);
}

module_init(IMX092AF_i2C_init);
module_exit(IMX092AF_i2C_exit);

MODULE_DESCRIPTION("IMX092AF lens module driver");
MODULE_AUTHOR("KY Chen <ky.chen@Mediatek.com>");
MODULE_LICENSE("GPL");



