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
#include "OV8831AF.h"
#include "../camera/kd_camera_hw.h"

#define LENS_I2C_BUSNUM 1
static struct i2c_board_info __initdata kd_lens_dev={ I2C_BOARD_INFO("OV8831AF", 0x1B)};


#define OV8831AF_DRVNAME "OV8831AF"
#define OV8831AF_VCM_WRITE_ID           0x18

#define OV8831AF_DEBUG
#ifdef OV8831AF_DEBUG
#define OV8831AFDB printk
#else
#define OV8831AFDB(x,...)
#endif

static spinlock_t g_OV8831AF_SpinLock;

static struct i2c_client * g_pstOV8831AF_I2Cclient = NULL;

static dev_t g_OV8831AF_devno;
static struct cdev * g_pOV8831AF_CharDrv = NULL;
static struct class *actuator_class = NULL;

static int  g_s4OV8831AF_Opened = 0;
static long g_i4MotorStatus = 0;
static long g_i4Dir = 0;
static unsigned long g_u4OV8831AF_INF = 0;
static unsigned long g_u4OV8831AF_MACRO = 1023;
static unsigned long g_u4TargetPosition = 0;
static unsigned long g_u4CurrPosition   = 0;

static int g_sr = 3;

extern s32 mt_set_gpio_mode(u32 u4Pin, u32 u4Mode);
extern s32 mt_set_gpio_out(u32 u4Pin, u32 u4PinOut);
extern s32 mt_set_gpio_dir(u32 u4Pin, u32 u4Dir);


static int s4OV8831AF_ReadReg(unsigned short * a_pu2Result)
{
    int  i4RetValue = 0;
    char pBuff[2];

    i4RetValue = i2c_master_recv(g_pstOV8831AF_I2Cclient, pBuff , 2);

    if (i4RetValue < 0) 
    {
        OV8831AFDB("[OV8831AF] I2C read failed!! \n");
        return -1;
    }

    //*a_pu2Result = (((u16)pBuff[0]) << 4) + (pBuff[1] >> 4);
    *a_pu2Result = (((u16)(pBuff[0] & 0x03)) << 8) + pBuff[1];
    return 0;
}

static int s4OV8831AF_WriteReg(u16 a_u2Data)
{
    int  i4RetValue = 0;

    //char puSendCmd[2] = {(char)(a_u2Data >> 4) , (char)(((a_u2Data & 0xF) << 4)+g_sr)};
    char puSendCmd[2] = {(char)(((a_u2Data >> 8) & 0x03) | 0xc0), (char)(a_u2Data & 0xff)};


    //OV8831AFDB("[OV8831AF] g_sr %d, write %d \n", g_sr, a_u2Data);
    g_pstOV8831AF_I2Cclient->ext_flag |= I2C_A_FILTER_MSG;
    i4RetValue = i2c_master_send(g_pstOV8831AF_I2Cclient, puSendCmd, 2);
	
    if (i4RetValue < 0) 
    {
        OV8831AFDB("[OV8831AF] I2C send failed!! \n");
        return -1;
    }

    return 0;
}

inline static int getOV8831AFInfo(__user stOV8831AF_MotorInfo * pstMotorInfo)
{
    stOV8831AF_MotorInfo stMotorInfo;
    stMotorInfo.u4MacroPosition   = g_u4OV8831AF_MACRO;
    stMotorInfo.u4InfPosition     = g_u4OV8831AF_INF;
    stMotorInfo.u4CurrentPosition = g_u4CurrPosition;
    stMotorInfo.bIsSupportSR      = TRUE;

	if (g_i4MotorStatus == 1)	{stMotorInfo.bIsMotorMoving = 1;}
	else						{stMotorInfo.bIsMotorMoving = 0;}

	if (g_s4OV8831AF_Opened >= 1)	{stMotorInfo.bIsMotorOpen = 1;}
	else						{stMotorInfo.bIsMotorOpen = 0;}

    if(copy_to_user(pstMotorInfo , &stMotorInfo , sizeof(stOV8831AF_MotorInfo)))
    {
        OV8831AFDB("[OV8831AF] copy to user failed when getting motor information \n");
    }

    return 0;
}

inline static int moveOV8831AF(unsigned long a_u4Position)
{
    int ret = 0;
    
    if((a_u4Position > g_u4OV8831AF_MACRO) || (a_u4Position < g_u4OV8831AF_INF))
    {
        OV8831AFDB("[OV8831AF] out of range \n");
        return -EINVAL;
    }

    if (g_s4OV8831AF_Opened == 1)
    {
        unsigned short InitPos;
        ret = s4OV8831AF_ReadReg(&InitPos);
	    
        spin_lock(&g_OV8831AF_SpinLock);
        if(ret == 0)
        {
            OV8831AFDB("[OV8831AF] Init Pos %6d \n", InitPos);
            g_u4CurrPosition = (unsigned long)InitPos;
        }
        else
        {		
            g_u4CurrPosition = 0;
        }
        g_s4OV8831AF_Opened = 2;
        spin_unlock(&g_OV8831AF_SpinLock);
    }

    if (g_u4CurrPosition < a_u4Position)
    {
        spin_lock(&g_OV8831AF_SpinLock);	
        g_i4Dir = 1;
        spin_unlock(&g_OV8831AF_SpinLock);	
    }
    else if (g_u4CurrPosition > a_u4Position)
    {
        spin_lock(&g_OV8831AF_SpinLock);	
        g_i4Dir = -1;
        spin_unlock(&g_OV8831AF_SpinLock);			
    }
    else										{return 0;}

    spin_lock(&g_OV8831AF_SpinLock);    
    g_u4TargetPosition = a_u4Position;
    spin_unlock(&g_OV8831AF_SpinLock);	

    //OV8831AFDB("[OV8831AF] move [curr] %d [target] %d\n", g_u4CurrPosition, g_u4TargetPosition);

            spin_lock(&g_OV8831AF_SpinLock);
            g_sr = 3;
            g_i4MotorStatus = 0;
            spin_unlock(&g_OV8831AF_SpinLock);	
		
            if(s4OV8831AF_WriteReg((unsigned short)g_u4TargetPosition) == 0)
            {
                spin_lock(&g_OV8831AF_SpinLock);		
                g_u4CurrPosition = (unsigned long)g_u4TargetPosition;
                spin_unlock(&g_OV8831AF_SpinLock);				
            }
            else
            {
                OV8831AFDB("[OV8831AF] set I2C failed when moving the motor \n");			
                spin_lock(&g_OV8831AF_SpinLock);
                g_i4MotorStatus = -1;
                spin_unlock(&g_OV8831AF_SpinLock);				
            }

    return 0;
}

inline static int setOV8831AFInf(unsigned long a_u4Position)
{
    spin_lock(&g_OV8831AF_SpinLock);
    g_u4OV8831AF_INF = a_u4Position;
    spin_unlock(&g_OV8831AF_SpinLock);	
    return 0;
}

inline static int setOV8831AFMacro(unsigned long a_u4Position)
{
    spin_lock(&g_OV8831AF_SpinLock);
    g_u4OV8831AF_MACRO = a_u4Position;
    spin_unlock(&g_OV8831AF_SpinLock);	
    return 0;	
}

////////////////////////////////////////////////////////////////
static long OV8831AF_Ioctl(
struct file * a_pstFile,
unsigned int a_u4Command,
unsigned long a_u4Param)
{
    long i4RetValue = 0;

    switch(a_u4Command)
    {
        case OV8831AFIOC_G_MOTORINFO :
            i4RetValue = getOV8831AFInfo((__user stOV8831AF_MotorInfo *)(a_u4Param));
        break;

        case OV8831AFIOC_T_MOVETO :
            i4RetValue = moveOV8831AF(a_u4Param);
        break;
 
        case OV8831AFIOC_T_SETINFPOS :
            i4RetValue = setOV8831AFInf(a_u4Param);
        break;

        case OV8831AFIOC_T_SETMACROPOS :
            i4RetValue = setOV8831AFMacro(a_u4Param);
        break;
		
        default :
      	    OV8831AFDB("[OV8831AF] No CMD \n");
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
static int OV8831AF_Open(struct inode * a_pstInode, struct file * a_pstFile)
{
    OV8831AFDB("[OV8831AF] OV8831AF_Open - Start\n");

    spin_lock(&g_OV8831AF_SpinLock);

    if(g_s4OV8831AF_Opened)
    {
        spin_unlock(&g_OV8831AF_SpinLock);
        OV8831AFDB("[OV8831AF] the device is opened \n");
        return -EBUSY;
    }

    g_s4OV8831AF_Opened = 1;
		
    spin_unlock(&g_OV8831AF_SpinLock);

    OV8831AFDB("[OV8831AF] OV8831AF_Open - End\n");

    return 0;
}

//Main jobs:
// 1.Deallocate anything that "open" allocated in private_data.
// 2.Shut down the device on last close.
// 3.Only called once on last time.
// Q1 : Try release multiple times.
static int OV8831AF_Release(struct inode * a_pstInode, struct file * a_pstFile)
{
    OV8831AFDB("[OV8831AF] OV8831AF_Release - Start\n");

    if (g_s4OV8831AF_Opened)
    {
        OV8831AFDB("[OV8831AF] feee \n");
        g_sr = 5;
	    s4OV8831AF_WriteReg(200);
        msleep(10);
	    s4OV8831AF_WriteReg(100);
        msleep(10);
            	            	    	    
        spin_lock(&g_OV8831AF_SpinLock);
        g_s4OV8831AF_Opened = 0;
        spin_unlock(&g_OV8831AF_SpinLock);

    }

    OV8831AFDB("[OV8831AF] OV8831AF_Release - End\n");

    return 0;
}

static const struct file_operations g_stOV8831AF_fops = 
{
    .owner = THIS_MODULE,
    .open = OV8831AF_Open,
    .release = OV8831AF_Release,
    .unlocked_ioctl = OV8831AF_Ioctl
};

inline static int Register_OV8831AF_CharDrv(void)
{
    struct device* vcm_device = NULL;

    OV8831AFDB("[OV8831AF] Register_OV8831AF_CharDrv - Start\n");

    //Allocate char driver no.
    if( alloc_chrdev_region(&g_OV8831AF_devno, 0, 1,OV8831AF_DRVNAME) )
    {
        OV8831AFDB("[OV8831AF] Allocate device no failed\n");

        return -EAGAIN;
    }

    //Allocate driver
    g_pOV8831AF_CharDrv = cdev_alloc();

    if(NULL == g_pOV8831AF_CharDrv)
    {
        unregister_chrdev_region(g_OV8831AF_devno, 1);

        OV8831AFDB("[OV8831AF] Allocate mem for kobject failed\n");

        return -ENOMEM;
    }

    //Attatch file operation.
    cdev_init(g_pOV8831AF_CharDrv, &g_stOV8831AF_fops);

    g_pOV8831AF_CharDrv->owner = THIS_MODULE;

    //Add to system
    if(cdev_add(g_pOV8831AF_CharDrv, g_OV8831AF_devno, 1))
    {
        OV8831AFDB("[OV8831AF] Attatch file operation failed\n");

        unregister_chrdev_region(g_OV8831AF_devno, 1);

        return -EAGAIN;
    }

    actuator_class = class_create(THIS_MODULE, "actuatordrv4");
    if (IS_ERR(actuator_class)) {
        int ret = PTR_ERR(actuator_class);
        OV8831AFDB("Unable to create class, err = %d\n", ret);
        return ret;            
    }

    vcm_device = device_create(actuator_class, NULL, g_OV8831AF_devno, NULL, OV8831AF_DRVNAME);

    if(NULL == vcm_device)
    {
        return -EIO;
    }
    
    OV8831AFDB("[OV8831AF] Register_OV8831AF_CharDrv - End\n");    
    return 0;
}

inline static void Unregister_OV8831AF_CharDrv(void)
{
    OV8831AFDB("[OV8831AF] Unregister_OV8831AF_CharDrv - Start\n");

    //Release char driver
    cdev_del(g_pOV8831AF_CharDrv);

    unregister_chrdev_region(g_OV8831AF_devno, 1);
    
    device_destroy(actuator_class, g_OV8831AF_devno);

    class_destroy(actuator_class);

    OV8831AFDB("[OV8831AF] Unregister_OV8831AF_CharDrv - End\n");    
}

//////////////////////////////////////////////////////////////////////

static int OV8831AF_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int OV8831AF_i2c_remove(struct i2c_client *client);
static const struct i2c_device_id OV8831AF_i2c_id[] = {{OV8831AF_DRVNAME,0},{}};   
struct i2c_driver OV8831AF_i2c_driver = {                       
    .probe = OV8831AF_i2c_probe,                                   
    .remove = OV8831AF_i2c_remove,                           
    .driver.name = OV8831AF_DRVNAME,                 
    .id_table = OV8831AF_i2c_id,                             
};  

#if 0 
static int OV8831AF_i2c_detect(struct i2c_client *client, int kind, struct i2c_board_info *info) {         
    strcpy(info->type, OV8831AF_DRVNAME);                                                         
    return 0;                                                                                       
}      
#endif 
static int OV8831AF_i2c_remove(struct i2c_client *client) {
    return 0;
}

/* Kirby: add new-style driver {*/
static int OV8831AF_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    int i4RetValue = 0;

    OV8831AFDB("[OV8831AF] OV8831AF_i2c_probe\n");

    /* Kirby: add new-style driver { */
    g_pstOV8831AF_I2Cclient = client;
    
    //g_pstOV8831AF_I2Cclient->addr = g_pstOV8831AF_I2Cclient->addr >> 1;
    g_pstOV8831AF_I2Cclient->addr = OV8831AF_VCM_WRITE_ID >> 1;
    
    //Register char driver
    i4RetValue = Register_OV8831AF_CharDrv();

    if(i4RetValue){

        OV8831AFDB("[OV8831AF] register char device failed!\n");

        return i4RetValue;
    }

    spin_lock_init(&g_OV8831AF_SpinLock);

    OV8831AFDB("[OV8831AF] Attached!! \n");

    return 0;
}

static int OV8831AF_probe(struct platform_device *pdev)
{
    return i2c_add_driver(&OV8831AF_i2c_driver);
}

static int OV8831AF_remove(struct platform_device *pdev)
{
    i2c_del_driver(&OV8831AF_i2c_driver);
    return 0;
}

static int OV8831AF_suspend(struct platform_device *pdev, pm_message_t mesg)
{
    return 0;
}

static int OV8831AF_resume(struct platform_device *pdev)
{
    return 0;
}

// platform structure
static struct platform_driver g_stOV8831AF_Driver = {
    .probe		= OV8831AF_probe,
    .remove	= OV8831AF_remove,
    .suspend	= OV8831AF_suspend,
    .resume	= OV8831AF_resume,
    .driver		= {
        .name	= "lens_actuator4",
        .owner	= THIS_MODULE,
    }
};

static struct platform_device actuator_dev4 = {
	.name		  = "lens_actuator4",
	.id		  = -1,
};

static int __init OV8831AF_i2C_init(void)
{
    i2c_register_board_info(LENS_I2C_BUSNUM, &kd_lens_dev, 1);
    platform_device_register(&actuator_dev4);
    if(platform_driver_register(&g_stOV8831AF_Driver)){
        OV8831AFDB("failed to register OV8831AF driver\n");
        return -ENODEV;
    }

    return 0;
}

static void __exit OV8831AF_i2C_exit(void)
{
	platform_driver_unregister(&g_stOV8831AF_Driver);
}

module_init(OV8831AF_i2C_init);
module_exit(OV8831AF_i2C_exit);

MODULE_DESCRIPTION("OV8831AF lens module driver");
MODULE_AUTHOR("KY Chen <ky.chen@Mediatek.com>");
MODULE_LICENSE("GPL");



