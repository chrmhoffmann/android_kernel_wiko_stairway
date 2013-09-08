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

#include <linux/i2c.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/wait.h>
#include <linux/delay.h>
#include <linux/miscdevice.h>

#include <linux/interrupt.h>
#include <linux/time.h>

#include "tpd.h"

#include <cust_eint.h>
#include <linux/rtpm_prio.h>

#include <linux/wakelock.h>

#include <asm/uaccess.h>

#include <linux/dma-mapping.h>
#include <linux/vmalloc.h>

#include "cust_gpio_usage.h"

#include "tpd_custom_ft53x6_5p.h"

#include <mach/mt_pm_ldo.h>
#include <mach/mt_typedefs.h>
#include <mach/mt_boot.h>


#define TPD_SLAVE_ADDR 0x72	//ES980=0x72  //ES970=0x7E	//default = 0x70

#define CONFIG_TOUCHSCREEN_FT5X05_SUPPORT_ISP

static int isUpgrade = 0; // 1:show that TP is upgrading ...edit by Magnum  2012-7-19 

#define FTS_PROTOCOL_LEN (sizeof(fts_report_data_t))
#define TINNO_TOUCH_TRACK_IDS (5)  //edit by Magnum 2012-8-6
#define FTS_INVALID_DATA (-1)

#define FTS_EF_DOWN (0)
#define FTS_EF_UP (1)
#define FTS_EF_CONTACT (2)
#define FTS_EF_RESERVED (3)


#define TPIO_RESET      GPIO_CTP_RST_PIN     //GPIO184
#define TPIO_WAKEUP     GPIO_CTP_EN_PIN      //GPIO187
#define TPIO_EINT		GPIO_CTP_EINT_PIN

static const int TPD_KEYSFACTORY[TPD_KEY_COUNT] =  {KEY_F1, KEY_F2, KEY_F3};

#ifdef TPD_HAVE_BUTTON 
static int tpd_keys_local[TPD_KEY_COUNT] = TPD_KEYS;
static int tpd_keys_dim_local[TPD_KEY_COUNT][4] = TPD_KEYS_DIM;
extern void tpd_button(unsigned int x, unsigned int y, unsigned int down);
#endif

#if (defined(TPD_WARP_START) && defined(TPD_WARP_END))
static int tpd_wb_start_local[TPD_WARP_CNT] = TPD_WARP_START;
static int tpd_wb_end_local[TPD_WARP_CNT]   = TPD_WARP_END;
#endif
#if (defined(TPD_HAVE_CALIBRATION) && !defined(TPD_CUSTOM_CALIBRATION))
static int tpd_calmat_local[8]     = TPD_CALIBRATION_MATRIX;
static int tpd_def_calmat_local[8] = TPD_CALIBRATION_MATRIX;
#endif

typedef struct _tinno_ts_point{
	int x, y, z, id;
} tinno_ts_point;

struct touch_info {
    tinno_ts_point pt[TINNO_TOUCH_TRACK_IDS];
    int p, count, pending;
};

struct touch_elapse {
    int t1, t2, i;
    int buf[5];
};

extern struct tpd_device *tpd;
extern int tpd_firmware_version[2];
extern BOOL bPMIC_Init_finish;

static void tpd_eint_interrupt_handler(void);
static int tpd_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int tpd_i2c_detect(struct i2c_client *client, int kind, struct i2c_board_info *info);
static int tpd_i2c_remove(struct i2c_client *client);
static int __devexit tpd_remove(struct i2c_client *client);

// TODO: should be moved into mach/xxx.h 
#ifdef MT6575
extern void mt65xx_eint_unmask(unsigned int line);
extern void mt65xx_eint_mask(unsigned int line);
extern void mt65xx_eint_set_hw_debounce(kal_uint8 eintno, kal_uint32 ms);
extern kal_uint32 mt65xx_eint_set_sens(kal_uint8 eintno, kal_bool sens);
extern void mt65xx_eint_registration(kal_uint8 eintno, kal_bool Dbounce_En,
                                     kal_bool ACT_Polarity, void (EINT_FUNC_PTR)(void),
                                     kal_bool auto_umask);
#endif

//#ifdef MT6577
	extern void mt65xx_eint_unmask(unsigned int line);
	extern void mt65xx_eint_mask(unsigned int line);
	extern void mt65xx_eint_set_hw_debounce(unsigned int eint_num, unsigned int ms);
	extern unsigned int mt65xx_eint_set_sens(unsigned int eint_num, unsigned int sens);
	extern void mt65xx_eint_registration(unsigned int eint_num, unsigned int is_deb_en, unsigned int pol, void (EINT_FUNC_PTR)(void), unsigned int is_auto_umask);
//#endif

static struct i2c_client *i2c_client = NULL;
static struct task_struct *thread = NULL;
static DECLARE_WAIT_QUEUE_HEAD(waiter);
static int tpd_flag=0;
extern int tp_boot_mode;

//edit by Magnum 2012-8-31 ctp up Questions
static int x_history[TINNO_TOUCH_TRACK_IDS+1];
static int y_history[TINNO_TOUCH_TRACK_IDS+1];
static int touch_event_handler(void *unused);

//static int raw_x1, raw_y1, raw_x2, raw_y2;
static int tpd_status = 0;
static int tpd_dcount = 0;

static const struct i2c_device_id ft5316_i2c_id[] = {{"ft5316",0},{}};
static struct i2c_board_info __initdata ft5206_i2c_tpd={ I2C_BOARD_INFO("ft5316", (TPD_SLAVE_ADDR>>1))};
//unsigned short force[] = {1, TPD_SLAVE_ADDR, I2C_CLIENT_END, I2C_CLIENT_END};
//static const unsigned short * const forces[] = { force, NULL };
//static struct i2c_client_address_data addr_data = { .forces = forces, };

static char panel_version = 0;

//edit by Magnum 2012-7-10
static char vendor_version = 0;
static int inTouchKey = 0;
static int tpd_downMode = 0; // 1:touchmode, 2:virtual key in touch mode, 0 : default and up event

static	int tpd_init_skip = 0;

#ifdef CONFIG_TOUCHSCREEN_FT5X05_SUPPORT_ISP
struct tinno_ts_data {
	struct wake_lock wake_lock;
	atomic_t isp_opened;
	uint8_t *isp_pBuffer;
	struct i2c_client *client;
};
struct tinno_ts_data *g_pts = NULL;

struct mutex tp_mutex;

#define ISP_FLASH_SIZE	0x8000 //32KB
#define FT5X06_FIRMWAIR_VERSION_D

#define FTS_MODE_OPRATE (0x00)
#define FTS_MODE_UPDATE (0x01)
#define FTS_MODE_SYSTEM (0x02)

#define TOUCH_IO_MAGIC ('F')
#define FT5X05_IOCTL_RESET 				_IO(TOUCH_IO_MAGIC, 0x00)
#define FT5X05_IOCTL_SWITCH_TO 		_IOW(TOUCH_IO_MAGIC, 0x01, int)
#define FT5X05_IOCTL_WRITE_PROTECT 	_IOW(TOUCH_IO_MAGIC, 0x02, int)
#define FT5X05_IOCTL_ERASE 				_IO(TOUCH_IO_MAGIC, 0x03)
#define FT5X05_IOCTL_GET_STATUS		_IOR(TOUCH_IO_MAGIC, 0x04, int)
#define FT5X05_IOCTL_GET_CHECKSUM		_IOR(TOUCH_IO_MAGIC, 0x05, int)
#define FT5X05_IOCTL_GET_TPID			_IOR(TOUCH_IO_MAGIC, 0x06, int)
#define FT5X05_IOCTL_GET_VENDORID		_IOR(TOUCH_IO_MAGIC, 0x07, int)
#define FT5X05_IOC_MAXNR				(0x08)

static u8 tpd_down_state=0;
static int down_x=0;
static int down_y=0;
static void fts_isp_register(struct i2c_client *client);

#endif
extern char phonestate;

typedef struct {
	uint8_t	x_h: 4,
		reserved_1: 2,
		event_flag: 2;
	uint8_t	x_l;
	uint8_t	y_h: 4,
		touch_id: 4;
	uint8_t	y_l;
	uint8_t reserved_2;
	uint8_t reserved_3;
} xy_data_t;

typedef struct {
	uint8_t	reserved_1: 4,
		device_mode: 3,
		reserved_2: 1;
	uint8_t	gesture;
	uint8_t	fingers: 4,
		reserved_3: 4;
	xy_data_t	 xy_data[TINNO_TOUCH_TRACK_IDS];
} fts_report_data_t;


struct i2c_driver tpd_i2c_driver = {
    .driver = {
	    .name = "ft5316_5p",//.name = TPD_DEVICE,
    },    
    .probe = tpd_i2c_probe,
    .remove = __devexit_p(tpd_remove),
    .detect = tpd_i2c_detect,
    .id_table = ft5316_i2c_id,
//    .address_data = &addr_data,
};

static int tpd_i2c_detect(struct i2c_client *client, int kind, struct i2c_board_info *info) {
    TPD_DEBUG("tpd_i2c_detect\n");
    strcpy(info->type, "ft5316");
    return 0;
}

static int __devexit tpd_remove(struct i2c_client *client) 
{
   
	 TPD_DEBUG("TPD removed\n");
 
   return 0;
}
 

int tpd_local_init(void) {
    int i;
    
    tp_boot_mode = get_boot_mode();
    // Software reset mode will be treated as normal boot
    if(tp_boot_mode==3) tp_boot_mode = NORMAL_BOOT;
//Ivan
    TPD_DEBUG("tpd_local_init boot mode = %d\n",tp_boot_mode);  

    if(i2c_add_driver(&tpd_i2c_driver)!=0)
        TPD_DEBUG("unable to add i2c driver.\n");
    
    if(tpd_load_status == 0) 
    {
    	TPD_DMESG("ft5206 add error touch panel driver.\n");
    	i2c_del_driver(&tpd_i2c_driver);
    	return -1;
    }

	
#ifdef TPD_HAVE_BUTTON     
    if (FACTORY_BOOT == tp_boot_mode)
    {
	for (i = 0; i < TPD_KEY_COUNT ; i++)
	    tpd_keys_local[i] = TPD_KEYSFACTORY[i];
    }
    
    tpd_button_setting(TPD_KEY_COUNT, tpd_keys_local, tpd_keys_dim_local);// initialize tpd button data
#endif   
  
//    printk("tpd_local_init I am here\n");
    
#if (defined(TPD_WARP_START) && defined(TPD_WARP_END))    
    TPD_DO_WARP = 1;
    memcpy(tpd_wb_start, tpd_wb_start_local, TPD_WARP_CNT*4);
    memcpy(tpd_wb_end, tpd_wb_start_local, TPD_WARP_CNT*4);
#endif 

#if (defined(TPD_HAVE_CALIBRATION) && !defined(TPD_CUSTOM_CALIBRATION))
    memcpy(tpd_calmat, tpd_calmat_local, 8*4);
    memcpy(tpd_def_calmat, tpd_def_calmat_local, 8*4);	
#endif  
    TPD_DMESG("end %s, %d\n", __FUNCTION__, __LINE__);  
    tpd_type_cap = 1;	
    return 0;
}


ssize_t tp_write_m_byte(u8 cmd, u8 *writeData,U16 len)
{
    char    write_data[8] = {0};
    int    i,ret=0;

    if(len == 0) {
        TPD_DEBUG("[Error]TP Write Len should not be zero!! \n");
        return 0;
    }
        
//    write_data[0] = cmd;

    //Driver does not allow (single write length > 8)
    while(len > 8)
    {
        for (i = 0; i<7; i++){
            write_data[i] = *(writeData+i);    
        }
        ret = i2c_master_send(i2c_client, write_data, 8);
        if (ret < 0) {
            TPD_DEBUG("[Error]TP reads data error!! \n");
            return 0;
        }
        writeData+=8;
        len -= 8;
    }
    if (len > 0){
        for (i = 0; i<len; i++){
            write_data[i] = *(writeData+i);    
        }
        ret = i2c_master_send(i2c_client, write_data, len);
        if (ret < 0) {
            TPD_DEBUG("[Error]TP reads data error!! \n");
            return 0;
        }
    }

    return 1;
}


static int tinno_ts_get_fw_version()
{
    char readbyte[2] = {0xA6};
    char fw_version;
    int err;

	// Read FW Version.
//	fw_version = i2c_smbus_read_byte_data(i2c_client, 0xA6);
	readbyte[0] = 0xA6;
	i2c_master_send(i2c_client,&readbyte[0],1);
	err = i2c_master_recv(i2c_client, &fw_version, 1);

	if (fw_version == 0 ||fw_version == 0xa6 || err < 0){
		TPD_DEBUG("i2c_smbus_write_byte_data failed.\n");
	    return 0;
	}
	
	printk("fw_version=0x%X \n", fw_version);
	
	//edit by Magnum 2012-7-10
     char vendor_id;
     readbyte[0] = 0xA8;
     i2c_master_send(i2c_client,&readbyte[0],1);
      err = i2c_master_recv(i2c_client, &vendor_id, 1);
     if (vendor_id == 0 || vendor_id == 0xa8 || err < 0){
        TPD_DEBUG("i2c_smbus_write_byte_data failed.\n");
         return 0;
      }
      vendor_version = vendor_id;
      printk("Magnum vendor_id=0x%X \n", vendor_id);   
	printk("tpd_local_init boot mode = %d \n", tp_boot_mode);    
	
//	return (int)fw_version;
	return 1;
}

static int tinno_ts_set_period_active(void)
{
    char readbyte[2] = {0x88};	
	char rdata;
	int ret = 0;

	ret = i2c_smbus_write_byte_data(i2c_client, 0x88, 6);/*6X10HZ*/
	readbyte[0] = 0x88;
	i2c_master_send(i2c_client,&readbyte[0],1);
	i2c_master_recv(i2c_client, &rdata, 1);

	if (rdata < 0) {
		TPD_DEBUG("get period active failed");
		goto err;
	}
	printk("active period is %d \n", rdata);

	if (ret < 0) {
		TPD_DEBUG("set period active failed");
		goto err;
	}
	return 0;
err:
	return ret;
}

//edit by Magnum 2012-12-17  check ic work state 
static int tinno_check_focaltech(void)
{
	char readbyte[2] = {0xA3};
   	char fw_version;
    	int err;

	// Read FW Version.
//	fw_version = i2c_smbus_read_byte_data(i2c_client, 0xA6);
	readbyte[0] = 0xA3;
	i2c_master_send(i2c_client,&readbyte[0],1);
	err = i2c_master_recv(i2c_client, &fw_version, 1);
	printk("fw_version=0x%X \n", fw_version);
	if (fw_version == 0xa3){
		printk("fw_version=0x%X \n", fw_version);
		return 1;
	}
	TPD_DEBUG("i2c_smbus_write_byte_data failed.\n");
	 return 0;
	
}

static void tpd_reset(void)
{
    mt_set_gpio_out(TPIO_RESET,1);				//output low  
    msleep(3);	
    mt_set_gpio_out(TPIO_RESET,0);				//output low
    msleep(3);
    mt_set_gpio_out(TPIO_RESET,1);				//output low  
    msleep(50);	
}

static void tinno_tp_power_on(void)
{
    int err = 0;
    int i=0;
//Ivan config Wakeup pin ??? assume high active???
	//Power On CTP
    hwPowerOn(MT65XX_POWER_LDO_VGP4, VOL_2800, "TP");     
    msleep(10);	
    
    mt_set_gpio_mode(TPIO_WAKEUP, 0x00);		//GPIO mode        
    mt_set_gpio_dir(TPIO_WAKEUP,1);
    mt_set_gpio_pull_enable(TPIO_WAKEUP, 1);	//external pull up???        
    mt_set_gpio_pull_select(TPIO_WAKEUP, GPIO_PULL_UP);	   
    
    mt_set_gpio_out(TPIO_WAKEUP,1);				//output high???
    msleep(5);	
    mt_set_gpio_mode(TPIO_RESET, 0x00);		//GPIO mode        
    mt_set_gpio_dir(TPIO_RESET,1);
    mt_set_gpio_pull_enable(TPIO_RESET, 1);	//external pull up???        
    mt_set_gpio_pull_select(TPIO_RESET, GPIO_PULL_UP);	   
    mt_set_gpio_out(TPIO_RESET,1);
    msleep(200);    
//Ivan Reset tp if wake up failed 
    /*for(i=0;i<5;i++){
        err = tinno_check_focaltech();
     //   if (err > 5 &&  err < 0x80)
       if(err)
            break;
        mt_set_gpio_out(TPIO_RESET,0);				//output high
        msleep(10);
        mt_set_gpio_out(TPIO_RESET,1);				//output high
	 msleep(200); 
    }*/
    if (err ==0)
        printk("tinno_init_panel ****************************************************power on failed.\n");
   
    err = tinno_ts_set_period_active();
    if (err < 0)
        printk("tinno_init_panel ****************************************************active failed.\n");
    
}    


static void tinno_tp_power_off(void)
{
    
//    hwPowerDown(TPD_POWER_SOURCE,"TP");
    mt_set_gpio_mode(TPIO_WAKEUP, 0x00);		//GPIO mode        
    mt_set_gpio_dir(TPIO_WAKEUP,1);
    mt_set_gpio_pull_enable(TPIO_WAKEUP, 1);	//external pull up???        
    mt_set_gpio_pull_select(TPIO_WAKEUP, GPIO_PULL_DOWN);	       
    mt_set_gpio_out(TPIO_WAKEUP,0);

    msleep(10);        
    
    mt_set_gpio_mode(TPIO_RESET, 0x00);		//GPIO mode        
    mt_set_gpio_dir(TPIO_RESET,1);
    mt_set_gpio_pull_enable(TPIO_RESET, 1);	//external pull up???        
    mt_set_gpio_pull_select(TPIO_RESET, GPIO_PULL_DOWN);	   
    mt_set_gpio_out(TPIO_RESET,0);				//output low
    hwPowerDown(MT65XX_POWER_LDO_VGP4, "TP");
    
    msleep(50); 
    
}    


static int tpd_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id) {

    int err = 0;

    printk("ft5316 tpd_i2c_probe\n");

    if (i2c_client != NULL)
	return 1;
    
    i2c_client = client;

    mt_set_gpio_mode(TPIO_WAKEUP, 0x00);		//GPIO mode
    mt_set_gpio_dir(TPIO_WAKEUP,1);
    mt_set_gpio_pull_enable(TPIO_WAKEUP, 1);	//external pull up???        
    mt_set_gpio_pull_select(TPIO_WAKEUP, GPIO_PULL_UP);	           
    mt_set_gpio_out(TPIO_WAKEUP,1);				//output high???    
    msleep(2);   
    hwPowerOn(MT65XX_POWER_LDO_VGP4, VOL_2800, "TP");     
    msleep(2);
    
    mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);
    mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ZERO);
    msleep(1);

    mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ONE);

 //edit by Magnum 2012-7-5  add delay time and read 5 times to insure get the right version
	msleep(200);   //50
	
    int i = 0;
	while(i++ <5){
		panel_version = tinno_ts_get_fw_version();
		printk("Product  version ==== %X \n", panel_version);
	}
    if ( panel_version == 0 ){
	    TPD_DEBUG("Product  version %X is invalid.\n", panel_version);
	    panel_version = 0;
	    goto err_detect_failed;
    }
    
    tpd_load_status = 1;

    err = tinno_ts_set_period_active();
    if (err < 0) {
      TPD_DEBUG("tinno_init_panel failed.\n");
//      goto err_detect_failed;
    }


    thread = kthread_run(touch_event_handler, 0, TPD_DEVICE);
    if (IS_ERR(thread)) { 
        err = PTR_ERR(thread);
        TPD_DEBUG(TPD_DEVICE " failed to create kernel thread: %d\n", err);
    }

    /* added in android 2.2, for configuring EINT2 */
    mt_set_gpio_mode(TPIO_EINT, GPIO_CTP_EINT_PIN_M_EINT);
    mt_set_gpio_pull_enable(TPIO_EINT, GPIO_PULL_ENABLE);
    mt_set_gpio_pull_select(TPIO_EINT,GPIO_PULL_UP);

    mt65xx_eint_set_sens(CUST_EINT_TOUCH_PANEL_NUM, CUST_EINT_TOUCH_PANEL_SENSITIVE);
    mt65xx_eint_set_hw_debounce(CUST_EINT_TOUCH_PANEL_NUM, CUST_EINT_TOUCH_PANEL_DEBOUNCE_CN);
    mt65xx_eint_registration(CUST_EINT_TOUCH_PANEL_NUM, CUST_EINT_TOUCH_PANEL_DEBOUNCE_EN, CUST_EINT_TOUCH_PANEL_POLARITY, tpd_eint_interrupt_handler, 1);    
    
    mt65xx_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);
    //msleep(20);


    
#ifdef CONFIG_TOUCHSCREEN_FT5X05_SUPPORT_ISP
	fts_isp_register(i2c_client);
	mutex_init(&tp_mutex);	
#endif/*CONFIG_TOUCHSCREEN_FT5X05_SUPPORT_ISP*/

    tpd_status = 1;

    printk("tpd_i2c_probe OK\n");	
    return 0;

err_detect_failed:
    printk("tpd_i2c_probe ERROR\n");
    i2c_client = NULL;
    tinno_tp_power_off();
   return 1;
}

static int tpd_get_fw_version_stored()
{
//edit by Magnum 2012-3-1
//    panel_version = tinno_ts_get_fw_version();
    return panel_version;
}

//edit by Magnum 2012-7-10 
static int tpd_get_vendor_version_stored()
{
     return vendor_version;
}

static void tpd_eint_interrupt_handler(void) {
    TPD_DEBUG_PRINT_INT; tpd_flag=1; wake_up_interruptible(&waiter);
}


static void tpd_down(int x, int y, int id) {
    if(tpd_status)
    {
		TPD_DEBUG("tpd_down!\n");
	 if(RECOVERY_BOOT != get_boot_mode())
	   {
	    input_report_key(tpd->dev, BTN_TOUCH, 1);
	    input_report_abs(tpd->dev, ABS_MT_TOUCH_MAJOR, 128);
	    input_report_abs(tpd->dev, ABS_MT_POSITION_X, x);
	    input_report_abs(tpd->dev, ABS_MT_POSITION_Y, y);
		input_report_abs(tpd->dev, ABS_MT_TRACKING_ID, id-1); 
	    input_mt_sync(tpd->dev);
	    tpd_down_state=1;
	    down_x=x;
	    down_y=y;
	     }
		if (FACTORY_BOOT == get_boot_mode()|| RECOVERY_BOOT == get_boot_mode())
	    {   
	        //tpd_button(x, y, 1); 
		     int i ;
		     for(i=0;i< TPD_KEY_COUNT;i++) 
			{
	            if( (x >= (tpd_keys_dim_local[i][0] - tpd_keys_dim_local[i][2]/2)) && (x <=(tpd_keys_dim_local[i][0]+tpd_keys_dim_local[i][2]/2))
					&&(y >=	(tpd_keys_dim_local[i][1]-tpd_keys_dim_local[i][3]/2)) &&(y <=(tpd_keys_dim_local[i][1]+tpd_keys_dim_local[i][3]/2)) )
	            {
	            	
	            	TPD_DEBUG("tpd down value ==%d \n",tpd_keys_local[i]);
					input_report_key(tpd->dev, tpd_keys_local[i], 1);
				}
		    }  
		}
    }
}

static void tpd_up(int x, int y,int *count) {
    if(tpd_status){
        //Ivan
          if(phonestate)

	    TPD_DEBUG("tpd_up!\n");
	   if(RECOVERY_BOOT != get_boot_mode())
	   {
//Ivan	    input_report_abs(tpd->dev, ABS_PRESSURE, 0);
	    input_report_key(tpd->dev, BTN_TOUCH, 0);
//Ivan	    input_report_abs(tpd->dev, ABS_MT_TOUCH_MAJOR, 0);
//Ivan	    input_report_abs(tpd->dev, ABS_MT_POSITION_X, x);
//Ivan	    input_report_abs(tpd->dev, ABS_MT_POSITION_Y, y);
	    input_mt_sync(tpd->dev);
	    
             tpd_down_state=0;
             down_x=0;
             down_y=0;
	   	}
			 if (FACTORY_BOOT == get_boot_mode()|| RECOVERY_BOOT == get_boot_mode())
		    {   
		     //   tpd_button(x, y, 0); 
		     	 int i ;
			     for(i=0;i< TPD_KEY_COUNT;i++) 
				{
		          if( (x >= (tpd_keys_dim_local[i][0] - tpd_keys_dim_local[i][2]/2)) && (x <=(tpd_keys_dim_local[i][0]+tpd_keys_dim_local[i][2]/2))
					&&(y >=	(tpd_keys_dim_local[i][1]-tpd_keys_dim_local[i][3]/2)) &&(y <=(tpd_keys_dim_local[i][1]+tpd_keys_dim_local[i][3]/2)) )
		            {
		            	TPD_DEBUG("tpd up value ==%d \n",tpd_keys_local[i]);
						input_report_key(tpd->dev, tpd_keys_local[i], 0);
					}
			    }  
		    }  
        }
}

int tinno_ts_parse_data(struct touch_info *cinfo, struct touch_info *sinfo, uint8_t buffer[])
{
	int iInvalidTrackIDs = 0;
	fts_report_data_t report_data = {0};
	tinno_ts_point touch_point[TINNO_TOUCH_TRACK_IDS];
	int iTouchID, i,iKeyCode;

	memset(touch_point, FTS_INVALID_DATA, sizeof(touch_point));
	memcpy(&report_data, buffer, FTS_PROTOCOL_LEN);

	cinfo->count = 0;

	cinfo->pt[0].z = FTS_EF_RESERVED;
	cinfo->pt[1].z = FTS_EF_RESERVED;
	//edit by Magnum 2012-8-6
	cinfo->pt[2].z = FTS_EF_RESERVED;
	cinfo->pt[3].z = FTS_EF_RESERVED;
	cinfo->pt[4].z = FTS_EF_RESERVED;

#ifdef TPD_USE_VIRTUAL_KEY
    if (tp_boot_mode == RECOVERY_BOOT || tp_boot_mode == FACTORY_BOOT)
    {
    	//edit by Magnum 2012-9-20: make only one point in Recovery & Factory mode.
    	if (report_data.fingers > 0 && report_data.fingers < 2 )
		    tpd_dcount = 1;
		else
		    tpd_dcount = 0;
    }
    else
    {
		 tpd_dcount=((report_data.xy_data[0].event_flag  != FTS_EF_RESERVED)?1:0)+
               ((report_data.xy_data[1].event_flag  != FTS_EF_RESERVED)?1:0);	
    }
#else
       tpd_dcount=((report_data.xy_data[0].event_flag  != FTS_EF_RESERVED)?1:0)+
               ((report_data.xy_data[1].event_flag  != FTS_EF_RESERVED)?1:0);
#endif
       if(tpd_dcount){		
		for ( i=0; i < TINNO_TOUCH_TRACK_IDS; i++ ){
			if (report_data.xy_data[i].event_flag != FTS_EF_RESERVED) {
				iTouchID = report_data.xy_data[i].touch_id;
				if ( iTouchID >= TINNO_TOUCH_TRACK_IDS )
				{
					TPD_DEBUG("Invalied Track ID(%d)!\n", iTouchID);
					iInvalidTrackIDs++;
					continue;
				}
				cinfo->pt[i].x = report_data.xy_data[i].x_h << 8 | report_data.xy_data[i].x_l;
				cinfo->pt[i].y = report_data.xy_data[i].y_h << 8 | report_data.xy_data[i].y_l;
				cinfo->pt[i].z = report_data.xy_data[i].event_flag;	
				cinfo->pt[i].id = report_data.xy_data[i].touch_id;		
				TPD_DEBUG("iTpuchID ==%d, Point ID == %d, x == %d , y == %d ,z ==%d\n!",iTouchID, cinfo->pt[iTouchID].id,cinfo->pt[iTouchID].x,cinfo->pt[iTouchID].y,cinfo->pt[iTouchID].z );
				cinfo->count++;						//Two fingers touched

				//edit by Magnum 2012-8-31 ctp up question
				x_history[i] = cinfo->pt[iTouchID].x;
                y_history[i] = cinfo->pt[iTouchID].y;
				
			}
		}
//		cinfo->count = 	i;						//Two fingers touched		
		if ( TINNO_TOUCH_TRACK_IDS == iInvalidTrackIDs ){
			TPD_DEBUG("All points are Invalied, Ignore the interrupt!");
			return 1; 
		}
	}

	if (1==report_data.fingers)
	{
        if (cinfo->pt[0].y > TPD_BUTTON_HEIGHT)
        {
              // cinfo->count = 3;
              inTouchKey = 1;
        } 
		else 
			 inTouchKey = 0;  
     }
    if (0==report_data.fingers)
        cinfo->count = 0 ; //0xFF;		//Release
	    
	return 0;
}

int tpd_gettouchinfo(struct touch_info *cinfo, struct touch_info *sinfo) {

    int ret;
    uint8_t start_reg = 0x00;
    char buffer[40];
//	memset(buffer, 0xFF, 16);
    memset(buffer, 0xFF, FTS_PROTOCOL_LEN);
    TPD_DEBUG("Magnum FTS_PROTOCOL_LEN ===%d\n",FTS_PROTOCOL_LEN);

	mutex_lock(&tp_mutex);

	start_reg = 0x02;
	ret = i2c_master_send(i2c_client,&start_reg,1);
	ret = i2c_master_recv(i2c_client, &buffer[2], 5);
	
	if (buffer[2] > 1)
	{
	    start_reg = 9;
	    ret = i2c_master_send(i2c_client,&start_reg,1);
	    ret = i2c_master_recv(i2c_client, &buffer[9], 4);
	} 
	//edit by Magnum 2012-8-6
	if (buffer[9] > 1)
	{
	    start_reg = 0x0f;
	    ret = i2c_master_send(i2c_client,&start_reg,1);
	    ret = i2c_master_recv(i2c_client, &buffer[15], 4);
	} 
	if (buffer[15] > 1)
	{
	    start_reg = 0x15;
	    ret = i2c_master_send(i2c_client,&start_reg,1);
	    ret = i2c_master_recv(i2c_client, &buffer[21], 4);
	} 
	if (buffer[21] > 1)
	{
	    start_reg = 0x1b;
	    ret = i2c_master_send(i2c_client,&start_reg,1);
	    ret = i2c_master_recv(i2c_client, &buffer[27], 4);
	} 
//	ret = i2c_master_recv(i2c_client, &buffer[0],8 );
	//ret = i2c_master_recv(i2c_client, &buffer[8],8 );	
	//ret = i2c_master_recv(i2c_client, &buffer[16],8 );	
	//ret = i2c_master_recv(i2c_client, &buffer[24],8 );	
	//ret = i2c_master_recv(i2c_client, &buffer[32],1 );	
	

	mutex_unlock(&tp_mutex);

	if (ret < 0) {
	    TPD_DEBUG("i2c_transfer failed");
		return 1;							//Error
	}

/*	TPD_DEBUG("[%x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x]\n",buffer[0],
				buffer[1],buffer[2],buffer[3],buffer[4],buffer[5],
				buffer[6],buffer[7],buffer[8],buffer[9],buffer[10],buffer[11],buffer[12],buffer[13],
				buffer[14],buffer[15]); */
	TPD_DEBUG("[%x %x %x ]\n",buffer[0],buffer[1],buffer[2]);			
    int i = 0;int j =3;
	for(i;i<4;i++){
       TPD_DEBUG("[%x %x %x %x ]\n",buffer[j],buffer[j+1],buffer[j+2],buffer[j+3],buffer[j+4]);
	   j = j+6;
	}
				
    ret = tinno_ts_parse_data(cinfo, sinfo, buffer);
	return ret;
}


static int touch_event_handler(void *unused) {
    struct sched_param param = { .sched_priority = RTPM_PRIO_TPD };
    struct touch_info cinfo, sinfo;
    int pending = 0;

    TPD_DEBUG("touch_event_handler\n");
	
    cinfo.pending=0;
    sched_setscheduler(current, SCHED_RR, &param);
    do {
//Ivan added for testing
		if (tpd_status == 1){
	        mt65xx_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM); // possibly to lose event?
          }
        set_current_state(TASK_INTERRUPTIBLE);
        if (!kthread_should_stop()) {
            TPD_DEBUG_CHECK_NO_RESPONSE;
            do {
                if(pending) wait_event_interruptible_timeout(waiter, tpd_flag!=0, HZ/10);
                else wait_event_interruptible_timeout(waiter,tpd_flag!=0, HZ*2);
            } while(0);
            if(tpd_flag==0 && !pending) continue; // if timeout for no touch, then re-wait.
            if(tpd_flag!=0 && pending>0)  pending=0;
            tpd_flag=0;
            TPD_DEBUG_SET_TIME;
        }
        set_current_state(TASK_RUNNING);
        
        if(!pending) if(tpd_gettouchinfo(&cinfo, &sinfo)) continue; 
        if(pending>1) { pending--; continue; }
	//Ivan ++
		if (tpd_init_skip) {tpd_init_skip = 0; continue; }
		TPD_DEBUG("cinfo->count == %d!\n", cinfo.count);
		if(cinfo.count > 0)
		{
			int i;
			for ( i=0; i < cinfo.count; i++ )
			{
			    TPD_DEBUG("cinfo->count == %d!\n", cinfo.count);
				TPD_DEBUG("Point ID == %d, x == %d , y == %d ,z ==%d\n!",cinfo.pt[i].id,cinfo.pt[i].x,cinfo.pt[i].y,cinfo.pt[i].z );
				tpd_down(cinfo.pt[i].x, cinfo.pt[i].y, cinfo.pt[i].id);
			}
			input_sync(tpd->dev);
		}
		else
		{
			TPD_DEBUG("TPD up x == %d , y == %d\n!",x_history[cinfo.count], y_history[cinfo.count]);
			tpd_up(x_history[cinfo.count], y_history[cinfo.count], 0);		
			input_sync(tpd->dev);
		}    
    } while (!kthread_should_stop());
    return 0;
}

#ifdef CONFIG_HAS_EARLYSUSPEND

/* platform device functions */
void tpd_suspend(struct early_suspend *h) {
//    char sleep[2] = {0xA5,0x03};
    printk("[mtk-tpd] Suspend++.\n");
//edit by Magnum 2012-3-20
    if(isUpgrade ==1)
    {
        printk("Magnum tp is Upgrading.....\r\n");
		return;
    }
    mt65xx_eint_mask(CUST_EINT_TOUCH_PANEL_NUM);
    if(tpd_down_state){
        tpd_up(down_x,down_y,NULL);
//Ivan        input_report_abs(tpd->dev, ABS_MT_TRACKING_ID, 0);
        input_sync(tpd->dev);
        tpd_status = 0;
        msleep(200);
    }
    else
        tpd_status = 0;

    tinno_tp_power_off();
    TPD_DEBUG("TP tpd_suspend\n");
	
    if (i2c_client == NULL)
	return;

    printk("[mtk-tpd] Suspend--.tpd_down_state=%d\n",tpd_down_state);
}


void tpd_resume(struct early_suspend *h) {
//    char wakeup[2] = {0xA5,0x00};

//Ivan
    TPD_DEBUG("TP tpd_resume\n");
    
    if (i2c_client == NULL)
	return;
    
//    if (atomic_read( &g_pts->isp_opened ))
//	return;
    
    printk("[mtk-tpd] Resume++.\n");
    tpd_downMode = 0;
    tinno_tp_power_on();

    tpd_init_skip = 1;
    tpd_status = 1;
//Ivan 6573    MT6516_IRQUnmask(MT6516_TOUCH_IRQ_LINE); 
    mt65xx_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);  
    printk("[mtk-tpd] Resume--.\n");  
}
#endif


/* switch touch panel into single scan mode for decreasing interference */
void _tpd_switch_single_mode(void) {
}

/* switch touch panel into multiple scan mode for better performance */
void _tpd_switch_multiple_mode(void) {
}

/* switch touch panel into deep sleep mode */
void _tpd_switch_sleep_mode(void) {
    
//    char sleep[2] = {0x00,0x01};
    
    if (!tpd_status) {
        TPD_DEBUG("do not need to switch tpd into deep sleep m mode\n");
        return;
    }
    
    TPD_DEBUG("switch tpd into deep sleep mode\n");

    mt65xx_eint_mask(CUST_EINT_TOUCH_PANEL_NUM);
//Ivan 6573    MT6516_IRQMask(MT6516_TOUCH_IRQ_LINE);
//Ivan    i2c_master_send(i2c_client,sleep,2);
    
    tpd_status = 0;
    
    // workaround: power down tp will also pull down ic2 bus, affect other drivers
    //             so not pull down it.
    //hwPowerDown(TPD_POWER_SOURCE,"TP");
}

/* switch touch panel back to normal mode */
void _tpd_switch_normal_mode(void) {
    
//    char wakeup[2] = {0x00,0x05};
    
    if (tpd_status) {
        TPD_DEBUG("do not need to switch tpd back to normal mode\n");
        return;
    }
    
    TPD_DEBUG("switch tpd back to normal mode\n");
    
//Ivan    i2c_master_send(i2c_client,wakeup,2);
    
//Ivan 6573    MT6516_IRQUnmask(MT6516_TOUCH_IRQ_LINE);    
    mt65xx_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);    

    tpd_status = 1; 
}

#ifdef CONFIG_TOUCHSCREEN_FT5X05_SUPPORT_ISP

static inline int _lock(atomic_t *excl)
{
	if (atomic_inc_return(excl) == 1) {
		return 0;
	} else {
		atomic_dec(excl);
		return -1;
	}
}

static inline void _unlock(atomic_t *excl)
{
	atomic_dec(excl);
}

static ssize_t fts_isp_read(struct file *file, char __user *buf, size_t count, loff_t *offset)
{	
	//ret = copy_to_user(buf,&acc, sizeof(acc));
	TPD_DEBUG("");
	return -EIO;
}

static ssize_t fts_isp_write(struct file *file, const char __user *buf, size_t count, loff_t *offset)
{
	int rc = 0;
	struct tinno_ts_data *ts = file->private_data;
	const char __user *start = buf;

	TPD_DEBUG("count = %d \r\n", count);
	
	if ( count > ISP_FLASH_SIZE ){
		TPD_DEBUG("isp code is too long.");
		return -EDOM;
	}

	if ( copy_from_user(ts->isp_pBuffer, start, count) ){
		TPD_DEBUG("i2c_transfer failed(%d)", rc);
		return -EACCES;
	}

//	rc = i2c_master_send(ts->client, ts->isp_pBuffer, count);
	rc = tp_write_m_byte(0,ts->isp_pBuffer,count);
	if (rc == 0) {
		TPD_DEBUG("i2c_transfer failed(%d)", rc);
	} 

	return rc;
}

static int fts_isp_open(struct inode *inode, struct file *file)
{
	struct tinno_ts_data *ts = file->private_data;
    
	TPD_DEBUG("try to open isp.");

/*	
	if ( atomic_read( &g_pts->ts_sleepState ) ){
		TPD_DEBUG("TP is in sleep state, please try again latter.");
		return -EAGAIN;
	}
*/
	if (_lock(&g_pts->isp_opened)){
		TPD_DEBUG("isp is already opened.");
		return -EBUSY;
	}

	mutex_lock(&tp_mutex);

//Wakeup TP

	if (tpd_status == 0)
	{    
	    tinno_tp_power_on();
	}
//Wakeup End
	
	file->private_data = g_pts;

	g_pts->isp_pBuffer = (uint8_t *)kmalloc(ISP_FLASH_SIZE, GFP_KERNEL);
	if ( NULL == g_pts->isp_pBuffer ){
		_unlock ( &g_pts->isp_opened );
		TPD_DEBUG("no memory for isp.");
		return -ENOMEM;
	}
	tpd_status = 0;
    mt65xx_eint_mask(CUST_EINT_TOUCH_PANEL_NUM);
//Ivan 6573    MT6516_IRQMask(MT6516_TOUCH_IRQ_LINE);
	wake_lock(&g_pts->wake_lock);

	TPD_DEBUG("isp open success.");
	return 0;
}

static int fts_isp_close(struct inode *inode, struct file *file)
{
	struct tinno_ts_data *ts = file->private_data;
	
	TPD_DEBUG("try to close isp.");
/*	
	if ( !atomic_read( &g_pts->isp_opened ) ){
		TPD_DEBUG("no opened isp.");
		return -ENODEV;
	}
*/

	kfree(ts->isp_pBuffer);
	ts->isp_pBuffer = NULL;
/*	
	if ( atomic_read( &ts->ts_opened ) ){//don't dec the counter
		if (ts->use_irq){
			enable_irq(ts->client->irq);
		}
		else{
			hrtimer_start(&ts->timer, ktime_set(1, 0), HRTIMER_MODE_REL);
		}
		TPD_DEBUG("open the opened touch device.");
	}
*/	


	mutex_unlock(&tp_mutex);
	tpd_status = 1;
//Ivan 6573	MT6516_IRQUnmask(MT6516_TOUCH_IRQ_LINE);    
    mt65xx_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);  

	file->private_data = NULL;
	
	_unlock ( &ts->isp_opened );
	
	wake_unlock(&ts->wake_lock);
	
	TPD_DEBUG("close isp success!");
	return 0;
}

#if defined(FT5X06_FIRMWAIR_VERSION_C)
static int fts_switch_to_update(struct tinno_ts_data *ts)
{
	int ret = 0;
	uint8_t  command = 0xFC;
	
	TPD_DEBUG("");
	
//	mutex_lock(&tp_mutex);

//	ret = i2c_smbus_read_byte_data(ts->client, command);	
    i2c_master_send(ts->client,&command,1);
	i2c_master_recv(ts->client, &ret, 1);
	

	if (ret < 0) {
		TPD_DEBUG("I2C transfer failed!(%d)\n", ret);
		goto err;
	}
	if ( 1 != ret ){
		TPD_DEBUG("FTS mode error!(%d)\n", ret);
		goto err;
	}
	/*write 0xaa to register 0xfc*/
	ret = i2c_smbus_write_byte_data(ts->client, command, 0xAA);
	if (ret < 0) {
		TPD_DEBUG("set period active failed\n");
		goto err;
	}
	msleep(10);/*wait 10 ms: needed!!!*/
	
	/*check the register 0xfc again*/
//	ret = i2c_smbus_read_byte_data(ts->client, command);
    i2c_master_send(ts->client,&command,1);
	i2c_master_recv(ts->client, &ret, 1);
	
	if (ret < 0) {
		TPD_DEBUG("I2C transfer failed!(%d)\n", ret);
		goto err;
	}
	if ( 0xAA != ret ){
		TPD_DEBUG("FTS mode switch error!(%d)\n", ret);
		goto err;
	}
	
	msleep(10);/*wait 10 ms: needed!!!*/
	
	/*write 0x55 to register 0xfc*/
	ret = i2c_smbus_write_byte_data(ts->client, command, 0x55);
	if (ret < 0) {
		TPD_DEBUG("set period active failed\n");
		goto err;
	}
	
	msleep(10);/*wait 10 ms: needed!!!*/

	ret = 0;
err:
//	mutex_unlock(&tp_mutex);
	return ret;
}

#elif defined(FT5X06_FIRMWAIR_VERSION_D)

static int fts_switch_to_update(struct tinno_ts_data *ts)
{
	int ret = 0;
	uint8_t  command = 0xFC;
	uint8_t arrCommand[] = {0x55, 0xaa};
	
	TPD_DEBUG("");
    
//	mutex_lock(&tp_mutex);
	
	/*write 0xaa to register 0xfc*/
	ret = i2c_smbus_write_byte_data(ts->client, command, 0xAA);
	if (ret < 0) {
		TPD_DEBUG("write 0xaa to register 0xfc failed\n");
		goto err;
	}
	msleep(50);
	/*write 0x55 to register 0xfc*/
	ret = i2c_smbus_write_byte_data(ts->client, command, 0x55);
	if (ret < 0) {
		TPD_DEBUG("write 0x55 to register 0xfc failed\n");
		goto err;
	}
	msleep(40);
	ret = i2c_master_send(ts->client, arrCommand, sizeof(arrCommand));
	if (ret < sizeof(arrCommand)) {
		TPD_DEBUG("i2c_transfer failed 5(%d)\n", ret);
	} 
	ret = 0;
	//edit by Magnum 2012-7-19
	isUpgrade = 1;
err:
//	mutex_unlock(&tp_mutex);
	return ret;
}

#endif

static int fts_mode_switch(struct tinno_ts_data *ts, int iMode)
{
	int ret = 0;
	
	TPD_DEBUG("iMode=%d\n", iMode);
	
	if ( FTS_MODE_OPRATE == iMode ){
	}
	else if (FTS_MODE_UPDATE == iMode){
		ret = fts_switch_to_update(ts);
	}
	else if (FTS_MODE_SYSTEM == iMode){
	}
	else{
		TPD_DEBUG("unsupport mode %d\n", iMode);
	}
	return ret;
}


//edit by Magnum 2012-2-2
static int  fts_ctpm_auto_clb(void)
{
    uint8_t  command = 0xFC;
    uint8_t  calib_buf=0x00;
    u8 uc_temp;
    u8 i ;   
    printk("[FTS] start auto CLB.\n");
    mdelay(200);
    //ft520x_write_reg(0, 0x40); 
    i2c_smbus_write_byte_data(i2c_client, 0x00, 0x40);
 
    mdelay(100);   //make sure already enter factory mode
    //ft520x_write_reg(2, 0x4);  //write command to start calibration
    i2c_smbus_write_byte_data(i2c_client, 0x02, 0x04);

    mdelay(300);
    for(i=0;i<100;i++)
    {
        //ft520x_read_reg(0,&uc_temp);
        command = 0x00;
        i2c_master_send(i2c_client,&command,1);
        i2c_master_recv(i2c_client, &calib_buf, 1);
        if ( ((calib_buf&0x70)>>4) == 0x0)  //return to normal mode, calibration finish
        {
            break;
        }
        mdelay(200);
        printk("[FTS] waiting calibration %d\n",i);
        
    }
    printk("[FTS] calibration OK.\n");
    
   mdelay(300);
    //ft520x_write_reg(0, 0x40);  //goto factory mode
    i2c_smbus_write_byte_data(i2c_client, 0x00, 0x40);

    mdelay(100);   //make sure already enter factory mode
    //ft520x_write_reg(2, 0x5);  //store CLB result
    i2c_smbus_write_byte_data(i2c_client, 0x02, 0x05);

    mdelay(300);
    //ft520x_write_reg(0, 0x0); //return to normal mode 
    i2c_smbus_write_byte_data(i2c_client, 0x00, 0x00);
    mdelay(300);
    printk("[FTS] store CLB result OK.\n");
    return 0;
}

//edit by Magnum 2012-7-18
//static int fts_isp_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg)
static int fts_isp_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	void __user *argp = (void __user *)arg;
	struct tinno_ts_data *ts = file->private_data;
	int flag;
	int rc = 0;
	uint8_t  command = 0xFC;
	uint8_t  checksum,calib_buf;
	
	if ( !atomic_read( &g_pts->isp_opened ) ){
		TPD_DEBUG("no opened isp.\n");
		return -ENODEV;
	}
	
	/* check cmd */
	if(_IOC_TYPE(cmd) != TOUCH_IO_MAGIC)	
	{
		TPD_DEBUG("cmd magic type error\n");
		return -EINVAL;
	}
	if(_IOC_NR(cmd) > FT5X05_IOC_MAXNR)
	{
		TPD_DEBUG("cmd number error\n");
		return -EINVAL;
	}

	if(_IOC_DIR(cmd) & _IOC_READ)
		rc = !access_ok(VERIFY_WRITE,(void __user*)arg, _IOC_SIZE(cmd));
	else if(_IOC_DIR(cmd) & _IOC_WRITE)
		rc = !access_ok(VERIFY_READ, (void __user*)arg, _IOC_SIZE(cmd));
	if(rc)
	{
		TPD_DEBUG("cmd access_ok error\n");
		return -EINVAL;
	}

	switch (cmd) {
	case FT5X05_IOCTL_SWITCH_TO:
		TPD_DEBUG("Try to switch to update mode!(%lu)\n", arg);
		rc = fts_mode_switch(ts, (int)arg);
		if(rc)
		{
			TPD_DEBUG("switch to update mode error\n");
			return -EIO;
		}
		break;
	case FT5X05_IOCTL_WRITE_PROTECT:
		TPD_DEBUG("Try to set write protect mode!(%lu)\n", arg);
#if defined(FT5X06_FIRMWAIR_VERSION_D)
		rc = -EINVAL;
		break;
#endif
		if ( 0 == arg ){/*write enable*/
			rc = i2c_smbus_write_byte(ts->client, 0x06);
		}
		else{/*write disable*/
			rc = i2c_smbus_write_byte(ts->client, 0x04);
		}
		if (rc < 0) {
			TPD_DEBUG("set period active failed\n");
		}
		break;
	case FT5X05_IOCTL_ERASE:
		TPD_DEBUG("Try to erase flash!\n");
#if defined(FT5X06_FIRMWAIR_VERSION_C)
		rc = i2c_smbus_write_byte(ts->client, 0x60);
#elif defined(FT5X06_FIRMWAIR_VERSION_D)
		rc = i2c_smbus_write_byte(ts->client, 0x61);
#endif
		if (rc < 0) {
			TPD_DEBUG("erase failed\n");
			break;
		}
		msleep(1500);
		break;
	case FT5X05_IOCTL_GET_STATUS:
		TPD_DEBUG("Try to get status!\n");
//		flag = i2c_smbus_read_byte_data(ts->client, 0x05);
		command = 0x05;
		i2c_master_send(ts->client,&command,1);
		i2c_master_recv(ts->client, &flag, 1);
		
		if (flag < 0) {
			TPD_DEBUG("read check status failed\n");
		}
		TPD_DEBUG("status=%d!", flag);
		if(copy_to_user(argp,&flag,sizeof(int))!=0)
		{
			printk(KERN_INFO "copy_to_user error\n");
			rc = -EFAULT;
		}
		break;
	case FT5X05_IOCTL_GET_CHECKSUM:
		TPD_DEBUG("Try to get checksum!\n");
//		flag = i2c_smbus_read_byte_data(ts->client, 0xCC);
		command = 0xCC;
		i2c_master_send(ts->client,&command,1);
		i2c_master_recv(ts->client, &checksum, 1);
		
//		if (checksum < 0) {
//			TPD_DEBUG("read checksum failed\n");
//		}
		flag = checksum;
		TPD_DEBUG("checksum=%x!\n", checksum);
		if(copy_to_user(argp,&flag,sizeof(int))!=0)
		{
			printk(KERN_INFO "copy_to_user error\n");
			rc = -EFAULT;
		}
		break;
	case FT5X05_IOCTL_RESET:
		TPD_DEBUG("Try to reset TP!\n");
		rc = i2c_smbus_write_byte(ts->client, 0x07);
		if (rc < 0) {
			TPD_DEBUG("reset failed\n");
		}
//edit by Magnum 2012-2-2
        fts_ctpm_auto_clb();
        //edit by Magnum 2012-7-19
	    isUpgrade = 0;
		//edit by Magnum 2012-7-19  if upgrade TP success, update the panel_version
		 panel_version = tinno_ts_get_fw_version();
		tpd_reset();
		break;
	case FT5X05_IOCTL_GET_TPID:
		{
			uint8_t arrCommand[] = {0x90, 0x00, 0x00, 0x00};

			TPD_DEBUG("Try to get TPID!\n");
			rc = i2c_master_send(ts->client, arrCommand, sizeof(arrCommand));
			if (rc < sizeof(arrCommand)) {
				TPD_DEBUG("i2c_master_send failed(%d)\n", rc);
			} 

			rc = i2c_master_recv(ts->client, arrCommand, 2);
			if (rc < 2) {
				TPD_DEBUG("i2c_master_recv failed(%d)\n", rc);
			} 

			flag = (( int )arrCommand[0] << 8) | (( int )arrCommand[1]);
			
			TPD_DEBUG("TPID=0x%X!\n", flag);
			if(copy_to_user(argp,&flag,sizeof(int))!=0)
			{
				printk(KERN_INFO "copy_to_user error\n");
				rc = -EFAULT;
			}
			break;
		}
	case FT5X05_IOCTL_GET_VENDORID:
            {
               char readbyte[2] = {0xA8};
               char vendor_id;
               int err;
           
                // Read FW Version.
                //	fw_version = i2c_smbus_read_byte_data(i2c_client, 0xA6);
                readbyte[0] = 0xA8;
                i2c_master_send(i2c_client,&readbyte[0],1);
                err = i2c_master_recv(i2c_client, &vendor_id, 1);

                if (vendor_id < 0 || err < 0){
                	TPD_DEBUG("i2c_smbus_write_byte_data failed.\n");
                    return 0;
                }

                printk("Magnum vendor_id=0x%X \n", vendor_id);

                if(copy_to_user(argp,&vendor_id,sizeof(char))!=0)
                {
                    printk(KERN_INFO "copy_to_user error\n");
                    rc = -EFAULT;
                }
                break;
            }
	default:
		TPD_DEBUG("invalid command %d\n", _IOC_NR(cmd));
		rc = -EINVAL;
		break;
	}

	return rc;
}


static const struct file_operations fts_isp_fops = {
	.owner = THIS_MODULE,
	.read = fts_isp_read,
	.write = fts_isp_write,
	.open = fts_isp_open,
	.release = fts_isp_close,
	.unlocked_ioctl = fts_isp_ioctl,
};

static struct miscdevice fts_isp_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "fts_isp",
	.fops = &fts_isp_fops,
};

static void fts_isp_register(struct i2c_client *client)
{
	struct tinno_ts_data *ts;
	int err,ret;
	
	ts = kzalloc(sizeof(*ts), GFP_KERNEL);
	if (ts == NULL) {
		ret = -ENOMEM;
		goto err_detect_failed;
	}

	wake_lock_init(&ts->wake_lock, WAKE_LOCK_SUSPEND, "fts_tp_isp");
	ts->client = i2c_client;
	err = misc_register(&fts_isp_device);
	if (err) {
		printk(KERN_ERR "fts_isp_device device register failed\n");
		goto exit_misc_device_register_failed;
	}else{
		g_pts = ts;
	}
	
	return;

err_detect_failed:
	kfree(ts);
	return;	
	
exit_misc_device_register_failed:
	wake_lock_destroy(&ts->wake_lock);
}
#endif

static int tpd_i2c_remove(struct i2c_client *client) {
    i2c_unregister_device(client);
    printk("[mtk-tpd] touch panel i2c device is removed.\n");
//[wj add]
       input_unregister_device(tpd->dev);
       input_unregister_device(tpd->kpd);
//[wj add end]
#ifdef CONFIG_TOUCHSCREEN_FT5X05_SUPPORT_ISP
	misc_deregister(&fts_isp_device);
	wake_lock_destroy(&g_pts->wake_lock);
	g_pts = NULL;
#endif		
    return 0;
}


static struct tpd_driver_t tpd_device_driver = {
		.tpd_device_name = "FT5316",
		.tpd_local_init = tpd_local_init,
		.suspend = tpd_suspend,
		.resume = tpd_resume,
#ifdef TPD_HAVE_BUTTON
		.tpd_have_button = 1,
#else
		.tpd_have_button = 0,
#endif		
		.tpd_x_res = TPD_CUST_RES_X,
		.tpd_y_res = TPD_CUST_RES_Y,	//including button area
        .tpd_get_fw_version = tpd_get_fw_version_stored,
        .tpd_get_vendor_version = tpd_get_vendor_version_stored,  //edit by Magnum 2012-7-10
};
/* called when loaded into kernel */
static int __init tpd_driver_init(void) {
    printk("MediaTek FT5316 touch panel driver init\n");
	   i2c_register_board_info(TPD_I2C_NUMBER, &ft5206_i2c_tpd, 1);    
		if(tpd_driver_add(&tpd_device_driver) < 0)
			TPD_DMESG("add generic driver failed\n");
    return 0;
}

/* should never be called */
static void __exit tpd_driver_exit(void) {
    TPD_DMESG("MediaTek FT5316 touch panel driver exit\n");
    //input_unregister_device(tpd->dev);
    tpd_driver_remove(&tpd_device_driver);
}

module_init(tpd_driver_init);
module_exit(tpd_driver_exit);

