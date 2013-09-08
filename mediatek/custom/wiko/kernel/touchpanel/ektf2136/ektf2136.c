/*****************************************************************************
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 *
*****************************************************************************/
/*****************************************************************************
 *
 * Filename:
 * ---------
 *  elan_driver.c
 *
 * Project:
 * --------
 *  W990
 *
 * Author:
 * -------
 *  konka
 *
 * Description:
 * ------------
 *  CTP's driver for elong's chip 
 *
 *
 *============================================================================
 * History:
 * May 7 2012, creat
 *
 *============================================================================
 ****************************************************************************/
#include "tpd_custom_ektf2136.h"



/*****************************************************************************
*                E X T E R N A L      R E F E R E N C E S
******************************************************************************
*/
extern struct tpd_device *tpd;
struct i2c_client * ektf2136_i2c_client = NULL;
//static struct i2c_client *i2c_client = NULL;
static struct task_struct *thread = NULL;

/*****************************************************************************
*           F U N C T I O N      D E L A R A T I O N
******************************************************************************
*/
static DECLARE_WAIT_QUEUE_HEAD(waiter);
static void tpd_eint_interrupt_handler(void);
extern void mt65xx_eint_unmask(unsigned int line);
extern void mt65xx_eint_mask(unsigned int line);
extern void mt65xx_eint_set_hw_debounce(kal_uint8 eintno, kal_uint32 ms);
extern kal_uint32 mt65xx_eint_set_sens(kal_uint8 eintno, kal_bool sens);
extern void mt65xx_eint_registration(kal_uint8 eintno, kal_bool Dbounce_En,
                                     kal_bool ACT_Polarity, void (EINT_FUNC_PTR)(void),
                                     kal_bool auto_umask);
static int __devinit tpd_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int tpd_detect(struct i2c_client *client, int kind, struct i2c_board_info *info);
static int __devexit tpd_remove(struct i2c_client *client);
static int touch_event_handler(void *unused);
static int tpd_local_init(void);
static int tpd_suspend(struct i2c_client *client, pm_message_t message);
static int tpd_resume(struct i2c_client *client);
static int ektf2136_get_fw_version_stored(void);
static int ektf2136_get_vendor_version_stored(void);

/*****************************************************************************
*                          C O N S T A N T S
******************************************************************************
*/
#if defined (SUPORT_5_POINTS)
#define POINTS_NUM                5
#else
#define POINTS_NUM                2
#endif

static int elan_status = 0;  // 1: elan working well, 0: elan not working or bad
#ifdef  ELAN_ESD_PROTECT  
#define TPD_ESD_CHECK_CIRCLE        2*HZ
 //  when CTP IC send eint , value elan_ic_power_state ==1, after eint handler 
 // func  value elan_ic_power_state == 0, so   1: has eint ,   0: no eint -> power state 
 // error
static int elan_ic_power_state = 0 ;  
static struct delayed_work elan_esd_check_work;
static struct workqueue_struct *elan_esd_check_workqueue = NULL;
#endif
//static struct mutex elan_mutex;
static DEFINE_MUTEX(elan_mutex);
extern void tpd_reset();
extern void tpd_wr_datas();
extern void tpd_rd_datas();
extern  int ektf2136_ts_get_fw_version(void);
extern  int ektf2136_register_misc(void);
char ektf_panel_version = 0;
char  ektf_vendor_version= 0 ;
int work_lock=0x00;

static int ctp_suspend= -1; // show that: ctp is power_down or not
static int tpd_flag = 0;
static int point_num = 0;
static int p_point_num = 0;
  //edit by Magnum 2013-3-7 edit for s9111 
static int  in_key = 0;
static int ektf_boot_mode;
static int x_history[POINTS_NUM];
static int y_history[POINTS_NUM];
static struct touch_info old_cinfo;

static const struct i2c_device_id tpd_id[] = {{tpd_driver_name, 0}, {}};
static struct i2c_board_info __initdata elan_i2c_tpd = { I2C_BOARD_INFO(tpd_driver_name, (slaver_addr >> 1))};

#ifdef TPD_HAVE_BUTTON 
static int tpd_keys_local[TPD_KEY_COUNT] = TPD_KEYS;
static int tpd_keys_dim_local[TPD_KEY_COUNT][4] = TPD_KEYS_DIM;
extern void tpd_button(unsigned int x, unsigned int y, unsigned int down);
#endif

#if (defined(TPD_HAVE_CALIBRATION) && !defined(TPD_CUSTOM_CALIBRATION))
static int tpd_calmat_local[8]     = TPD_CALIBRATION_MATRIX;
static int tpd_def_calmat_local[8] = TPD_CALIBRATION_MATRIX;
#endif

#ifdef      SUPPORT_UPDATE_FW
static struct task_struct *s_update_thread;
extern int update_firmware_thread(void *priv);
#endif




// Firmware Information
// For Firmare Update 


/*****************************************************************************
*                         D A T A      T Y P E S
******************************************************************************
*/
struct touch_info
{
    int y[POINTS_NUM];
    int x[POINTS_NUM];
    int touch_id[POINTS_NUM];
    int count;
};

static struct i2c_driver tpd_i2c_driver =
{
    .driver = {
        .name = tpd_driver_name,
        //       .owner = THIS_MODULE,
    },
    .probe = tpd_probe,
    .remove = __devexit_p(tpd_remove),
    .id_table = tpd_id,
    .detect = tpd_detect,
    //    .address_data = &addr_data,
};

static struct tpd_driver_t tpd_device_driver =
{
    .tpd_device_name = tpd_driver_name,
    .tpd_local_init = tpd_local_init,
    .suspend = tpd_suspend,
    .resume = tpd_resume,
    .tpd_have_button = 1,
	.tpd_x_res = TPD_CUST_RES_X,
	.tpd_y_res = TPD_CUST_RES_Y,	//including touch area
    .tpd_get_fw_version = ektf2136_get_fw_version_stored,
	.tpd_get_vendor_version = ektf2136_get_vendor_version_stored,

};


/*****************************************************************************
*                  F U N C T I O N        D E F I N I T I O N
******************************************************************************
*/



static void tpd_down(int x, int y, int id) {
  
	
	if(elan_status ==1)
	{
		TPD_DEBUG("tpd_down! x == %d, y == %d \n",x,y);
	//Ivan	
		if (RECOVERY_BOOT != get_boot_mode())
		{
		    input_report_key(tpd->dev, BTN_TOUCH, 1);
		    input_report_abs(tpd->dev, ABS_MT_TOUCH_MAJOR, 128);
		    input_report_abs(tpd->dev, ABS_MT_POSITION_X, x);
		    input_report_abs(tpd->dev, ABS_MT_POSITION_Y, y);
		    input_report_abs(tpd->dev, ABS_MT_TRACKING_ID, id); 
		    input_mt_sync(tpd->dev);
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

/*	    if(elan_status ==0){
		  TPD_DEBUG("no touch in the panel,do not report tpd_up!\n");	
		  return ;
	    }  */
	 if(elan_status ==0)
	{
		    TPD_DEBUG("TPD up x == %d , y == %d\n!",x, y);
		    if(RECOVERY_BOOT != get_boot_mode())
		   {
			//Ivan	    input_report_abs(tpd->dev, ABS_PRESSURE, 0);
				    input_report_key(tpd->dev, BTN_TOUCH, 0);
			//Ivan	    input_report_abs(tpd->dev, ABS_MT_TOUCH_MAJOR, 0);
			//Ivan	    input_report_abs(tpd->dev, ABS_MT_POSITION_X, x);
			//Ivan	    input_report_abs(tpd->dev, ABS_MT_POSITION_Y, y);
				    input_mt_sync(tpd->dev);	    
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
		   elan_status = 0;
	 }
}

static inline int ektf2136_ts_parse_xy(uint8_t *data,
			uint16_t *x, uint16_t *y)
{
	*x = *y = 0;
	*x = (data[0] & 0xf0);
	*x <<= 4;
	*x |= data[1];
	*y = (data[0] & 0x0f);
	*y <<= 8;
	*y |= data[2];

	return 0;
}
static int tpd_touchinfo(struct touch_info *cinfo, struct touch_info *pinfo)
{
    int i, j;
	int ret = 0;
    uint8_t index[5] = {0};
    uint8_t fingers = 0;
	uint8_t raw_data_buf[18]= {0};
    int tpd_dcount;
    u16 x, y;
	uint8_t idx;

	cinfo->count = 0;
#ifdef SUPORT_5_POINTS
//Ivan    ret = i2c_master_recv(i2c_client, raw_data_buf, 18);  // i2c support > 8bits transfer
	ret = i2c_master_recv(ektf2136_i2c_client, raw_data_buf, 8);  // i2c support > 8bits transfer
    if (ret != 8)
        printk("[elan] %s I2C Transfer Error\n", __func__);
	ret = i2c_master_recv(ektf2136_i2c_client, &raw_data_buf[8], 8);  // i2c support > 8bits transfer
    if (ret != 8)
        printk("[elan] %s I2C Transfer Error\n", __func__);
	ret = i2c_master_recv(ektf2136_i2c_client, &raw_data_buf[16], 2);  // i2c support > 8bits transfer
    if (ret != 2)
        printk("[elan] %s I2C Transfer Error\n", __func__);
	
	//mutex_unlock(&elan_mutex);  
    p_point_num = point_num;    //remerber the last point number
  
// for 5 fingers	
    if ((raw_data_buf[0] == 0x6D))  //6D for packet <=8 , 5D for packet > 8
    {   	  
	    TPD_DEBUG("\n/********************************************/\n");
	    for (i=0; i<18; i++) {
	        TPD_DEBUG("%x  ", raw_data_buf[i]);
	        if ((i%6 == 0) && (i != 0)) {
	            TPD_DEBUG("\n");
	        }
	    }
	    TPD_DEBUG("/********************************************/\n");
        	point_num = raw_data_buf[1] & 0x07;   
		//value touch_id...
		idx = raw_data_buf[1]>>3;
		int j = 0;
        TPD_DEBUG("[elan] point_num =%d\n", point_num);
#ifdef TPD_USE_VIRTUAL_KEY
		if (ektf_boot_mode == RECOVERY_BOOT || ektf_boot_mode == FACTORY_BOOT)
		{
			//edit by Magnum 2012-9-20: make only one point in Recovery & Factory mode.
			if (point_num > 0 && point_num < 2 )
			    tpd_dcount = 1;
			else
			    tpd_dcount = 0;
		}
		else
		{
			tpd_dcount = point_num; 
		}
#else
		  tpd_dcount = point_num; 
#endif

      //edit by Magnum 2013-3-7 edit for s9111 
         if ((raw_data_buf[17] == 0x21)){
		tpd_dcount = 1;
		in_key =1;
		cinfo->x[p_point_num] =TPD_CUST_KEY_X3;
		cinfo->y[p_point_num] = TPD_CUST_KEY_Y;
		cinfo->count = ++p_point_num;
		x_history[j] = cinfo->x[j];
            	y_history[j] = cinfo->y[j];
	}
	else  if ((raw_data_buf[17] == 0x41)){
		tpd_dcount = 1;
		in_key =1;
		cinfo->x[p_point_num] =TPD_CUST_KEY_X2;
		cinfo->y[p_point_num] = TPD_CUST_KEY_Y;
		cinfo->count = ++p_point_num;
		x_history[j] = cinfo->x[j];
            	y_history[j] = cinfo->y[j];
	}
	else  if ((raw_data_buf[17] == 0x81)){
		tpd_dcount = 1;
		in_key = 1;
		cinfo->x[p_point_num] =TPD_CUST_KEY_X1;
		cinfo->y[p_point_num] = TPD_CUST_KEY_Y;
		cinfo->count = ++p_point_num;
		x_history[j] = cinfo->x[j];
            	y_history[j] = cinfo->y[j];
	}
	else 
		in_key = 0;
    	if(tpd_dcount > 0)
		elan_status = 1;
	else if(tpd_dcount == 0)
		elan_status = 0;
	TPD_DEBUG("[elan::] elan status == %d\n",elan_status);
        TPD_DEBUG("Magnum tpd_dcount == %d, in key == %d  \n",tpd_dcount,in_key);
	    if(tpd_dcount && in_key == 0)
	    {
	        struct touch_info tinfo;
	        for (i = 0; i < POINTS_NUM; i++) 
			{	
			//	TPD_DEBUG("Magnum idx ===%d\n",idx);
			    if((idx & 0x01) == 1)
				{
					cinfo->touch_id[j] = i;
					TPD_DEBUG("Magnum touchid ===%d\n",cinfo->touch_id[j]);
					ektf2136_ts_parse_xy(&raw_data_buf[ 2 + 3*i ], &x, &y);	
					x = x * TPD_CUST_RES_X / EKTF_WIDTH;
					y = y * TPD_CUST_RES_Y / EKTF_HEIGHT;
					cinfo->x[j] = x;
					cinfo->y[j] = y;
					cinfo->count++;
					x_history[j] = cinfo->x[j];
	            			y_history[j] = cinfo->y[j];
					j++;
			    }
				idx>>= 1;	// left and right value , please value up it
			} // end for 
	    }
   }	
   else if((raw_data_buf[0] == 0x78)){
		//elan_status = 0;
   }
   else{
   	//	elan_status = 0;
	    printk("\n/********************************************/\n");
	    for (i=0; i<18; i++) {
	        printk("%x  ", raw_data_buf[i]);
	        if ((i%6 == 0) && (i != 0)) {
	           printk("\n");
	        }
	    }
	    printk("/********************************************/\n");
   	}

#endif 
	return true;
};

#ifdef ELAN_ESD_PROTECT
static void elan_esd_reset()
{
	TPD_DEBUG("elan esd reset.....\n");
	hwPowerDown(EKTF_POWER, "TP");
	msleep(40);
	hwPowerOn(EKTF_POWER, VOL_2800, "TP");
	tpd_reset(); 			    
	msleep(400);
}

static void elan_esd_check(struct work_struct *work)
{
	//TPD_DEBUG("elan_esd_check\n");
	TPD_DEBUG("elan_esd_check -- CTP  elan_ic_power_state == %d \n",elan_ic_power_state);		
	if(work_lock ) //tp is upgrading...
	{
		TPD_DEBUG("elan_esd_check -- CTP is upgrading , work_lock == %d  exit elan_esd_check....\n",work_lock);
		queue_delayed_work(elan_esd_check_workqueue, &elan_esd_check_work, TPD_ESD_CHECK_CIRCLE);
     		return ;
	}  
	
	if(elan_ic_power_state)
		elan_ic_power_state = 0;  
	else{
		printk("Magnum elan esd FAIL FAIL ...\n");
		elan_esd_reset();		
	}
	queue_delayed_work(elan_esd_check_workqueue, &elan_esd_check_work, TPD_ESD_CHECK_CIRCLE);
	return ;
	
/*	uint8_t cmd[] = {0x53, 0x50, 0x00, 0x01};
	
//	uint8_t buf[4] = {0};
//	int rc = 0;

//	mt65xx_eint_mask(CUST_EINT_TOUCH_PANEL_NUM);

	rc = i2c_master_send(ektf2136_i2c_client, cmd, 4);
	if (rc != 4)
	{
		printk("elan esd sent cmd error , reset now\n");
		goto esdreset;
	}
	else{
		msleep(5);  
	rc = i2c_master_recv(ektf2136_i2c_client, buf, 4);
	if(rc != 4){
		printk("elan esd recv data error\n");
		goto esdreset;
	}	
	 TPD_DEBUG("elan esd receive buf[0] == %x,buf[1] == %x,buf[2] == %x,buf[3] == %x...\n",buf[0],buf[1],buf[2],buf[3]);
	if(buf[0] != 0x52){
		printk("elan esd respone error\n");
		goto esdreset;
	}	
	else{
		TPD_DEBUG("elan esd respone normal\n");
		mt65xx_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);
	  	queue_delayed_work(elan_esd_check_workqueue, &elan_esd_check_work, TPD_ESD_CHECK_CIRCLE);
		
		return;
	}  
						
esdreset:
	//reset code,must be power off then power on
	elan_esd_reset();	
	mt65xx_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);
	queue_delayed_work(elan_esd_check_workqueue, &elan_esd_check_work, TPD_ESD_CHECK_CIRCLE);
	 
	 return ;  */
}

#endif

static void tpd_eint_interrupt_handler(void)
{
    TPD_DEBUG("tpd_eint_interrupt_handler......\n");
    tpd_flag = 1;
    #ifdef  ELAN_ESD_PROTECT  
    elan_ic_power_state  = 1;
    #endif 
    wake_up_interruptible(&waiter);
}

static int touch_event_handler(void *unused)
{
    int i = 0;
    struct touch_info cinfo = {0};
    struct touch_info pinfo = {0};
    static struct touch_info lastpoint = {0};
    
    struct sched_param param = { .sched_priority = RTPM_PRIO_TPD };
    sched_setscheduler(current, SCHED_RR, &param);

    do {
wait:
        mt65xx_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);
        set_current_state(TASK_INTERRUPTIBLE);
        wait_event_interruptible(waiter, tpd_flag != 0);
        tpd_flag = 0;
	 mt65xx_eint_mask(CUST_EINT_TOUCH_PANEL_NUM);
        set_current_state(TASK_RUNNING);
//	memset(&cinfo, 0 ,sizeof(struct touch_info));
        tpd_touchinfo(&cinfo, &pinfo);          
        TPD_DEBUG("cinfo->count == %d!\n", cinfo.count);
	if(cinfo.count > 0)
	{
		int i;
		for ( i=0; i < cinfo.count; i++ )
		{
			TPD_DEBUG("Point ID == %d, x == %d , y == %d\n!",cinfo.touch_id[i],cinfo.x[i],cinfo.y[i]);
			tpd_down(cinfo.x[i], cinfo.y[i], cinfo.touch_id[i]);
		}
		input_sync(tpd->dev);
	}
	else
	{
		tpd_up(x_history[cinfo.count], y_history[cinfo.count], 0);
		input_sync(tpd->dev);
	}    
#ifdef  ELAN_ESD_PROTECT  
	//	elan_ic_power_state = 0;  
#endif
    }
    while (!kthread_should_stop());
    
    return 0;
}

static int ektf2136_get_fw_version_stored()
{
//edit by Magnum 2012-3-1
//    panel_version = tinno_ts_get_fw_version();
    return ektf_panel_version;
}

static int ektf2136_get_vendor_version_stored()
{
     return ektf_vendor_version;
}



static int tpd_detect(struct i2c_client *client, int kind, struct i2c_board_info *info)
{
    strcpy(info->type, tpd_driver_name);
    return 0;
}

		

static int __devinit tpd_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    TPD_DEBUG("[elan::] tpd_probe... \n");
    int retval = 0;
    unsigned char data[4] = {0}; 
   ektf2136_i2c_client = client;
    hwPowerOn(EKTF_POWER, VOL_2800, "TP");
    TPD_DEBUG("[elan::] tpd_probe  client->addr=0x%x, client->name=%s, client->flags=%d\n", client->addr, client->name, client->flags);

    mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);
    mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
    tpd_reset();

    mt_set_gpio_mode(GPIO_CTP_EINT_PIN, GPIO_CTP_EINT_PIN_M_EINT);
    mt_set_gpio_dir(GPIO_CTP_EINT_PIN, GPIO_DIR_IN);
    mt_set_gpio_pull_enable(GPIO_CTP_EINT_PIN, GPIO_PULL_ENABLE);
    mt_set_gpio_pull_select(GPIO_CTP_EINT_PIN, GPIO_PULL_UP);
    mt65xx_eint_set_sens(CUST_EINT_TOUCH_PANEL_NUM, CUST_EINT_TOUCH_PANEL_SENSITIVE);
    mt65xx_eint_set_hw_debounce(CUST_EINT_TOUCH_PANEL_NUM, CUST_EINT_TOUCH_PANEL_DEBOUNCE_CN);
    mt65xx_eint_registration(CUST_EINT_TOUCH_PANEL_NUM, CUST_EINT_TOUCH_PANEL_DEBOUNCE_EN, CUST_EINT_TOUCH_PANEL_POLARITY, tpd_eint_interrupt_handler, 1);
//Ivan    mt65xx_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);
    msleep(800);

	 //ic prepare ok ,send 55555555 to host. if upgrade err, send 55558080,go into recovery mode.
    if (i2c_master_recv(ektf2136_i2c_client, data, 4) <= 0){ 
        printk("I2C transfer error, line: %d\n", __LINE__);
        goto err_detect_failed;
    }else {
        TPD_DEBUG("[elan::] data[0]=%d, data[1]=%d, data[2]=%d, data[3]=%d\n", data[0], data[1], data[2], data[3]);
    }
    tpd_load_status = 1;

	ektf2136_ts_get_fw_version();

#ifdef      SUPPORT_UPDATE_FW
    s_update_thread = kthread_run(update_firmware_thread, 0, TPD_DEVICE);
    if (IS_ERR(s_update_thread)) {
        retval = PTR_ERR(s_update_thread);
        printk(TPD_DEVICE " failed to create update_firmware_thread thread: %d\n", retval);
    }
#endif

	
    thread = kthread_run(touch_event_handler, 0, TPD_DEVICE);

    if (IS_ERR(thread)) {
        retval = PTR_ERR(thread);
        printk(" failed to create kernel thread: %d\n", retval);
    }

#ifdef SW_FIRMWARE_UPDATE
	   	
	ektf2136_register_misc();		
#endif

		
#ifdef  ELAN_ESD_PROTECT
    INIT_DELAYED_WORK(&elan_esd_check_work, elan_esd_check);
  //  elan_esd_check_workqueue = create_singlethread_workqueue("elan_esd_check");
    elan_esd_check_workqueue = create_workqueue("elan_esd_check");
    queue_delayed_work(elan_esd_check_workqueue, &elan_esd_check_work, TPD_ESD_CHECK_CIRCLE);
#endif


    mt65xx_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);
    return 0;
err_detect_failed:
    printk("tpd_i2c_probe ERROR\n");
    ektf2136_i2c_client = NULL;
    //tinno_tp_power_off();
    hwPowerDown(EKTF_POWER, "TP");
   return 1;
}

static int __devexit tpd_remove(struct i2c_client *client)
{
    TPD_DEBUG("[elan::] TPD removed\n");
#ifdef ELAN_ESD_PROTECT
    destroy_workqueue(elan_esd_check_workqueue);
#endif
    return 0;
}
static int tpd_local_init(void)
{
    ektf_boot_mode = get_boot_mode();
    // Software reset mode will be treated as normal boot
    if(ektf_boot_mode==3) ektf_boot_mode = NORMAL_BOOT;
//Ivan
    printk("tpd_local_init boot mode = %d\n",ektf_boot_mode); 
    if(i2c_add_driver(&tpd_i2c_driver) != 0)
    {
        printk("[elan::] unable to add i2c driver.\n");
        return -1;
    }

	if(tpd_load_status == 0) 
    {
    	TPD_DMESG("ft5206 add error touch panel driver.\n");
    	i2c_del_driver(&tpd_i2c_driver);
    	return -1;
    }
	
#ifdef TPD_HAVE_BUTTON     
    if (FACTORY_BOOT == ektf_boot_mode)
    {
	    int i;
		for (i = 0; i < TPD_KEY_COUNT ; i++)
		    tpd_keys_local[i] = TPD_KEYSFACTORY[i];
    }   
    tpd_button_setting(TPD_KEY_COUNT, tpd_keys_local, tpd_keys_dim_local);// initialize tpd button data
#endif   
    printk("[elan::] end %s, %d\n", __FUNCTION__, __LINE__);
    return 0;
}
static int tpd_resume(struct i2c_client *client)
{
    //unsigned char sleep_out[] = {0x54, 0x58, 0x00, 0x01};
    int retval = 0;
	 if (ektf2136_i2c_client == NULL)
	return;
    #ifdef  ELAN_ESD_PROTECT
    queue_delayed_work(elan_esd_check_workqueue, &elan_esd_check_work, TPD_ESD_CHECK_CIRCLE);
	
#endif

	if(!ctp_suspend){
		TPD_DEBUG("ctp not suspend, resume depends on suspend...!\n");
		 return retval;
	}
    hwPowerOn(EKTF_POWER, VOL_2800, "TP");
    tpd_reset(); 
    msleep(10);    // solve i2c commication err... 
    mt65xx_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);
    //tpd_wr_datas(sleep_out, sizeof(sleep_out));
    TPD_DEBUG("[elan::] TPD wake up\n");
     TPD_DEBUG("[elan::] elan status == %d\n",elan_status);
    if(elan_status == 1)
		elan_status = 0;
/*    int i = 0; int err = -1;
	for(i;i<5;i++){
        err = ektf2136_ts_get_fw_version();
        if (err == 1)
            break;
	else{
            tpd_reset();
	    msleep(10); 
	}
    } */
	ctp_suspend = -1;
    return retval;
}
static int tpd_suspend(struct i2c_client *client, pm_message_t message)
{
     if (ektf2136_i2c_client == NULL)
	return;

#ifdef  ELAN_ESD_PROTECT
	 cancel_delayed_work_sync(&elan_esd_check_work);
#endif
	 
	if(work_lock) //tp is upgrading...
		return 0;
    unsigned char sleep_in[] = {0x54, 0x50, 0x00, 0x01};
    int retval = 0;    
    TPD_DEBUG("[elan::] enter sleep!!\n");
    tpd_wr_datas(sleep_in, sizeof(sleep_in));
	//edit by Magnum 2012-10-25 solve idle current > 3mA
    mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ZERO);
    mt65xx_eint_mask(CUST_EINT_TOUCH_PANEL_NUM);
    hwPowerDown(EKTF_POWER, "TP");
	ctp_suspend = 1;
    return retval;
}

static int __init tpd_driver_init(void)
{
    printk("[elan::] MediaTek elan touch panel driver init i2c = %d\n",TPD_I2C_NUMBER);
    i2c_register_board_info(TPD_I2C_NUMBER, &elan_i2c_tpd, 1);

    if(tpd_driver_add(&tpd_device_driver) < 0)
    {
        TPD_DEBUG("add elan driver failed\n");
    }
    return 0;
}
static void __exit tpd_driver_exit(void)
{
    printk("MediaTek elan touch panel driver exit\n");
    tpd_driver_remove(&tpd_device_driver);
}
module_init(tpd_driver_init);
module_exit(tpd_driver_exit);
MODULE_DESCRIPTION("CTP driver for elong on MT6589 platform");
MODULE_AUTHOR("tinno");
MODULE_LICENSE("GPL");
