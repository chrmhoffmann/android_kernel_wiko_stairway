/* 
 * Author: yucong xiong <yucong.xion@mediatek.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/kobject.h>
#include <linux/earlysuspend.h>
#include <linux/platform_device.h>
#include <asm/atomic.h>

//#include <mach/mt_devs.h>
#include <mach/mt_typedefs.h>
#include <mach/mt_gpio.h>
#include <mach/mt_pm_ldo.h>


#define POWER_NONE_MACRO MT65XX_POWER_NONE

#include <linux/hwmsensor.h>
#include <linux/hwmsen_dev.h>
#include <linux/sensors_io.h>
#include <asm/io.h>
#include <cust_eint.h>
#include <cust_alsps.h>
#include "ap3220.h"
#include <mach/mt_boot.h>

/******************************************************************************
 * configuration
*******************************************************************************/
/*----------------------------------------------------------------------------*/

#define AP3220_DEV_NAME     "ap3220"
/*----------------------------------------------------------------------------*/
#define APS_TAG                  "[ALS/PS] "
#define APS_FUN(f)               printk(KERN_INFO APS_TAG"%s\n", __FUNCTION__)
#define APS_ERR(fmt, args...)    printk(KERN_ERR  APS_TAG"%s %d : "fmt, __FUNCTION__, __LINE__, ##args)
#define APS_LOG(fmt, args...)    printk(KERN_ERR APS_TAG fmt, ##args)
#define APS_DBG(fmt, args...)    printk(KERN_INFO APS_TAG fmt, ##args)         

//Ivan
#define ALS_AUTO_RANGE
//#define ALS_RANGE_DEBUG		//Ivan

#define ALS_QUEUE_LEN	3

#define TINNO_PS_STARTUP_IRQ	//Ivan

//Ivan
#ifdef TINNO_PS_STARTUP_IRQ	
static struct delayed_work ps_startup_irq_work;
static struct workqueue_struct * ps_startup_irq_workqueue = NULL;
static void ps_startup_irq_func(struct work_struct *);
#define STARTUP_IRQ_DELAY 10
#endif

/******************************************************************************
* extern functions 
*******************************************************************************/

		extern void mt65xx_eint_unmask(unsigned int line);
		extern void mt65xx_eint_mask(unsigned int line);
		extern void mt65xx_eint_set_polarity(unsigned int eint_num, unsigned int pol);
		extern void mt65xx_eint_set_hw_debounce(unsigned int eint_num, unsigned int ms);
		extern unsigned int mt65xx_eint_set_sens(unsigned int eint_num, unsigned int sens);
		extern void mt65xx_eint_registration(unsigned int eint_num, unsigned int is_deb_en, unsigned int pol, void (EINT_FUNC_PTR)(void), unsigned int is_auto_umask);
		extern void mt_eint_soft_set(unsigned int eint_num);
		extern void mt_eint_soft_clr(unsigned int eint_num);

/*----------------------------------------------------------------------------*/
static int ap3220_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id); 
static int ap3220_i2c_remove(struct i2c_client *client);
static int ap3220_i2c_detect(struct i2c_client *client, struct i2c_board_info *info);
static int ap3220_i2c_suspend(struct i2c_client *client, pm_message_t msg);
static int ap3220_i2c_resume(struct i2c_client *client);

/*----------------------------------------------------------------------------*/
static const struct i2c_device_id ap3220_i2c_id[] = {{AP3220_DEV_NAME,0},{}};
static struct i2c_board_info __initdata i2c_ap3220={ I2C_BOARD_INFO(AP3220_DEV_NAME, 0x1C)};
/*----------------------------------------------------------------------------*/
struct ap3220_priv {
	struct alsps_hw  *hw;
	struct i2c_client *client;
	struct work_struct	eint_work;

	/*misc*/
	u16 		als_modulus;
	atomic_t	i2c_retry;
	atomic_t	als_suspend;
	atomic_t	als_debounce;	/*debounce time after enabling als*/
	atomic_t	als_deb_on; 	/*indicates if the debounce is on*/
	atomic_t	als_deb_end;	/*the jiffies representing the end of debounce*/
	atomic_t	ps_mask;		/*mask ps: always return far away*/
	atomic_t	ps_debounce;	/*debounce time after enabling ps*/
	atomic_t	ps_deb_on;		/*indicates if the debounce is on*/
	atomic_t	ps_deb_end; 	/*the jiffies representing the end of debounce*/
	atomic_t	ps_suspend;
	atomic_t 	trace;
	
	
	/*data*/
//Ivan	u8			als;
	u32			als;
	u32 			ps;
//	u8			_align;
	u32			als_level_num;
	u32			als_value_num;
	u32			als_level[C_CUST_ALS_LEVEL-1];
	u32			als_value[C_CUST_ALS_LEVEL];
	
	atomic_t	als_cmd_val;	/*the cmd value can't be read, stored in ram*/
	atomic_t	ps_cmd_val; 	/*the cmd value can't be read, stored in ram*/
	atomic_t	ps_thd_val_high;	 /*the cmd value can't be read, stored in ram*/
	atomic_t	ps_thd_val_low; 	/*the cmd value can't be read, stored in ram*/
	atomic_t	als_thd_val_high;	 /*the cmd value can't be read, stored in ram*/
	atomic_t	als_thd_val_low; 	/*the cmd value can't be read, stored in ram*/
	atomic_t	ps_thd_val;
	ulong		enable; 		/*enable mask*/
	ulong		pending_intr;	/*pending interrupt*/
	
	/*early suspend*/
	#if defined(CONFIG_HAS_EARLYSUSPEND)
	struct early_suspend	early_drv;
	#endif     
};
/*----------------------------------------------------------------------------*/

static struct i2c_driver ap3220_i2c_driver = {	
	.probe      = ap3220_i2c_probe,
	.remove     = ap3220_i2c_remove,
	.detect     = ap3220_i2c_detect,
	.suspend    = ap3220_i2c_suspend,
	.resume     = ap3220_i2c_resume,
	.id_table   = ap3220_i2c_id,
	.driver = {
		.name = AP3220_DEV_NAME,
	},
};

/*----------------------------------------------------------------------------*/
struct PS_CALI_DATA_STRUCT
{
	int close;
	int far_away;
	int valid;
};

/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
static struct i2c_client *ap3220_i2c_client = NULL;
static struct ap3220_priv *g_ap3220_ptr = NULL;
static struct ap3220_priv *ap3220_obj = NULL;
static struct platform_driver ap3220_alsps_driver;
static struct PS_CALI_DATA_STRUCT ps_cali={0,0,0};
//Ivan added
static u32 g_als_value_queue[ALS_QUEUE_LEN]={0,0,0};
static u32 g_als_value_previous = 0;
#ifdef ALS_AUTO_RANGE
static int g_als_range = AP3220_ALS_SETTING_RANGE_16383;
static u32 g_als_previous_data = 0xFFFF;
static u32 g_als_previous_level = 0xFFFF;
#endif
static u32 g_als_previous_value = 0xFFFF;
//Ivan added
#ifdef TINNO_PS_STARTUP_IRQ	
static int g_emu_int = 0;
static int g_first_int = 0;
#endif
#ifdef ALS_RANGE_DEBUG
int currentBootMode = 0;
#endif


/*----------------------------------------------------------------------------*/


/*----------------------------------------------------------------------------*/
typedef enum {
	CMC_BIT_ALS    = 1,
	CMC_BIT_PS	   = 2,
}CMC_BIT;
/*-----------------------------CMC for debugging-------------------------------*/
typedef enum {
    CMC_TRC_ALS_DATA= 0x0001,
    CMC_TRC_PS_DATA = 0x0002,
    CMC_TRC_EINT    = 0x0004,
    CMC_TRC_IOCTL   = 0x0008,
    CMC_TRC_I2C     = 0x0010,
    CMC_TRC_CVT_ALS = 0x0020,
    CMC_TRC_CVT_PS  = 0x0040,
    CMC_TRC_DEBUG   = 0x8000,
} CMC_TRC;


#ifdef TINNO_PS_STARTUP_IRQ	
static void ps_startup_irq_func(struct work_struct *work)
{   
    if (g_emu_int == 0)
    {
	mt_eint_soft_set(CUST_EINT_ALS_NUM);
	APS_ERR("debug ps_startup_irq_func!");	
	g_emu_int = 1;
    }
}
#endif
/* 
 * #########
 * ## I2C ##
 * #########
 */

// I2C Read
static int ap3220_i2c_read_reg(u8 regnum, u8* data)
{
    u8 buffer[2],reg_value[2];
	int res = 0;
	
	buffer[0]= regnum;
	ap3220_i2c_client->addr &=I2C_MASK_FLAG;
	ap3220_i2c_client->addr |=I2C_WR_FLAG;
	ap3220_i2c_client->addr |=I2C_RS_FLAG;
	res = i2c_master_send(ap3220_i2c_client, buffer, 0x101);
	ap3220_i2c_client->addr &=I2C_MASK_FLAG;
	
//	res = i2c_master_send(ap3220_i2c_client, buffer, 0x1);
	if(res < 0)
	{
		APS_ERR("ap3220_i2c_read_reg ERROR!!!\n");	    
		return res;
	}
//	res = i2c_master_recv(ap3220_i2c_client, reg_value, 0x1);
//	if(res <= 0)
//	{
//		return res;
//	}
	*data = buffer[0];
	return buffer[0];
}

// I2C Write
static int ap3220_i2c_write_reg(u8 regnum, u8 value)
{
	u8 databuf[2];    
	int res = 0;
   
	databuf[0] = regnum;   
	databuf[1] = value;
	
	ap3220_i2c_client->addr &=I2C_MASK_FLAG;
	res = i2c_master_send(ap3220_i2c_client, databuf, 0x2);

	if (res < 0)
	{
		APS_ERR("ap3220_i2c_write_reg ERROR!!!\n");	    	    
		return res;
	}
	else
		return 0;
}

/*----------------------------------------------------------------------------*/
static void ap3220_power(struct alsps_hw *hw, unsigned int on) 
{
	static unsigned int power_on = 0;

	APS_LOG("power %s\n", on ? "on" : "off");

	if(hw->power_id != POWER_NONE_MACRO)
	{
		if(power_on == on)
		{
			APS_LOG("ignore power control: %d\n", on);
		}
		else if(on)
		{
			if(!hwPowerOn(hw->power_id, hw->power_vol, "AP3220")) 
			{
				APS_ERR("power on fails!!\n");
			}
		}
		else
		{
			if(!hwPowerDown(hw->power_id, "AP3220")) 
			{
				APS_ERR("power off fail!!\n");   
			}
		}
	}
	power_on = on;
}
/********************************************************************/
int ap3220_enable_ps(struct i2c_client *client, int enable)
{
	struct ap3220_priv *obj = i2c_get_clientdata(client);
	int res;
	u8 databuf[1];

	if(enable == 1)
		{
			APS_LOG("ap3220_enable_ps enable_ps\n");
			res = ap3220_i2c_read_reg(AP3220_REG_SYS_CONF,&databuf[0]);
			if(res < 0)
			{
				APS_ERR("i2c_master_read function err\n");
				goto ENABLE_PS_EXIT_ERR;
			}			
			databuf[0] |= AP3220_SYSTEM_PS_ENABLE;
			databuf[0] &= AP3220_SYSTEM_DEVICE_MASK;
			res = ap3220_i2c_write_reg(AP3220_REG_SYS_CONF,databuf[0]);
			if(res < 0)
			{
				APS_ERR("i2c_master_send function err\n");
				goto ENABLE_PS_EXIT_ERR;
			}
			
			atomic_set(&obj->ps_deb_on, 1);
			atomic_set(&obj->ps_deb_end, jiffies+atomic_read(&obj->ps_debounce)/(1000/HZ));
#ifdef TINNO_PS_STARTUP_IRQ						
			g_first_int = 0;
#endif			
		}
	else{
			APS_LOG("ap3220_enable_ps disable_ps\n");
			res = ap3220_i2c_read_reg(AP3220_REG_SYS_CONF,&databuf[0]);
			if(res < 0)
			{
				APS_ERR("i2c_master_read function err\n");
				goto ENABLE_PS_EXIT_ERR;
			}			
			databuf[0] &= ~AP3220_SYSTEM_PS_ENABLE;
			databuf[0] &= AP3220_SYSTEM_DEVICE_MASK;			
			res = ap3220_i2c_write_reg(AP3220_REG_SYS_CONF,databuf[0]);
			if(res < 0)
			{
				APS_ERR("i2c_master_send function err\n");
				goto ENABLE_PS_EXIT_ERR;
			}
			atomic_set(&obj->ps_deb_on, 0);
		}
	
	return 0;
	ENABLE_PS_EXIT_ERR:
	return res;
}
/********************************************************************/
int ap3220_enable_als(struct i2c_client *client, int enable)
{
	struct ap3220_priv *obj = i2c_get_clientdata(client);
	int res;
	u8 databuf[3];
	client->addr &=I2C_MASK_FLAG;
	client->addr |=I2C_WR_FLAG;
	client->addr |=I2C_RS_FLAG;
	if(enable == 1)
		{
			APS_LOG("ap3220_enable_als enable_als\n");
			res = ap3220_i2c_read_reg(AP3220_REG_SYS_CONF,&databuf[0]);
			APS_LOG("ap3220_enable_als SYS_CONF = %x\n",databuf[0]);			
			if(res < 0)
			{
				APS_ERR("i2c_master_read function err\n");
				goto ENABLE_ALS_EXIT_ERR;
			}			
			databuf[0] |= AP3220_SYSTEM_ALS_ENABLE;
			databuf[0] &= AP3220_SYSTEM_DEVICE_MASK;
			res = ap3220_i2c_write_reg(AP3220_REG_SYS_CONF,databuf[0]);
			if(res < 0)
			{
				APS_ERR("i2c_master_send function err\n");
				goto ENABLE_ALS_EXIT_ERR;
			}
			atomic_set(&obj->als_deb_on, 0);
			atomic_set(&obj->als_deb_end, jiffies+atomic_read(&obj->als_debounce)/(1000/HZ));
		}
	else{
			APS_LOG("ap3220_enable_als disable_als\n");
			res = ap3220_i2c_read_reg(AP3220_REG_SYS_CONF,&databuf[0]);
			if(res < 0)
			{
				APS_ERR("i2c_master_read function err\n");
				goto ENABLE_ALS_EXIT_ERR;
			}		
			APS_LOG("ap3220_enable_als SYS_CONF = %x\n",databuf[0]);
			databuf[0] &= ~AP3220_SYSTEM_ALS_ENABLE;
			databuf[0] &= AP3220_SYSTEM_DEVICE_MASK;			
			res = ap3220_i2c_write_reg(AP3220_REG_SYS_CONF,databuf[0]);
			if(res < 0)
			{
				APS_ERR("i2c_master_send function err\n");
				goto ENABLE_ALS_EXIT_ERR;
			}
			atomic_set(&obj->als_deb_on, 0);
		}
	return 0;
	ENABLE_ALS_EXIT_ERR:
	return res;
}
/********************************************************************/
long ap3220_read_ps(struct i2c_client *client, u32 *data)
{
	long res;
	u8 databuf[2];
	u8 ps_l, ps_h;
	u32 ps_val;
	
//	APS_FUN(f);
	res = ap3220_i2c_read_reg(AP3220_REG_SYS_PS_DATA_LOW,&databuf[0]);
	res = ap3220_i2c_read_reg(AP3220_REG_SYS_PS_DATA_HIGH,&databuf[1]);
	if(res < 0)
	{
		APS_ERR("i2c_master_send function err\n");
		goto READ_PS_EXIT_ERR;
	}
	
//	APS_LOG("AP3220_REG_PS_DATA value value_low = %x, value_high = %x\n",databuf[0],databuf[1]);
	*data = ((databuf[1]<<8)|databuf[0]);
	
	ps_l = *data & 0x00FF;
	ps_h = *data >> 8;
	ps_val = ps_h << 2 | ps_l & 0x03;
//Ivan
	printk("ALSPS Ivan ps_val = %x \n",ps_val);
	
	APS_LOG("ALSPS Ivan AP3220_REG_PS_DATA value %d\n",*data);	
	return 0;
READ_PS_EXIT_ERR:
	return res;
}


/********************************************************************/
long ap3220_read_als(struct i2c_client *client, u32 *value)
{
	long res;
	u8 databuf[2];
#ifdef ALS_AUTO_RANGE	
	u32 temp_data,temp_data2;
	u8 data, data2, data3;
#endif
//	APS_FUN(f);
	
	res = ap3220_i2c_read_reg(AP3220_REG_SYS_ALS_DATA_LOW,&databuf[0]);
	res = ap3220_i2c_read_reg(AP3220_REG_SYS_ALS_DATA_HIGH,&databuf[1]);
	if(res < 0)
	{
		APS_ERR("i2c_master_send function err\n");
		goto READ_ALS_EXIT_ERR;
	}
	
//	APS_LOG("AP3220_REG_ALS_DATA value value_low = %x, value_high = %x\n",databuf[0],databuf[1]);
#ifdef ALS_AUTO_RANGE
	temp_data = ((databuf[1]<<8)|databuf[0]);
	if (temp_data < 0x100 && g_als_range == AP3220_ALS_SETTING_RANGE_16383)		//Ivan Switch sensor range...get better resolution on low light condition
	{
	    data = (AP3220_ALS_SETTING_RANGE_4095 << AP3220_ALS_RANGE_SHIFT) & AP3220_ALS_RANGE_MASK;
	    data2 = (AP3220_ALS_SETTING_PERSIST_1 << AP3220_ALS_PERSIST_SHIFT) & AP3220_ALS_PERSIST_MASK;
	    data |= data2;
	    res = ap3220_i2c_write_reg(AP3220_REG_ALS_CONF,data);
	    
	    g_als_range = AP3220_ALS_SETTING_RANGE_4095;
	    
	    if (g_als_previous_data != 0xFFFF)		//Ivan use previous data if switching range
		*value = g_als_previous_data;
	    else
		*value = temp_data;
	}
	else if (temp_data > 0xD00 && g_als_range > AP3220_ALS_SETTING_RANGE_16383)	//Range order <
	{
	    data = (AP3220_ALS_SETTING_RANGE_16383 << AP3220_ALS_RANGE_SHIFT) & AP3220_ALS_RANGE_MASK;
	    data2 = (AP3220_ALS_SETTING_PERSIST_1 << AP3220_ALS_PERSIST_SHIFT) & AP3220_ALS_PERSIST_MASK;
	    data |= data2;
	    res = ap3220_i2c_write_reg(AP3220_REG_ALS_CONF,data);
	    
	    g_als_range = AP3220_ALS_SETTING_RANGE_16383;
	    
	    if (g_als_previous_data != 0xFFFF)		//Ivan use previous data if switching range
		*value = g_als_previous_data;	    
	    else
		*value = temp_data;	    
	}
	else
	{
	    if (g_als_range == AP3220_ALS_SETTING_RANGE_4095)
	    {
		temp_data2 = temp_data >> 2;
		if (temp_data >= 0 && temp_data < 0x0A)
		{
//		    if (temp_data2 == 0)
			temp_data2 = 0;
		}
		else if (temp_data >= 0xA && temp_data < 0x20)		//Keep previous value
		    temp_data2 = g_als_previous_data;
		
		*value = temp_data2;
	    }
	    else
		*value = temp_data;
	}
#else
	*value = ((databuf[1]<<8)|databuf[0]);
#endif
	
#ifdef ALS_AUTO_RANGE
#ifdef ALS_RANGE_DEBUG
	if (FACTORY_BOOT == currentBootMode)
	{
	    *value = ((databuf[1]<<8)|databuf[0]);
	    *value += g_als_range << 28;
	}
	    
#endif
//	APS_LOG("AP3220_REG_ALS_DATA value %d\n",*value);
	g_als_previous_data = *value;
#endif
	return 0;
READ_ALS_EXIT_ERR:
	return res;
}
/********************************************************************/
static int ap3220_get_ps_value(struct ap3220_priv *obj, u32 ps)
{
	int val, mask = atomic_read(&obj->ps_mask);
	int invalid = 0;
	u8 ps_l, ps_h;
	u32 ps_val;
	
	ps_l = ps & 0x00FF;
	ps_h = ps >> 8;
	ps_val = ps_h << 2 | ps_l & 0x03;
	if(ps_val > atomic_read(&obj->ps_thd_val_high))
	{
		val = 0;  /*close*/
	}
	else if(ps_val < atomic_read(&obj->ps_thd_val_low))
	{
		val = 1;  /*far away*/
	}
	
	if(atomic_read(&obj->ps_suspend))
	{
		invalid = 1;
	}
	else if(1 == atomic_read(&obj->ps_deb_on))
	{
		unsigned long endt = atomic_read(&obj->ps_deb_end);
		if(time_after(jiffies, endt))
		{
			atomic_set(&obj->ps_deb_on, 0);
		}
		
		if (1 == atomic_read(&obj->ps_deb_on))
		{
			invalid = 1;
		}
	}

	if(!invalid)
	{
		if(unlikely(atomic_read(&obj->trace) & CMC_TRC_CVT_PS))
		{
			if(mask)
			{
				APS_DBG("PS:  %05d => %05d [M] \n", ps, val);
			}
			else
			{
				APS_DBG("PS:  %05d => %05d\n", ps, val);
			}
		}
		if(0 == test_bit(CMC_BIT_PS,  &obj->enable))
		{
		  //if ps is disable do not report value
		  APS_DBG("PS: not enable and do not report this value\n");
		  return -1;
		}
		else
		{
		   return val;
		}
		
	}	
	else
	{
		if(unlikely(atomic_read(&obj->trace) & CMC_TRC_CVT_PS))
		{
			APS_DBG("PS:  %05d => %05d (-1)\n", ps, val);    
		}
		return -1;
	}	
}
/********************************************************************/
static int ap3220_get_als_value(struct ap3220_priv *obj, u32 als)
{
		int idx;
		int invalid = 0;
		u32 als_sum;
		u32 als_level1 = 0;
		u32 als_level2 = 0;
//Ivan added SW LOW PASS FILTER
		als_sum = 0;
		
		
		for (idx = 0; idx < (ALS_QUEUE_LEN - 1); idx++)
		    g_als_value_queue[idx] = g_als_value_queue[idx+1];
		g_als_value_queue[ALS_QUEUE_LEN-1] = als;
		
		for (idx = 0; idx < ALS_QUEUE_LEN; idx++)
		    als_sum+= g_als_value_queue[idx];
		
		if (als_sum > 0 && als_sum <3)
		    als_sum = 3;
		else
		    als_sum /= 3;
		
//		APS_LOG("AP3220_REG_ALS_DATA sum value %d\n",als_sum);
		
		
		for(idx = 0; idx < obj->als_level_num; idx++)
		{
			if(als_sum < obj->hw->als_level[idx])
			{
				break;
			}
		}
		if(idx >= obj->als_value_num)
		{
			APS_ERR("exceed range\n"); 
			idx = obj->als_value_num - 1;
		}
//Ivan added threshold to keep level stable
		if (idx > 0 && g_als_previous_value != 0xFFFF)
		{
		    if (idx > g_als_previous_value)
		    {
			als_level1 = obj->hw->als_level[idx];
			als_level2 = obj->hw->als_level[idx+1];
			als_level1 = (als_level1 + als_level2)/2;
			if (als_sum < als_level1)
			    idx--;
		    }
		    if (idx < g_als_previous_value)
		    {
			als_level1 = obj->hw->als_level[idx];
			als_level2 = obj->hw->als_level[idx-1];
			als_level1 = (als_level1 + als_level2)/2;
			if (als_sum > als_level1)
			    idx++;			
		    }
		}
		
		g_als_previous_value = idx;
		
		if(1 == atomic_read(&obj->als_deb_on))
		{
			unsigned long endt = atomic_read(&obj->als_deb_end);
			if(time_after(jiffies, endt))
			{
				atomic_set(&obj->als_deb_on, 0);
			}
			
			if(1 == atomic_read(&obj->als_deb_on))
			{
				invalid = 1;
			}
		}
	
		if(!invalid)
		{
			if (atomic_read(&obj->trace) & CMC_TRC_CVT_ALS)
			{
				APS_DBG("ALS: %05d => %05d\n", als, obj->hw->als_value[idx]);
			}
			return obj->hw->als_value[idx];
//			return als/10;			
		}
		else
		{
			if(atomic_read(&obj->trace) & CMC_TRC_CVT_ALS)
			{
				APS_DBG("ALS: %05d => %05d (-1)\n", als, obj->hw->als_value[idx]);	  
			}
			return -1;
		}

}


/*-------------------------------attribute file for debugging----------------------------------*/

/******************************************************************************
 * Sysfs attributes
*******************************************************************************/
static ssize_t ap3220_show_config(struct device_driver *ddri, char *buf)
{
	ssize_t res;
	
	if(!ap3220_obj)
	{
		APS_ERR("ap3220_obj is null!!\n");
		return 0;
	}
	
	res = snprintf(buf, PAGE_SIZE, "(%d %d %d %d %d)\n", 
		atomic_read(&ap3220_obj->i2c_retry), atomic_read(&ap3220_obj->als_debounce), 
		atomic_read(&ap3220_obj->ps_mask), atomic_read(&ap3220_obj->ps_thd_val), atomic_read(&ap3220_obj->ps_debounce));     
	return res;    
}
/*----------------------------------------------------------------------------*/
static ssize_t ap3220_store_config(struct device_driver *ddri, const char *buf, size_t count)
{
	int retry, als_deb, ps_deb, mask, thres;
	if(!ap3220_obj)
	{
		APS_ERR("ap3220_obj is null!!\n");
		return 0;
	}
	
	if(5 == sscanf(buf, "%d %d %d %d %d", &retry, &als_deb, &mask, &thres, &ps_deb))
	{ 
		atomic_set(&ap3220_obj->i2c_retry, retry);
		atomic_set(&ap3220_obj->als_debounce, als_deb);
		atomic_set(&ap3220_obj->ps_mask, mask);
		atomic_set(&ap3220_obj->ps_thd_val, thres);        
		atomic_set(&ap3220_obj->ps_debounce, ps_deb);
	}
	else
	{
		APS_ERR("invalid content: '%s', length = %d\n", buf, count);
	}
	return count;    
}
/*----------------------------------------------------------------------------*/
static ssize_t ap3220_show_trace(struct device_driver *ddri, char *buf)
{
	ssize_t res;
	if(!ap3220_obj)
	{
		APS_ERR("ap3220_obj is null!!\n");
		return 0;
	}

	res = snprintf(buf, PAGE_SIZE, "0x%04X\n", atomic_read(&ap3220_obj->trace));     
	return res;    
}
/*----------------------------------------------------------------------------*/
static ssize_t ap3220_store_trace(struct device_driver *ddri, const char *buf, size_t count)
{
    int trace;
    if(!ap3220_obj)
	{
		APS_ERR("ap3220_obj is null!!\n");
		return 0;
	}
	
	if(1 == sscanf(buf, "0x%x", &trace))
	{
		atomic_set(&ap3220_obj->trace, trace);
	}
	else 
	{
		APS_ERR("invalid content: '%s', length = %d\n", buf, count);
	}
	return count;    
}
/*----------------------------------------------------------------------------*/
static ssize_t ap3220_show_als(struct device_driver *ddri, char *buf)
{
	int res;
	
	if(!ap3220_obj)
	{
		APS_ERR("ap3220_obj is null!!\n");
		return 0;
	}
	if((res = ap3220_read_als(ap3220_obj->client, &ap3220_obj->als)))
	{
		return snprintf(buf, PAGE_SIZE, "ERROR: %d\n", res);
	}
	else
	{
		return snprintf(buf, PAGE_SIZE, "0x%04X\n", ap3220_obj->als);     
	}
}
/*----------------------------------------------------------------------------*/
static ssize_t ap3220_show_ps(struct device_driver *ddri, char *buf)
{
	ssize_t res;
	if(!ap3220_obj)
	{
		APS_ERR("cm3623_obj is null!!\n");
		return 0;
	}
	
	if((res = ap3220_read_ps(ap3220_obj->client, &ap3220_obj->ps)))
	{
		return snprintf(buf, PAGE_SIZE, "ERROR: %d\n", res);
	}
	else
	{
		return snprintf(buf, PAGE_SIZE, "0x%04X\n", ap3220_obj->ps);     
	}
}
/*----------------------------------------------------------------------------*/
static ssize_t ap3220_show_reg(struct device_driver *ddri, char *buf)
{
	if(!ap3220_obj)
	{
		APS_ERR("ap3220_obj is null!!\n");
		return 0;
	}
	
	
	return 0;
}
/*----------------------------------------------------------------------------*/
static ssize_t ap3220_show_send(struct device_driver *ddri, char *buf)
{
    return 0;
}
/*----------------------------------------------------------------------------*/
static ssize_t ap3220_store_send(struct device_driver *ddri, const char *buf, size_t count)
{
	int addr, cmd;
	u8 dat;

	if(!ap3220_obj)
	{
		APS_ERR("ap3220_obj is null!!\n");
		return 0;
	}
	else if(2 != sscanf(buf, "%x %x", &addr, &cmd))
	{
		APS_ERR("invalid format: '%s'\n", buf);
		return 0;
	}

	dat = (u8)cmd;
	//****************************
	return count;
}
/*----------------------------------------------------------------------------*/
static ssize_t ap3220_show_recv(struct device_driver *ddri, char *buf)
{
    return 0;
}
/*----------------------------------------------------------------------------*/
static ssize_t ap3220_store_recv(struct device_driver *ddri, const char *buf, size_t count)
{
	int addr;
	u8 dat;
	if(!ap3220_obj)
	{
		APS_ERR("ap3220_obj is null!!\n");
		return 0;
	}
	else if(1 != sscanf(buf, "%x", &addr))
	{
		APS_ERR("invalid format: '%s'\n", buf);
		return 0;
	}

	//****************************
	return count;
}
/*----------------------------------------------------------------------------*/
static ssize_t ap3220_show_status(struct device_driver *ddri, char *buf)
{
	ssize_t len = 0;
	
	if(!ap3220_obj)
	{
		APS_ERR("ap3220_obj is null!!\n");
		return 0;
	}
	
	if(ap3220_obj->hw)
	{
		len += snprintf(buf+len, PAGE_SIZE-len, "CUST: %d, (%d %d)\n", 
			ap3220_obj->hw->i2c_num, ap3220_obj->hw->power_id, ap3220_obj->hw->power_vol);
	}
	else
	{
		len += snprintf(buf+len, PAGE_SIZE-len, "CUST: NULL\n");
	}
	
	len += snprintf(buf+len, PAGE_SIZE-len, "REGS: %02X %02X %02X %02lX %02lX\n", 
				atomic_read(&ap3220_obj->als_cmd_val), atomic_read(&ap3220_obj->ps_cmd_val), 
				atomic_read(&ap3220_obj->ps_thd_val),ap3220_obj->enable, ap3220_obj->pending_intr);
	
	len += snprintf(buf+len, PAGE_SIZE-len, "MISC: %d %d\n", atomic_read(&ap3220_obj->als_suspend), atomic_read(&ap3220_obj->ps_suspend));

	return len;
}
/*----------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/
#define IS_SPACE(CH) (((CH) == ' ') || ((CH) == '\n'))
/*----------------------------------------------------------------------------*/
static int read_int_from_buf(struct ap3220_priv *obj, const char* buf, size_t count, u32 data[], int len)
{
	int idx = 0;
	char *cur = (char*)buf, *end = (char*)(buf+count);

	while(idx < len)
	{
		while((cur < end) && IS_SPACE(*cur))
		{
			cur++;        
		}

		if(1 != sscanf(cur, "%d", &data[idx]))
		{
			break;
		}

		idx++; 
		while((cur < end) && !IS_SPACE(*cur))
		{
			cur++;
		}
	}
	return idx;
}
/*----------------------------------------------------------------------------*/
static ssize_t ap3220_show_alslv(struct device_driver *ddri, char *buf)
{
	ssize_t len = 0;
	int idx;
	if(!ap3220_obj)
	{
		APS_ERR("ap3220_obj is null!!\n");
		return 0;
	}
	
	for(idx = 0; idx < ap3220_obj->als_level_num; idx++)
	{
		len += snprintf(buf+len, PAGE_SIZE-len, "%d ", ap3220_obj->hw->als_level[idx]);
	}
	len += snprintf(buf+len, PAGE_SIZE-len, "\n");
	return len;    
}
/*----------------------------------------------------------------------------*/
static ssize_t ap3220_store_alslv(struct device_driver *ddri, const char *buf, size_t count)
{
	if(!ap3220_obj)
	{
		APS_ERR("ap3220_obj is null!!\n");
		return 0;
	}
	else if(!strcmp(buf, "def"))
	{
		memcpy(ap3220_obj->als_level, ap3220_obj->hw->als_level, sizeof(ap3220_obj->als_level));
	}
	else if(ap3220_obj->als_level_num != read_int_from_buf(ap3220_obj, buf, count, 
			ap3220_obj->hw->als_level, ap3220_obj->als_level_num))
	{
		APS_ERR("invalid format: '%s'\n", buf);
	}    
	return count;
}
/*----------------------------------------------------------------------------*/
static ssize_t ap3220_show_alsval(struct device_driver *ddri, char *buf)
{
	ssize_t len = 0;
	int idx;
	if(!ap3220_obj)
	{
		APS_ERR("ap3220_obj is null!!\n");
		return 0;
	}
	
	for(idx = 0; idx < ap3220_obj->als_value_num; idx++)
	{
		len += snprintf(buf+len, PAGE_SIZE-len, "%d ", ap3220_obj->hw->als_value[idx]);
	}
	len += snprintf(buf+len, PAGE_SIZE-len, "\n");
	return len;    
}
/*----------------------------------------------------------------------------*/
static ssize_t ap3220_store_alsval(struct device_driver *ddri, const char *buf, size_t count)
{
	if(!ap3220_obj)
	{
		APS_ERR("ap3220_obj is null!!\n");
		return 0;
	}
	else if(!strcmp(buf, "def"))
	{
		memcpy(ap3220_obj->als_value, ap3220_obj->hw->als_value, sizeof(ap3220_obj->als_value));
	}
	else if(ap3220_obj->als_value_num != read_int_from_buf(ap3220_obj, buf, count, 
			ap3220_obj->hw->als_value, ap3220_obj->als_value_num))
	{
		APS_ERR("invalid format: '%s'\n", buf);
	}    
	return count;
}
/*---------------------------------------------------------------------------------------*/
static DRIVER_ATTR(als,     S_IWUSR | S_IRUGO, ap3220_show_als, NULL);
static DRIVER_ATTR(ps,      S_IWUSR | S_IRUGO, ap3220_show_ps, NULL);
static DRIVER_ATTR(config,  S_IWUSR | S_IRUGO, ap3220_show_config,	ap3220_store_config);
static DRIVER_ATTR(alslv,   S_IWUSR | S_IRUGO, NULL, NULL);
static DRIVER_ATTR(alsval,  S_IWUSR | S_IRUGO, NULL, NULL);
static DRIVER_ATTR(trace,   S_IWUSR | S_IRUGO, ap3220_show_trace,		ap3220_store_trace);
static DRIVER_ATTR(status,  S_IWUSR | S_IRUGO, NULL, NULL);
static DRIVER_ATTR(send,    S_IWUSR | S_IRUGO, NULL, NULL);
static DRIVER_ATTR(recv,    S_IWUSR | S_IRUGO, NULL, NULL);
static DRIVER_ATTR(reg,     S_IWUSR | S_IRUGO, ap3220_show_reg, NULL);
/*----------------------------------------------------------------------------*/
static struct driver_attribute *ap3220_attr_list[] = {
    &driver_attr_als,
    &driver_attr_ps,    
    &driver_attr_trace,        /*trace log*/
    &driver_attr_config,
    &driver_attr_alslv,
    &driver_attr_alsval,
    &driver_attr_status,
    &driver_attr_send,
    &driver_attr_recv,
    &driver_attr_reg,
};

/*----------------------------------------------------------------------------*/
static int ap3220_create_attr(struct device_driver *driver) 
{
	int idx, err = 0;
	int num = (int)(sizeof(ap3220_attr_list)/sizeof(ap3220_attr_list[0]));
	if (driver == NULL)
	{
		return -EINVAL;
	}

	for(idx = 0; idx < num; idx++)
	{
		if((err = driver_create_file(driver, ap3220_attr_list[idx])))
		{            
			APS_ERR("driver_create_file (%s) = %d\n", ap3220_attr_list[idx]->attr.name, err);
			break;
		}
	}    
	return err;
}
/*----------------------------------------------------------------------------*/
	static int ap3220_delete_attr(struct device_driver *driver)
	{
	int idx ,err = 0;
	int num = (int)(sizeof(ap3220_attr_list)/sizeof(ap3220_attr_list[0]));

	if (!driver)
	return -EINVAL;

	for (idx = 0; idx < num; idx++) 
	{
		driver_remove_file(driver, ap3220_attr_list[idx]);
	}
	
	return err;
}
/*----------------------------------------------------------------------------*/


/*----------------------------------interrupt functions--------------------------------*/
static int intr_flag = 0;
/*----------------------------------------------------------------------------*/
static int ap3220_check_intr(struct i2c_client *client) 
{
	int res;
	u8 databuf[2];
	u8 intr_status;
	u8 intr;
	U8 ret = 0;
	
	ret= 0;
	res = ap3220_i2c_read_reg(AP3220_REG_SYS_ISTATUS,&intr_status);
	if(res < 0)
	{
		APS_ERR("i2c_master_send function err\n");
		goto EXIT_ERR;
	}
//	APS_LOG("AP3220_REG_INT_FLAG value = %x\n",intr_status);	
//Ivan ALS INT should not happen!!! Just in case...we need to clear the INT pin		
	if (intr_status & AP3220_SYSTEM_ALS_INT_TRIGGER)
	{
	    res = ap3220_i2c_read_reg(AP3220_REG_SYS_ALS_DATA_LOW,&databuf[0]);
	    res = ap3220_i2c_read_reg(AP3220_REG_SYS_ALS_DATA_HIGH,&databuf[1]);
	    APS_ERR("Ivan ap3220_check_intr ALS = %d \n", databuf[1] << 8 | databuf[0]);	    
	    if(res < 0)
	    {
		    APS_ERR("i2c_master_send function err\n");
//		    goto EXIT_ERR;
	    }
	    
//thrown away the data????	    
	}	
	if (intr_status & AP3220_SYSTEM_PS_INT_TRIGGER)
	{
	    res = ap3220_i2c_read_reg(AP3220_REG_SYS_PS_DATA_LOW,&databuf[0]);
	    res = ap3220_i2c_read_reg(AP3220_REG_SYS_PS_DATA_HIGH,&databuf[1]);
//To be removed	    
//	    APS_ERR("Ivan ap3220_check_intr PS0= %x \n", databuf[0]);
//	    APS_ERR("Ivan ap3220_check_intr PS1 = %x \n", databuf[1]);
	    
	    if(res < 0)
	    {
		    APS_ERR("i2c_master_send function err\n");
		    goto EXIT_ERR;
	    }
	    if (!(databuf[0] & AP3220_SYSTEM_PS_IR_OVERFLOW) && !(databuf[1] & AP3220_SYSTEM_PS_IR_OVERFLOW))
	    {
		if (databuf[0] & AP3220_SYSTEM_PS_OBJ_CLOSE)
		    intr_flag = 0;//for close
		else
		    intr_flag = 1;//for away
	    }
	    else
	    {
		intr_flag = 1;//for away
//		res = -1;		
//		goto EXIT_ERR;
	    }
	    ret = 1;
	}

//	APS_LOG("AP3220_REG_INT_FLAG value value_low = %x, value_high = %x\n",databuf[0],databuf[1]);
	
	return ret;
EXIT_ERR:
	APS_ERR("ap3220_check_intr dev: %d\n", res);
	return -1;
}
/*----------------------------------------------------------------------------*/
static void ap3220_eint_work(struct work_struct *work)
{
	struct ap3220_priv *obj = (struct ap3220_priv *)container_of(work, struct ap3220_priv, eint_work);
	hwm_sensor_data sensor_data;
	int res = 0;
	//res = ap3220_check_intr(obj->client);

#if 1
	res = ap3220_check_intr(obj->client);
	if(res < 0){
		goto EXIT_INTR_ERR;
	}else{
		sensor_data.values[0] = intr_flag;
		sensor_data.value_divide = 1;
		sensor_data.status = SENSOR_STATUS_ACCURACY_MEDIUM;	
#ifdef TINNO_PS_STARTUP_IRQ			
		if (g_emu_int)
		{
		    g_emu_int = 0;
		    mt_eint_soft_clr(CUST_EINT_ALS_NUM);
		    if (g_first_int == 0)
		    {
			intr_flag = 1;
			sensor_data.values[0] = intr_flag;
		    }
		    g_first_int = 1;
		}
		if (res >= 1)
		    g_first_int = 1;
#endif		
	}
	APS_LOG("AP3220_eint_work intr_flag = %x\n",intr_flag);		
	if((res = hwmsen_get_interrupt_data(ID_PROXIMITY, &sensor_data)))
		{
		  APS_ERR("call hwmsen_get_interrupt_data fail = %d\n", res);
		  goto EXIT_INTR_ERR;
		}
#endif
	mt65xx_eint_unmask(CUST_EINT_ALS_NUM);
	return;
	EXIT_INTR_ERR:
	mt65xx_eint_unmask(CUST_EINT_ALS_NUM);
	APS_ERR("ap3220_eint_work err: %d\n", res);
}
/*----------------------------------------------------------------------------*/
static void ap3220_eint_func(void)
{
	struct ap3220_priv *obj = g_ap3220_ptr;
	if(!obj)
	{
		return;
	}	
//	APS_ERR("debug ap3220_eint_func!");
//Ivan	
	if (g_emu_int == 1)
	    mt65xx_eint_mask(CUST_EINT_ALS_NUM);	
	schedule_work(&obj->eint_work);
}

int ap3220_setup_eint(struct i2c_client *client)
{
	struct ap3220_priv *obj = i2c_get_clientdata(client);        

	g_ap3220_ptr = obj;
	
	mt_set_gpio_dir(GPIO_ALS_EINT_PIN, GPIO_DIR_IN);
	mt_set_gpio_mode(GPIO_ALS_EINT_PIN, GPIO_ALS_EINT_PIN_M_EINT);
	mt_set_gpio_pull_enable(GPIO_ALS_EINT_PIN, TRUE);
	mt_set_gpio_pull_select(GPIO_ALS_EINT_PIN, GPIO_PULL_UP);

	mt65xx_eint_set_sens(CUST_EINT_ALS_NUM, CUST_EINT_ALS_SENSITIVE);
	mt65xx_eint_set_polarity(CUST_EINT_ALS_NUM, CUST_EINT_ALS_POLARITY);
	mt65xx_eint_set_hw_debounce(CUST_EINT_ALS_NUM, CUST_EINT_ALS_DEBOUNCE_CN);
	mt65xx_eint_registration(CUST_EINT_ALS_NUM, CUST_EINT_ALS_DEBOUNCE_EN, CUST_EINT_ALS_POLARITY, ap3220_eint_func, 0);

	mt65xx_eint_unmask(CUST_EINT_ALS_NUM);  
    return 0;
}
/*-------------------------------MISC device related------------------------------------------*/



/************************************************************/
static int ap3220_open(struct inode *inode, struct file *file)
{
	file->private_data = ap3220_i2c_client;

	if (!file->private_data)
	{
		APS_ERR("null pointer!!\n");
		return -EINVAL;
	}
	return nonseekable_open(inode, file);
}
/************************************************************/
static int ap3220_release(struct inode *inode, struct file *file)
{
	file->private_data = NULL;
	return 0;
}
/************************************************************/
static long ap3220_unlocked_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
		struct i2c_client *client = (struct i2c_client*)file->private_data;
		struct ap3220_priv *obj = i2c_get_clientdata(client);  
		long err = 0;
		void __user *ptr = (void __user*) arg;
		int dat;
		uint32_t enable;
		int ps_result;
		
		switch (cmd)
		{
			case ALSPS_SET_PS_MODE:
				if(copy_from_user(&enable, ptr, sizeof(enable)))
				{
					err = -EFAULT;
					goto err_out;
				}
				if(enable)
				{
					if((err = ap3220_enable_ps(obj->client, 1)))
					{
						APS_ERR("enable ps fail: %ld\n", err); 
						goto err_out;
					}
					
					set_bit(CMC_BIT_PS, &obj->enable);
				}
				else
				{
					if((err = ap3220_enable_ps(obj->client, 0)))
					{
						APS_ERR("disable ps fail: %ld\n", err); 
						goto err_out;
					}
					clear_bit(CMC_BIT_PS, &obj->enable);
				}
				break;
	
			case ALSPS_GET_PS_MODE:
				enable = test_bit(CMC_BIT_PS, &obj->enable) ? (1) : (0);
				if(copy_to_user(ptr, &enable, sizeof(enable)))
				{
					err = -EFAULT;
					goto err_out;
				}
				break;
	
			case ALSPS_GET_PS_DATA:    
				if((err = ap3220_read_ps(obj->client, &obj->ps)))
				{
					goto err_out;
				}
				
				dat = ap3220_get_ps_value(obj, obj->ps);
				if(copy_to_user(ptr, &dat, sizeof(dat)))
				{
					err = -EFAULT;
					goto err_out;
				}  
				break;
	
			case ALSPS_GET_PS_RAW_DATA:    
				if((err = ap3220_read_ps(obj->client, &obj->ps)))
				{
					goto err_out;
				}
				
				dat = obj->ps;
				if(copy_to_user(ptr, &dat, sizeof(dat)))
				{
					err = -EFAULT;
					goto err_out;
				}  
				break;			  
	
			case ALSPS_SET_ALS_MODE:
	
				if(copy_from_user(&enable, ptr, sizeof(enable)))
				{
					err = -EFAULT;
					goto err_out;
				}
				if(enable)
				{
					if((err = ap3220_enable_als(obj->client, 1)))
					{
						APS_ERR("enable als fail: %ld\n", err); 
						goto err_out;
					}
					set_bit(CMC_BIT_ALS, &obj->enable);
				}
				else
				{
					if((err = ap3220_enable_als(obj->client, 0)))
					{
						APS_ERR("disable als fail: %ld\n", err); 
						goto err_out;
					}
					clear_bit(CMC_BIT_ALS, &obj->enable);
				}
				break;
	
			case ALSPS_GET_ALS_MODE:
				enable = test_bit(CMC_BIT_ALS, &obj->enable) ? (1) : (0);
				if(copy_to_user(ptr, &enable, sizeof(enable)))
				{
					err = -EFAULT;
					goto err_out;
				}
				break;
	
			case ALSPS_GET_ALS_DATA: 
				if((err = ap3220_read_als(obj->client, &obj->als)))
				{
					goto err_out;
				}
	
				dat = ap3220_get_als_value(obj, obj->als);
				if(copy_to_user(ptr, &dat, sizeof(dat)))
				{
					err = -EFAULT;
					goto err_out;
				}			   
				break;
	
			case ALSPS_GET_ALS_RAW_DATA:	
				if((err = ap3220_read_als(obj->client, &obj->als)))
				{
					goto err_out;
				}
	
				dat = obj->als;
				if(copy_to_user(ptr, &dat, sizeof(dat)))
				{
					err = -EFAULT;
					goto err_out;
				}			   
				break;

			/*----------------------------------for factory mode test---------------------------------------*/
			case ALSPS_GET_PS_TEST_RESULT:
				if((err = ap3220_read_ps(obj->client, &obj->ps)))
				{
					goto err_out;
				}
				if(obj->ps > atomic_read(&obj->ps_thd_val_high))
					{
						ps_result = 0;
					}
				else	ps_result = 1;
				
				if(copy_to_user(ptr, &ps_result, sizeof(ps_result)))
				{
					err = -EFAULT;
					goto err_out;
				}			   
				break;
			/*------------------------------------------------------------------------------------------*/
			case ALSPS_GET_PS_THRESHOLD_HIGH:
				dat = atomic_read(&obj->ps_thd_val_high);
				if(copy_to_user(ptr, &dat, sizeof(dat)))
				{
					err = -EFAULT;
					goto err_out;
				}			   			    
			    break;
			    
			    
			case ALSPS_GET_PS_THRESHOLD_LOW:
				dat = atomic_read(&obj->ps_thd_val_low);
				if(copy_to_user(ptr, &dat, sizeof(dat)))
				{
					err = -EFAULT;
					goto err_out;
				}			   			    
			    break;
			
			
			default:
				APS_ERR("%s not supported = 0x%04x", __FUNCTION__, cmd);
				err = -ENOIOCTLCMD;
				break;
		}
	
		err_out:
		return err;    
	}
/********************************************************************/
/*------------------------------misc device related operation functions------------------------------------*/
static struct file_operations ap3220_fops = {
	.owner = THIS_MODULE,
	.open = ap3220_open,
	.release = ap3220_release,
	.unlocked_ioctl = ap3220_unlocked_ioctl,
};

static struct miscdevice ap3220_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "als_ps",
	.fops = &ap3220_fops,
};

/*--------------------------------------------------------------------------------------*/
static void ap3220_early_suspend(struct early_suspend *h)
{
		struct ap3220_priv *obj = container_of(h, struct ap3220_priv, early_drv);	
		int err;
		APS_FUN();	  
	
		if(!obj)
		{
			APS_ERR("null pointer!!\n");
			return;
		}
		
		atomic_set(&obj->als_suspend, 1);
		if((err = ap3220_enable_als(obj->client, 0)))
		{
			APS_ERR("disable als fail: %d\n", err); 
		}
}

static void ap3220_late_resume(struct early_suspend *h) 
{
		struct ap3220_priv *obj = container_of(h, struct ap3220_priv, early_drv);		  
		int err;
		hwm_sensor_data sensor_data;
		memset(&sensor_data, 0, sizeof(sensor_data));
		APS_FUN();
		if(!obj)
		{
			APS_ERR("null pointer!!\n");
			return;
		}
	
		atomic_set(&obj->als_suspend, 0);
		if(test_bit(CMC_BIT_ALS, &obj->enable))
		{
			if((err = ap3220_enable_als(obj->client, 1)))
			{
				APS_ERR("enable als fail: %d\n", err);		  
	
			}
		}
}
/*--------------------------------------------------------------------------------*/
static int ap3220_init_client(struct i2c_client *client)
{
	struct ap3220_priv *obj = i2c_get_clientdata(client);
	u8 databuf[3];    
	u8 data, data2, data3;
	int res = 0;
	u16 tmp_data;
//ALS Range, 0 ~ 65536
//ALS PERSIST	AP3220_ALS_SETTING_RANGE_16383
	data = (AP3220_ALS_SETTING_RANGE_16383 << AP3220_ALS_RANGE_SHIFT) & AP3220_ALS_RANGE_MASK;
	data2 = (AP3220_ALS_SETTING_PERSIST_1 << AP3220_ALS_PERSIST_SHIFT) & AP3220_ALS_PERSIST_MASK;
	data |= data2;
	res = ap3220_i2c_write_reg(AP3220_REG_ALS_CONF,data);
	if(res < 0)
	{
		APS_ERR("i2c_master_send function err\n");
		goto EXIT_ERR;
	}
#ifdef ALS_AUTO_RANGE
	g_als_range = AP3220_ALS_SETTING_RANGE_16383;
#endif

//Ivan FIXME tunnable value to be confirmed by HW
//ALS Calibration 
	data = 0xFE;				//0x40/64 = 1
	res = ap3220_i2c_write_reg(AP3220_REG_ALS_CAL,data);
	if(res < 0)
	{
		APS_ERR("i2c_master_send function err\n");
		goto EXIT_ERR;
	}
//Ivan Set Low threshold to 0; try to disable ALS interrupt	
//ALS Low Threshold L 
	tmp_data = atomic_read(&obj->als_thd_val_low);
	data = tmp_data & 0x00FF;		// Read from cust_alsps.c
	APS_ERR("Ivan ALS Low Threshold L = %x\n",data);		
	res = ap3220_i2c_write_reg(AP3220_REG_ALS_THDL_L,data);
	if(res < 0)
	{
		APS_ERR("i2c_master_send function err\n");
		goto EXIT_ERR;
	}
//ALS Low Threshold H
	data = tmp_data >> 8;		// Read from cust_alsps.c
	APS_ERR("Ivan ALS Low Threshold H = %x\n",data);			
	res = ap3220_i2c_write_reg(AP3220_REG_ALS_THDL_H,data);
	if(res < 0)
	{
		APS_ERR("i2c_master_send function err\n");
		goto EXIT_ERR;
	}
//Ivan Set High threshold to 65536; try to disable ALS interrupt
//ALS High Threshold L 
	tmp_data = atomic_read(&obj->als_thd_val_high);
	data = tmp_data & 0x00FF;		// Read from cust_alsps.c
	APS_ERR("Ivan ALS High Threshold L = %x\n",data);			
	res = ap3220_i2c_write_reg(AP3220_REG_ALS_THDH_L,data);
	if(res < 0)
	{
		APS_ERR("i2c_master_send function err\n");
		goto EXIT_ERR;
	}
//ALS High Threshold H
	data = tmp_data >> 8;		// Read from cust_alsps.c
	APS_ERR("Ivan ALS High Threshold H = %x\n",data);
	res = ap3220_i2c_write_reg(AP3220_REG_ALS_THDH_L,data);
	if(res < 0)
	{
		APS_ERR("i2c_master_send function err\n");
		goto EXIT_ERR;
	}
	
//PS Configuration 
	data = (AP3220_PS_SETTING_INTG_TIME_1 << AP3220_PS_INTEGEATED_TIME_SHIFT) & AP3220_PS_INTEGEATED_TIME_MASK;
	data2 = (AP3220_PS_SETTING_GAIN_2 << AP3220_PS_GAIN_SHIFT) & AP3220_PS_GAIN_MASK;
	data3 = (AP3220_PS_SETTING_PERSIST_4 << AP3220_PS_PERSIST_SHIFT) & AP3220_PS_PERSIST_MASK;	
	data |= data2;
	data |= data3;
	res = ap3220_i2c_write_reg(AP3220_REG_PS_CONF,data);
	if(res < 0)
	{
		APS_ERR("i2c_master_send function err\n");
		goto EXIT_ERR;
	}
//PS LED Control
	data = (AP3220_PS_SETTING_LED_PULSE_1 << AP3220_PS_LED_PULSE_SHIFT) & AP3220_PS_LED_PULSE_MASK;
	data2 = (AP3220_PS_SETTING_LED_RATIO_66 << AP3220_PS_LED_RATIO_SHIFT) & AP3220_PS_LED_RATIO_MASK;
	data |= data2;
	res = ap3220_i2c_write_reg(AP3220_REG_PS_LED,data);
	if(res < 0)
	{
		APS_ERR("i2c_master_send function err\n");
		goto EXIT_ERR;
	}
//PS INT MODE
	data = AP3220_PS_SETTING_PS_ALGO_HYST;
	res = ap3220_i2c_write_reg(AP3220_REG_PS_INT_FORM,data);
	if(res < 0)
	{
		APS_ERR("i2c_master_send function err\n");
		goto EXIT_ERR;
	}
//PS MEAN TIME
	data = AP3220_PS_SETTING_PS_MEAN_12;
	res = ap3220_i2c_write_reg(AP3220_REG_PS_MEAN_TIME,data);
	if(res < 0)
	{
		APS_ERR("i2c_master_send function err\n");
		goto EXIT_ERR;
	}	
//PS LED WAITING
	data = 0;		// 0 = no waiting; 1 = 1 mean time, etc
	res = ap3220_i2c_write_reg(AP3220_REG_PS_WAIT_TIME,data);
	if(res < 0)
	{
		APS_ERR("i2c_master_send function err\n");
		goto EXIT_ERR;
	}	
//IVAN FIXME value to be confirmed with HW
//PS Calibration L 
	data = 0;		// 0 = no waiting; 1 = 1 mean time, etc
	res = ap3220_i2c_write_reg(AP3220_REG_PS_CAL_L,data);
	if(res < 0)
	{
		APS_ERR("i2c_master_send function err\n");
		goto EXIT_ERR;
	}
//PS Calibration H	
	data = 0;		// 0 = no waiting; 1 = 1 mean time, etc
	res = ap3220_i2c_write_reg(AP3220_REG_PS_CAL_H,data);
	if(res < 0)
	{
		APS_ERR("i2c_master_send function err\n");
		goto EXIT_ERR;
	}	
//PS Low Threshold L 
	tmp_data = atomic_read(&obj->ps_thd_val_low);
	data = tmp_data & 0x0003;		// Read from cust_alsps.c
	APS_ERR("Ivan PS Low Threshold L = %x\n",data);	
	res = ap3220_i2c_write_reg(AP3220_REG_PS_THDL_L,data);
	if(res < 0)
	{
		APS_ERR("i2c_master_send function err\n");
		goto EXIT_ERR;
	}	
//PS Low Threshold H
	data = tmp_data >> 2;		// Read from cust_alsps.c
	APS_ERR("Ivan PS Low Threshold H = %x\n",data);		
	res = ap3220_i2c_write_reg(AP3220_REG_PS_THDL_H,data);
	if(res < 0)
	{
		APS_ERR("i2c_master_send function err\n");
		goto EXIT_ERR;
	}	
//PS High Threshold L 
	tmp_data = atomic_read(&obj->ps_thd_val_high);
	data = tmp_data & 0x0003;	// Read from cust_alsps.c
	APS_ERR("Ivan PS High Threshold L = %x\n",data);	
	res = ap3220_i2c_write_reg(AP3220_REG_PS_THDH_L,data);
	if(res < 0)
	{
		APS_ERR("i2c_master_send function err\n");
		goto EXIT_ERR;
	}	
//PS High Threshold H
	data = tmp_data >> 2;		// Read from cust_alsps.c
	APS_ERR("Ivan PS High Threshold H = %x\n",data);		
	res = ap3220_i2c_write_reg(AP3220_REG_PS_THDH_H,data);
	if(res < 0)
	{
		APS_ERR("i2c_master_send function err\n");
		goto EXIT_ERR;
	}	
//DEVICE POWER DOWN
	data = 0;
	res = ap3220_i2c_write_reg(AP3220_REG_SYS_CONF,data);
	if(res < 0)
	{
		APS_ERR("i2c_master_send function err\n");
		goto EXIT_ERR;
	}	
				
	res = ap3220_setup_eint(client);
	if(res!=0)
	{
		APS_ERR("setup eint: %d\n", res);
		return res;
	}
	
	return AP3220_SUCCESS;
	
	EXIT_ERR:
	APS_ERR("init dev: %d\n", res);
	return res;
}
/*--------------------------------------------------------------------------------*/

/*--------------------------------------------------------------------------------*/
long ap3220_ps_operate(void* self, uint32_t command, void* buff_in, int size_in,
		void* buff_out, int size_out, int* actualout)
{
		long err = 0;
		int value;
		hwm_sensor_data* sensor_data;
		struct ap3220_priv *obj = (struct ap3220_priv *)self;		
		APS_FUN(f);
		switch (command)
		{
			case SENSOR_DELAY:
				APS_ERR("ap3220 ps delay command!\n");
				if((buff_in == NULL) || (size_in < sizeof(int)))
				{
					APS_ERR("Set delay parameter error!\n");
					err = -EINVAL;
				}
				break;
	
			case SENSOR_ENABLE:
				APS_ERR("ap3220 ps enable command!\n");
				if((buff_in == NULL) || (size_in < sizeof(int)))
				{
					APS_ERR("Enable sensor parameter error!\n");
					err = -EINVAL;
				}
				else
				{				
					value = *(int *)buff_in;
					if(value)
					{
						if((err = ap3220_enable_ps(obj->client, 1)))
						{
							APS_ERR("enable ps fail: %d\n", err); 
							return -1;
						}
						set_bit(CMC_BIT_PS, &obj->enable);
//Ivan
#ifdef TINNO_PS_STARTUP_IRQ
						queue_delayed_work(ps_startup_irq_workqueue, &ps_startup_irq_work,STARTUP_IRQ_DELAY);
#endif						
					}
					else
					{
						if((err = ap3220_enable_ps(obj->client, 0)))
						{
							APS_ERR("disable ps fail: %d\n", err); 
							return -1;
						}
						clear_bit(CMC_BIT_PS, &obj->enable);
					}
				}
				break;
	
			case SENSOR_GET_DATA:
				APS_ERR("ap3220 ps get data command!\n");
				if((buff_out == NULL) || (size_out< sizeof(hwm_sensor_data)))
				{
					APS_ERR("get sensor data parameter error!\n");
					err = -EINVAL;
				}
				else
				{
					sensor_data = (hwm_sensor_data *)buff_out;				
					
					if((err = ap3220_read_ps(obj->client, &obj->ps)))
					{
						err = -1;;
					}
					else
					{
						sensor_data->values[0] = ap3220_get_ps_value(obj, obj->ps);
						sensor_data->value_divide = 1;
						sensor_data->status = SENSOR_STATUS_ACCURACY_MEDIUM;
					}				
				}
				break;
			default:
				APS_ERR("proxmy sensor operate function no this parameter %d!\n", command);
				err = -1;
				break;
		}
		
		return err;

}

long ap3220_als_operate(void* self, uint32_t command, void* buff_in, int size_in,
		void* buff_out, int size_out, int* actualout)
{
		long err = 0;
		int value;
		hwm_sensor_data* sensor_data;
		struct ap3220_priv *obj = (struct ap3220_priv *)self;
//		APS_FUN(f);
		switch (command)
		{
			case SENSOR_DELAY:
				APS_ERR("ap3220 als delay command!\n");
				if((buff_in == NULL) || (size_in < sizeof(int)))
				{
					APS_ERR("Set delay parameter error!\n");
					err = -EINVAL;
				}
				break;
	
			case SENSOR_ENABLE:
				APS_ERR("ap3220 als enable command!\n");
				if((buff_in == NULL) || (size_in < sizeof(int)))
				{
					APS_ERR("Enable sensor parameter error!\n");
					err = -EINVAL;
				}
				else
				{
					value = *(int *)buff_in;				
					if(value)
					{
						if((err = ap3220_enable_als(obj->client, 1)))
						{
							APS_ERR("enable als fail: %d\n", err); 
							return -1;
						}
						set_bit(CMC_BIT_ALS, &obj->enable);
					}
					else
					{
						if((err = ap3220_enable_als(obj->client, 0)))
						{
							APS_ERR("disable als fail: %d\n", err); 
							return -1;
						}
						clear_bit(CMC_BIT_ALS, &obj->enable);
					}
					
				}
				break;
	
			case SENSOR_GET_DATA:
//				APS_ERR("ap3220 als get data command!\n");
				if((buff_out == NULL) || (size_out< sizeof(hwm_sensor_data)))
				{
					APS_ERR("get sensor data parameter error!\n");
					err = -EINVAL;
				}
				else
				{
					sensor_data = (hwm_sensor_data *)buff_out;
									
					if((err = ap3220_read_als(obj->client, &obj->als)))
					{
						err = -1;;
					}
					else
					{
						sensor_data->values[0] = ap3220_get_als_value(obj, obj->als);
						sensor_data->value_divide = 1;
						sensor_data->status = SENSOR_STATUS_ACCURACY_MEDIUM;
					}				
				}
				break;
			default:
				APS_ERR("light sensor operate function no this parameter %d!\n", command);
				err = -1;
				break;
		}
		
		return err;

}
/*--------------------------------------------------------------------------------*/


/*-----------------------------------i2c operations----------------------------------*/
static int ap3220_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct ap3220_priv *obj;
	struct hwmsen_object obj_ps, obj_als;
	int err = 0;

	if(!(obj = kzalloc(sizeof(*obj), GFP_KERNEL)))
	{
		err = -ENOMEM;
		goto exit;
	}
	
	memset(obj, 0, sizeof(*obj));
	ap3220_obj = obj;
	
	obj->hw = get_cust_alsps_hw();//get custom file data struct
	
	INIT_WORK(&obj->eint_work, ap3220_eint_work);

	obj->client = client;
	i2c_set_clientdata(client, obj);

	/*-----------------------------value need to be confirmed-----------------------------------------*/
	atomic_set(&obj->als_debounce, 200);
	atomic_set(&obj->als_deb_on, 0);
	atomic_set(&obj->als_deb_end, 0);
	atomic_set(&obj->ps_debounce, 200);
	atomic_set(&obj->ps_deb_on, 0);
	atomic_set(&obj->ps_deb_end, 0);
	atomic_set(&obj->ps_mask, 0);
	atomic_set(&obj->als_suspend, 0);
	atomic_set(&obj->als_cmd_val, 0xDF);
	atomic_set(&obj->ps_cmd_val,  0xC1);
	atomic_set(&obj->ps_thd_val_high,  obj->hw->ps_threshold_high);
	atomic_set(&obj->ps_thd_val_low,  obj->hw->ps_threshold_low);
	atomic_set(&obj->als_thd_val_high,  obj->hw->als_threshold_high);
	atomic_set(&obj->als_thd_val_low,  obj->hw->als_threshold_low);
	
	obj->enable = 0;
	obj->pending_intr = 0;
	obj->als_level_num = sizeof(obj->hw->als_level)/sizeof(obj->hw->als_level[0]);
	obj->als_value_num = sizeof(obj->hw->als_value)/sizeof(obj->hw->als_value[0]);
	/*-----------------------------value need to be confirmed-----------------------------------------*/
	
	BUG_ON(sizeof(obj->als_level) != sizeof(obj->hw->als_level));
	memcpy(obj->als_level, obj->hw->als_level, sizeof(obj->als_level));
	BUG_ON(sizeof(obj->als_value) != sizeof(obj->hw->als_value));
	memcpy(obj->als_value, obj->hw->als_value, sizeof(obj->als_value));
	atomic_set(&obj->i2c_retry, 3);
	set_bit(CMC_BIT_ALS, &obj->enable);
	set_bit(CMC_BIT_PS, &obj->enable);

	ap3220_i2c_client = client;

	if((err = ap3220_init_client(client)))
	{
		goto exit_init_failed;
	}
	APS_LOG("ap3220_init_client() OK!\n");

	if((err = misc_register(&ap3220_device)))
	{
		APS_ERR("ap3220_device register failed\n");
		goto exit_misc_device_register_failed;
	}
	APS_LOG("ap3220_device misc_register OK!\n");

	/*------------------------ap3220 attribute file for debug--------------------------------------*/
	if((err = ap3220_create_attr(&ap3220_alsps_driver.driver)))
	{
		APS_ERR("create attribute err = %d\n", err);
		goto exit_create_attr_failed;
	}
	/*------------------------ap3220 attribute file for debug--------------------------------------*/

	obj_ps.self = ap3220_obj;
	obj_ps.polling = obj->hw->polling_mode_ps;	
	obj_ps.sensor_operate = ap3220_ps_operate;
	if((err = hwmsen_attach(ID_PROXIMITY, &obj_ps)))
	{
		APS_ERR("attach fail = %d\n", err);
		goto exit_sensor_obj_attach_fail;
	}
		
	obj_als.self = ap3220_obj;
	obj_als.polling = obj->hw->polling_mode_als;;
	obj_als.sensor_operate = ap3220_als_operate;
	if((err = hwmsen_attach(ID_LIGHT, &obj_als)))
	{
		APS_ERR("attach fail = %d\n", err);
		goto exit_sensor_obj_attach_fail;
	}

	#if defined(CONFIG_HAS_EARLYSUSPEND)
	obj->early_drv.level    = EARLY_SUSPEND_LEVEL_STOP_DRAWING - 2,
	obj->early_drv.suspend  = ap3220_early_suspend,
	obj->early_drv.resume   = ap3220_late_resume,    
	register_early_suspend(&obj->early_drv);
	#endif

#ifdef TINNO_PS_STARTUP_IRQ
	ps_startup_irq_workqueue = create_workqueue("ps_startup_irq");
	
	INIT_DELAYED_WORK(&ps_startup_irq_work, ps_startup_irq_func);
	
//	queue_delayed_work(ps_startup_irq_workqueue, &ps_startup_irq_work,STARTUP_IRQ_DELAY);
#endif
#ifdef ALS_RANGE_DEBUG
	currentBootMode = get_boot_mode();
#endif	
	APS_LOG("%s: OK\n", __func__);
	return 0;

	exit_create_attr_failed:
	exit_sensor_obj_attach_fail:
	exit_misc_device_register_failed:
		misc_deregister(&ap3220_device);
	exit_init_failed:
		kfree(obj);
	exit:
	ap3220_i2c_client = NULL;           
	APS_ERR("%s: err = %d\n", __func__, err);
	return err;
}

static int ap3220_i2c_remove(struct i2c_client *client)
{
	int err;	
	/*------------------------ap3220 attribute file for debug--------------------------------------*/	
	if((err = ap3220_delete_attr(&ap3220_i2c_driver.driver)))
	{
		APS_ERR("ap3220_delete_attr fail: %d\n", err);
	} 
	/*----------------------------------------------------------------------------------------*/
	
	if((err = misc_deregister(&ap3220_device)))
	{
		APS_ERR("misc_deregister fail: %d\n", err);    
	}
		
	ap3220_i2c_client = NULL;
	i2c_unregister_device(client);
	kfree(i2c_get_clientdata(client));
	return 0;

}

static int ap3220_i2c_detect(struct i2c_client *client, struct i2c_board_info *info)
{
	strcpy(info->type, AP3220_DEV_NAME);
	return 0;

}

static int ap3220_i2c_suspend(struct i2c_client *client, pm_message_t msg)
{
	APS_FUN();
	return 0;
}

static int ap3220_i2c_resume(struct i2c_client *client)
{
	APS_FUN();
	return 0;
}

/*----------------------------------------------------------------------------*/

static int ap3220_probe(struct platform_device *pdev) 
{
	APS_FUN();  
	struct alsps_hw *hw = get_cust_alsps_hw();

	ap3220_power(hw, 1); //*****************   
	
	if(i2c_add_driver(&ap3220_i2c_driver))
	{
		APS_ERR("add driver error\n");
		return -1;
	} 
	return 0;
}
/*----------------------------------------------------------------------------*/
static int ap3220_remove(struct platform_device *pdev)
{
	
	APS_FUN(); 
	struct alsps_hw *hw = get_cust_alsps_hw();
	
	ap3220_power(hw, 0);//*****************  
	
	i2c_del_driver(&ap3220_i2c_driver);
	
#ifdef TINNO_PS_STARTUP_IRQ
	destroy_workqueue(ps_startup_irq_workqueue);
#endif
	return 0;
}



/*----------------------------------------------------------------------------*/
static struct platform_driver ap3220_alsps_driver = {
	.probe      = ap3220_probe,
	.remove     = ap3220_remove,    
	.driver     = {
		.name  = "als_ps",
	}
};

/*----------------------------------------------------------------------------*/
static int __init ap3220_init(void)
{
	APS_FUN();
	i2c_register_board_info(3, &i2c_ap3220, 1);
	if(platform_driver_register(&ap3220_alsps_driver))
	{
		APS_ERR("failed to register driver");
		return -ENODEV;
	}
	return 0;
}
/*----------------------------------------------------------------------------*/
static void __exit ap3220_exit(void)
{
	APS_FUN();
	platform_driver_unregister(&ap3220_alsps_driver);
}
/*----------------------------------------------------------------------------*/
module_init(ap3220_init);
module_exit(ap3220_exit);
/*----------------------------------------------------------------------------*/
MODULE_AUTHOR("yucong xiong");
MODULE_DESCRIPTION("ap3220 driver");
MODULE_LICENSE("GPL");

