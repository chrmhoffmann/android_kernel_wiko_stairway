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
#include "tpd.h"
#include <linux/interrupt.h>
#include <cust_eint.h>
#include <linux/i2c.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/rtpm_prio.h>
#include <linux/wait.h>
#include <linux/time.h>
#include <linux/delay.h>
#include <linux/miscdevice.h>
#include <linux/cdev.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include "tpd_custom_ektf2136.h"
#include "cust_gpio_usage.h"


#include <mach/mt_pm_ldo.h>
#include <mach/mt_typedefs.h>
#include <mach/mt_boot.h>

/*****************************************************************************
*                E X T E R N A L      R E F E R E N C E S
******************************************************************************
*/
extern struct tpd_device *tpd;
static struct i2c_client *i2c_client = NULL;
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
static int ektf2136_ts_get_fw_version(void);
#ifdef      SUPPORT_UPDATE_FW

#endif
#ifdef SW_FIRMWARE_UPDATE
int elan_iap_open(struct inode *inode, struct file *filp);
int elan_iap_release(struct inode *inode, struct file *filp);
static ssize_t elan_iap_write(struct file *filp, const char *buff, size_t count, loff_t *offp);
ssize_t elan_iap_read(struct file *filp, char *buff, size_t count, loff_t *offp);
ssize_t elan_iap_read(struct file *filp, char *buff, size_t count, loff_t *offp);
static int elan_ktf2k_ts_poll(struct i2c_client *client);
static int elan_ktf2k_ts_get_data(struct i2c_client *client, uint8_t *cmd,
			uint8_t *buf, size_t size);
static int fw_packet_handler(struct i2c_client *client);
static long elan_iap_ioctl( struct file *filp,    unsigned int cmd, unsigned long arg);
#endif 
/*****************************************************************************
*                          C O N S T A N T S
******************************************************************************
*/
#if defined (SUPORT_5_POINTS)
#define POINTS_NUM                5
#else
#define POINTS_NUM                2
#endif

static int ctp_suspend= -1; // show that: ctp is power_down or not
static char vendor_version = 0;
static char panel_version = 0;
static int tpd_flag = 0;
static int point_num = 0;
static int p_point_num = 0;

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
#define TP_LIB_NAME     "eKTF2136.i"
static unsigned char s_ctp_fw[] =
{
    #include TP_LIB_NAME
};
#define PAGE_SIZE       132
#define ACK_OK          0xaa
#endif

#ifdef  SW_FIRMWARE_UPDATE
#define CMD_S_PKT			0x52    // IC response to packet 0x53 from the host
#define CMD_R_PKT			0x53
#define CMD_W_PKT			0x54

#define ELAN_IOCTLID	0xD0
#define IOCTL_I2C_SLAVE	_IOW(ELAN_IOCTLID,  1, int)
#define IOCTL_MAJOR_FW_VER  _IOR(ELAN_IOCTLID, 2, int)
#define IOCTL_MINOR_FW_VER  _IOR(ELAN_IOCTLID, 3, int)
#define IOCTL_RESET  _IOR(ELAN_IOCTLID, 4, int)
#define IOCTL_IAP_MODE_LOCK  _IOR(ELAN_IOCTLID, 5, int)
#define IOCTL_CHECK_RECOVERY_MODE  _IOR(ELAN_IOCTLID, 6, int)
#define IOCTL_FW_VER  _IOR(ELAN_IOCTLID, 7, int)
#define IOCTL_X_RESOLUTION  _IOR(ELAN_IOCTLID, 8, int)
#define IOCTL_Y_RESOLUTION  _IOR(ELAN_IOCTLID, 9, int)
#define IOCTL_FW_ID  _IOR(ELAN_IOCTLID, 10, int)
#define IOCTL_IAP_MODE_UNLOCK  _IOR(ELAN_IOCTLID, 11, int)
#define IOCTL_I2C_INT  _IOR(ELAN_IOCTLID, 12, int)

uint16_t checksum_err=0;
uint8_t RECOVERY=0x00;
int FW_VERSION=0x00;
int X_RESOLUTION=0x00;
int Y_RESOLUTION=0x00;
int FW_ID=0x00;
int work_lock=0x00;
int button_state = 0;

struct elan_ktf2k_ts_data {
	struct i2c_client *client;
	struct input_dev *input_dev;
	struct workqueue_struct *elan_wq;
	struct work_struct work;
	struct early_suspend early_suspend;
	int intr_gpio;   //ctp EINT pin
// Firmware Information
	int fw_ver;
	int fw_id;
	int x_resolution;
	int y_resolution;
// For Firmare Update 
	struct miscdevice firmware;
};

static struct elan_ktf2k_ts_data *private_ts;

typedef enum
{
    ERR_OK,
    ERR_MODE,
    ERR_READID,
    ERR_ERASE,
    ERR_STATUS,
    ERR_ECC,
    ERR_DL_ERASE_FAIL,
    ERR_DL_PROGRAM_FAIL,
    ERR_DL_VERIFY_FAIL
} E_UPGRADE_ERR_TYPE;
#endif
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

#ifdef SW_FIRMWARE_UPDATE
struct file_operations elan_touch_fops = {    
    .open =         elan_iap_open,    
    .write =        elan_iap_write,    
    .read = 	elan_iap_read,    
    .release =	elan_iap_release,    
	.unlocked_ioctl = elan_iap_ioctl, 
};
#endif
/*****************************************************************************
*                  F U N C T I O N        D E F I N I T I O N
******************************************************************************
*/


static int i2c_read_bytes(struct i2c_client *client, u16 addr, u8 *rxbuf, int len)
{
    u8 buffer[GTP_ADDR_LENGTH];
    u16 left = len;
    u16 offset = 0;

    struct i2c_msg msg[2] =
    {
        {
            .addr = ((client->addr &I2C_MASK_FLAG) | (I2C_ENEXT_FLAG)),
            //.addr = (client->addr &I2C_MASK_FLAG),
	    //.ext_flag = I2C_ENEXT_FLAG,
            //.addr = ((client->addr &I2C_MASK_FLAG) | (I2C_PUSHPULL_FLAG)),
            .flags = 0,
            .buf = buffer,
            .len = GTP_ADDR_LENGTH,
            .timing = I2C_MASTER_CLOCK
        },
        {
            .addr = ((client->addr &I2C_MASK_FLAG) | (I2C_ENEXT_FLAG)),
            //.addr = (client->addr &I2C_MASK_FLAG),
	    //.ext_flag = I2C_ENEXT_FLAG,
            //.addr = ((client->addr &I2C_MASK_FLAG) | (I2C_PUSHPULL_FLAG)),
            .flags = I2C_M_RD,
            .timing = I2C_MASTER_CLOCK
        },
    };

    if (rxbuf == NULL)
        return -1;

    TPD_DEBUG("i2c_read_bytes to device %02X address %04X len %d\n", client->addr, addr, len);

    while (left > 0)
    {
        buffer[0] = ((addr + offset) >> 8) & 0xFF;
        buffer[1] = (addr + offset) & 0xFF;

        msg[1].buf = &rxbuf[offset];

        if (left > MAX_TRANSACTION_LENGTH)
        {
            msg[1].len = MAX_TRANSACTION_LENGTH;
            left -= MAX_TRANSACTION_LENGTH;
            offset += MAX_TRANSACTION_LENGTH;
        }
        else
        {
            msg[1].len = left;
            left = 0;
        }

        if (i2c_transfer(client->adapter, &msg[0], 2) != 2)
        {
                printk("I2C read 0x%X length=%d failed\n", addr + offset, len);
                return -1;
        }
    }

    return 0;
}

s32 elan_i2c_read(struct i2c_client *client, u8 *buf, s32 len)
{
    s32 ret = -1;
    u16 addr = (buf[0] << 8) + buf[1];

    ret = i2c_read_bytes(client, addr, &buf[2], len - 2);

    if (!ret)
    {
        return len;
    }
    else
    {
//        gtp_reset_guitar(client, 20);
        return ret;
    }
}

static int i2c_write_bytes(struct i2c_client *client, u16 addr, u8 *txbuf, int len)
{
    u8 buffer[MAX_TRANSACTION_LENGTH];
    u16 left = len;
    u16 offset = 0;

    struct i2c_msg msg =
    {
        .addr = ((client->addr &I2C_MASK_FLAG) | (I2C_ENEXT_FLAG)),
        //.addr = (client->addr &I2C_MASK_FLAG),
	//.ext_flag = I2C_ENEXT_FLAG,
        //.addr = ((client->addr &I2C_MASK_FLAG) | (I2C_PUSHPULL_FLAG)),
        .flags = 0,
        .buf = buffer,
        .timing = I2C_MASTER_CLOCK,
    };


    if (txbuf == NULL)
        return -1;

    TPD_DEBUG("i2c_write_bytes to device %02X address %04X len %d\n", client->addr, addr, len);

    while (left > 0)
    {
        buffer[0] = ((addr + offset) >> 8) & 0xFF;
        buffer[1] = (addr + offset) & 0xFF;

        if (left > MAX_I2C_TRANSFER_SIZE)
        {
            memcpy(&buffer[GTP_ADDR_LENGTH], &txbuf[offset], MAX_I2C_TRANSFER_SIZE);
            msg.len = MAX_TRANSACTION_LENGTH;
            left -= MAX_I2C_TRANSFER_SIZE;
            offset += MAX_I2C_TRANSFER_SIZE;
        }
        else
        {
            memcpy(&buffer[GTP_ADDR_LENGTH], &txbuf[offset], left);
            msg.len = left + GTP_ADDR_LENGTH;
            left = 0;
        }

        //GTP_DEBUG("byte left %d offset %d\n", left, offset);

        if (i2c_transfer(client->adapter, &msg, 1) != 1)
        {
                printk("I2C write 0x%X%X length=%d failed\n", buffer[0], buffer[1], len);
                return -1;
        }
    }

    return 0;
}

s32 elan_i2c_write(struct i2c_client *client, u8 *buf, s32 len)
{
    s32 ret = -1;
    u16 addr = (buf[0] << 8) + buf[1];

    ret = i2c_write_bytes(client, addr, &buf[2], len - 2);

    if (!ret)
    {
        return 1;
    }
    else
    {
//        gtp_reset_guitar(client, 20);
        return ret;
    }
}


void tpd_reset()
{
    mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ZERO);
    msleep(30);
    mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ONE);
}

void tpd_wr_datas(unsigned char *buff, int count)
{
    int ret;
    ret = i2c_master_send(i2c_client, buff, count);

    if(ret <= 0) {
        TPD_DEBUG("[elan::] tpd_wr_datas error!\n");
    }
}

void tpd_rd_datas(unsigned char *buff, int count)
{
    int ret;
    ret = i2c_master_recv(i2c_client, buff, count);

    if(ret <= 0) {
        TPD_DEBUG("[elan::] tpd_rd_datas error!\n");
    }
}

#ifdef      SUPPORT_UPDATE_FW
// one page is 132byte = 16*8 + 4
kal_uint8 update_write_page(unsigned char *fw)
{
	kal_uint8 byte_count, curIndex = 0;
	kal_uint8 *szBuff = NULL;

	for(byte_count=1; byte_count<=17; byte_count++) {
		if(byte_count!=17) {
			szBuff = fw + curIndex;
			curIndex =  curIndex + 8;
			tpd_wr_datas(szBuff, 8);
		} else {
			szBuff = fw + curIndex;
			curIndex =  curIndex + 4;
			tpd_wr_datas(szBuff, 4);
		}
	}
}

kal_uint8 update_firmware()
{
	kal_uint8 *fw_ptr;
	kal_uint8 res = 0;
	kal_uint8 iPage = 0, ackcnt = 0, iCnt = 0;
	kal_uint8 i = 0; // sndisp=0, sndid=0, j=0, recovery_slave=0;
	kal_uint8 data;
	kal_uint8 intlow = 0, ioctl_cnt = 0;
	kal_uint8 byte_count;
	kal_uint8 *szBuff = NULL;
	kal_uint8 curIndex = 0, retries = 0;
	kal_uint8 isp_cmd[] = {0x54, 0x00, 0x12, 0x34},dummy_cmd[] = {0x15};
	unsigned char read_buffer[2] = {0x55,0x55};
	unsigned char iapbuf[4];
	unsigned char fw_minor_version = 0x0, ekt_minor_version = 0xff;
	const unsigned char hello_packet[4] = {0x55, 0x55, 0x55, 0x55};
	const unsigned char recov_packet[4] = {0x55, 0x55, 0x80, 0x80};
    kal_uint8 rfwver[4] = {0x53, 0x00, 0x00, 0x01};

	TPD_DEBUG("[elan::] update_firmware()\n");
    fw_ptr = s_ctp_fw;
	{			
    //get firmware version
again:
		tpd_wr_datas(rfwver,4);
		msleep(10);
		tpd_rd_datas(iapbuf,4);	
        TPD_DEBUG("[elan::] ver related datas iapbuf[0]=0x%x, iapbuf[1]=0x%x, iapbuf[2]=0x%x, iapbuf[3]=0x%x\n", iapbuf[0], iapbuf[1], iapbuf[2], iapbuf[3]);
		if (iapbuf[0] == 0x52){
			fw_minor_version =((iapbuf[2] & 0x0f) << 4) | ((iapbuf[3] & 0xf0) >> 4);
            TPD_DEBUG("[elan::] fw_minor_version=%d, v[0]=0x%x, v[1]=0x%x, v[2]=0x%x, v[3]=0x%x!\n", fw_minor_version, iapbuf[0], iapbuf[1], iapbuf[2], iapbuf[3]);
		}else {
			retries++;
			if(retries<3)
				goto again;
				/* if more than 5 times, fw may stay in recovery, continue IAP anyway */
		}

		retries = 0;
		/* 0xe0e1 postion is what elan fw version be located */
		ekt_minor_version = fw_ptr[0x7bd0];
        TPD_DEBUG("[elan::] ekt_minor_version=0x%x, fw_minor_version=0x%x\n", ekt_minor_version, fw_minor_version);
		if (ekt_minor_version <= fw_minor_version){
            TPD_DEBUG("[elan::] version is the same,no need to update!\n");
			return KAL_TRUE;
		}
    }

	//update firmware:
reset:
	/* reset chip */
	tpd_reset();
	msleep(800);
	/* read hello packet */
	tpd_rd_datas(iapbuf,4);	
	TPD_DEBUG("[elan::] read hello packet iapbuf[0]=0x%x, iapbuf[1]=0x%x, iapbuf[2]=0x%x, iapbuf[3]=0x%x\n", iapbuf[0], iapbuf[1], iapbuf[2], iapbuf[3]);

    if (memcmp(iapbuf, hello_packet, 4) == 0) {
        TPD_DEBUG("[elan::] Normal mode, preparing to update!\n");
		goto normal_iap;
    }else if (memcmp(iapbuf, recov_packet, 4) == 0){
		/* recived BC version */
		msleep(20);
		tpd_rd_datas(iapbuf, 4);
		TPD_DEBUG("[elan::] Recovery mode, BC Ver:%02x:%02x:%02x:%02x\n",
		          iapbuf[0], iapbuf[1], iapbuf[2], iapbuf[3]);
		goto recovery;
	}
    

normal_iap:
	msleep(10);
	tpd_wr_datas(isp_cmd, 4);
recovery:
	msleep(50);
	tpd_wr_datas(dummy_cmd, 1);
    msleep(500);
	// write 249 pages
	for( iPage = 1; iPage <= 249; iPage++ ) {
retry:
        //TPD_DEBUG("[elan::] Update firmware ipage=%d\n", iPage);
		update_write_page(fw_ptr);
        msleep(1);
		tpd_rd_datas(read_buffer, 2);
		if(read_buffer[0] != ACK_OK) {
			ackcnt = ackcnt+1;
			if (ackcnt >= 30) {
				TPD_DEBUG("[elan::] serial_read_register %x  %x\n", read_buffer[0], read_buffer[1]);
				TPD_DEBUG("[elan::] Update_FW_ERR\n");				
				tpd_reset();
				msleep(2);
				return KAL_FALSE;		
			}else {
				goto retry;
			}
		}else {
			fw_ptr += PAGE_SIZE;
			ackcnt = 0;
		}
		msleep(1);
	}
	/* reset chip for continue TP work */
	tpd_reset();
	msleep(100);
    ektf2136_ts_get_fw_version();  
	TPD_DEBUG("[elan::] Update_FW_OK\n");
	return KAL_TRUE;
}

static int update_firmware_thread(void *priv)
{
    TPD_DEBUG("[elan::] enter update_firmware_thread\n");
    update_firmware();
	kthread_should_stop();

	return NULL;
}
#endif


static void tpd_down(int x, int y, int id) {
  
	TPD_DEBUG("tpd_down!\n");
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

static void tpd_up(int x, int y,int *count) {

	    TPD_DEBUG("tpd_up!\n");
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
	ret = i2c_master_recv(i2c_client, raw_data_buf, 8);  // i2c support > 8bits transfer
	ret = i2c_master_recv(i2c_client, &raw_data_buf[8], 8);  // i2c support > 8bits transfer
	ret = i2c_master_recv(i2c_client, &raw_data_buf[16], 2);  // i2c support > 8bits transfer
	
    p_point_num = point_num;    //remerber the last point number
    
    TPD_DEBUG("\n/********************************************/\n");
    for (i=0; i<18; i++) {
        TPD_DEBUG("%x  ", raw_data_buf[i]);
        if ((i%6 == 0) && (i != 0)) {
            TPD_DEBUG("\n");
        }
    }
    TPD_DEBUG("/********************************************/\n");
	
// for 5 fingers	
    if ((raw_data_buf[0] == 0x6D))  //6D for packet <=8 , 5D for packet > 8
    {   
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
        TPD_DEBUG("Magnum tpd_dcount == %d \n",tpd_dcount);
	    if(tpd_dcount)
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

#endif 
	return true;
};

static void tpd_eint_interrupt_handler(void)
{
    TPD_DEBUG("tpd_eint_interrupt_handler......\n");
    tpd_flag = 1;
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
        set_current_state(TASK_RUNNING);

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
			TPD_DEBUG("TPD up x == %d , y == %d\n!",x_history[cinfo.count], y_history[cinfo.count]);
			tpd_up(x_history[cinfo.count], y_history[cinfo.count], 0);
		//	input_mt_sync(tpd->dev);
			input_sync(tpd->dev);
		}    
    }
    while (!kthread_should_stop());
    
    return 0;
}

static int ektf2136_get_fw_version_stored()
{
//edit by Magnum 2012-3-1
//    panel_version = tinno_ts_get_fw_version();
    return panel_version;
}

static int ektf2136_get_vendor_version_stored()
{
     return vendor_version;
}

static int ektf2136_ts_get_fw_version(void)
{
	unsigned char fwbuf[4];
	kal_uint8 rfwver[4] = {0x53, 0x00, 0x00, 0x01};
	tpd_wr_datas(rfwver,4);
	msleep(10);
	tpd_rd_datas(fwbuf,4);	
    TPD_DEBUG("[elan::] ver related datas iapbuf[0]=0x%x, iapbuf[1]=0x%x, iapbuf[2]=0x%x, iapbuf[3]=0x%x\n", fwbuf[0], fwbuf[1], fwbuf[2], fwbuf[3]);
	if (fwbuf[0] == 0x52){
		vendor_version =((fwbuf[1] & 0x0f) << 4) | ((fwbuf[2] & 0xf0) >> 4);
		panel_version = ((fwbuf[2] & 0x0f) << 4) | ((fwbuf[3] & 0xf0) >> 4);
		#ifdef SW_FIRMWARE_UPDATE
		FW_VERSION = vendor_version<< 8 | panel_version; 
		#endif
		TPD_DEBUG("*** vendor_version = %x ***\n", vendor_version);
	    TPD_DEBUG("*** panel_version = %x ***\n", panel_version);
		return 1;
	}
	else{
		TPD_DEBUG("*** read version FAIL ***\n");
		return 0;
	}
}

static int tpd_detect(struct i2c_client *client, int kind, struct i2c_board_info *info)
{
    strcpy(info->type, tpd_driver_name);
    return 0;
}

#ifdef SW_FIRMWARE_UPDATE
int elan_iap_open(struct inode *inode, struct file *filp){ 
	TPD_DEBUG("[ELAN]into elan_iap_open\n");
		if (private_ts == NULL)  printk("private_ts is NULL~~~");
		
	return 0;
}

int elan_iap_release(struct inode *inode, struct file *filp){    
	return 0;
}

static ssize_t elan_iap_write(struct file *filp, const char *buff, size_t count, loff_t *offp){  
    int ret;
    char *tmp;
    TPD_DEBUG("[ELAN]into elan_iap_write\n");

    if (count > 8192)
        count = 8192;

    tmp = kmalloc(count, GFP_KERNEL);
    
    if (tmp == NULL)
        return -ENOMEM;

    if (copy_from_user(tmp, buff, count)) {
        return -EFAULT;
    }
	
    ret = i2c_master_send(private_ts->client, tmp, count);
    if (ret != count) TPD_DEBUG("ELAN i2c_master_send fail, ret=%d \n", ret);
    kfree(tmp);
    return ret;

}

ssize_t elan_iap_read(struct file *filp, char *buff, size_t count, loff_t *offp){    
    char *tmp;
    int ret;  
    long rc;
    TPD_DEBUG("[ELAN]into elan_iap_read\n");
   
    if (count > 8192)
        count = 8192;

    tmp = kmalloc(count, GFP_KERNEL);

    if (tmp == NULL)
        return -ENOMEM;

    ret = i2c_master_recv(private_ts->client, tmp, count);

    if (ret >= 0)
        rc = copy_to_user(buff, tmp, count);
    
    kfree(tmp);

    return ret;
}

static int elan_ktf2k_ts_poll(struct i2c_client *client)
{
	struct elan_ktf2k_ts_data *ts = private_ts;
	int status = 0, retry = 10;

	do {
		//Elan modify
		//status = gpio_get_value(ts->intr_gpio);
		status = mt_get_gpio_in(ts->intr_gpio);
		dev_err(&client->dev, "%s: status = %d\n", __func__, status);
		retry--;
		mdelay(20);
	} while (status == 1 && retry > 0);

	dev_err(&client->dev, "[elan]%s: poll interrupt status %s\n",
			__func__, status == 1 ? "high" : "low");
	return (status == 0 ? 0 : -ETIMEDOUT);
}

static int elan_ktf2k_ts_get_data(struct i2c_client *client, uint8_t *cmd,
			uint8_t *buf, size_t size)
{
	int rc;

	dev_dbg(&client->dev, "[elan]%s: enter\n", __func__);

	if (buf == NULL)
		return -EINVAL;

	if ((i2c_master_send(client, cmd, 4)) != 4) {
		dev_err(&client->dev,
			"[elan]%s: i2c_master_send failed\n", __func__);
		return -EINVAL;
	}

	rc = elan_ktf2k_ts_poll(client);
	if (rc < 0)
		return -EINVAL;
	else {
		if (i2c_master_recv(client, buf, size) != size ||
		    buf[0] != CMD_S_PKT)
			return -EINVAL;
	}

	return 0;
}

static int fw_packet_handler(struct i2c_client *client)
{
	struct elan_ktf2k_ts_data *ts = private_ts;
	int rc;
	int major, minor;
	uint8_t cmd[] = {CMD_R_PKT, 0x00, 0x00, 0x01};
	uint8_t cmd_x[] = {0x53, 0x60, 0x00, 0x00}; /*Get x resolution*/
	uint8_t cmd_y[] = {0x53, 0x63, 0x00, 0x00}; /*Get y resolution*/
	uint8_t cmd_id[] = {0x53, 0xf0, 0x00, 0x01}; /*Get firmware ID*/
	uint8_t buf_recv[4] = {0};
// Firmware version
	rc = elan_ktf2k_ts_get_data(client, cmd, buf_recv, 4);
	if (rc < 0)
		return rc;
	major = ((buf_recv[1] & 0x0f) << 4) | ((buf_recv[2] & 0xf0) >> 4);
	minor = ((buf_recv[2] & 0x0f) << 4) | ((buf_recv[3] & 0xf0) >> 4);
	ts->fw_ver = major << 8 | minor;
	FW_VERSION = ts->fw_ver;
// X Resolution
	rc = elan_ktf2k_ts_get_data(client, cmd_x, buf_recv, 4);
	if (rc < 0)
		return rc;
	minor = ((buf_recv[2])) | ((buf_recv[3] & 0xf0) << 4);
	ts->x_resolution =minor;
	X_RESOLUTION = ts->x_resolution;
// Y Resolution	
	rc = elan_ktf2k_ts_get_data(client, cmd_y, buf_recv, 4);
	if (rc < 0)
		return rc;
	minor = ((buf_recv[2])) | ((buf_recv[3] & 0xf0) << 4);
	ts->y_resolution =minor;
	Y_RESOLUTION = ts->y_resolution;
// Firmware ID
	rc = elan_ktf2k_ts_get_data(client, cmd_id, buf_recv, 4);
	if (rc < 0)
		return rc;
	major = ((buf_recv[1] & 0x0f) << 4) | ((buf_recv[2] & 0xf0) >> 4);
	minor = ((buf_recv[2] & 0x0f) << 4) | ((buf_recv[3] & 0xf0) >> 4);
	ts->fw_id = major << 8 | minor;
	FW_ID = ts->fw_id;

	TPD_DEBUG(KERN_INFO "[elan] %s: firmware version: 0x%4.4x\n",
			__func__, ts->fw_ver);
	TPD_DEBUG(KERN_INFO "[elan] %s: firmware ID: 0x%4.4x\n",
			__func__, ts->fw_id);
	TPD_DEBUG(KERN_ERR "[elan] %s: x resolution: %d, y resolution: %d\n",
			__func__, ts->x_resolution, ts->y_resolution);
	
	return 0;
}

static long elan_iap_ioctl( struct file *filp,    unsigned int cmd, unsigned long arg){

	int __user *ip = (int __user *)arg;
	TPD_DEBUG("[ELAN]into elan_iap_ioctl\n");
	TPD_DEBUG("cmd value %x\n",cmd);
	
	switch (cmd) {        
		case IOCTL_I2C_SLAVE: 
			private_ts->client->addr = (int __user)arg;
			break;   
		case IOCTL_MAJOR_FW_VER:   
			 return vendor_version;
			break;        
		case IOCTL_MINOR_FW_VER:   
			 return panel_version;
			break;        
		case IOCTL_RESET:
			 tpd_reset();
			break;
		case IOCTL_IAP_MODE_LOCK:
			work_lock=1;
			break;
		case IOCTL_IAP_MODE_UNLOCK:
			work_lock=0;
			//Elan modify
			//if (gpio_get_value(private_ts->intr_gpio))
			if (mt_get_gpio_in(private_ts->intr_gpio))
    			{
        			enable_irq(private_ts->client->irq);
			}
			break;
		case IOCTL_CHECK_RECOVERY_MODE:
			return RECOVERY;
			break;
		case IOCTL_FW_VER:
			//fw_packet_handler(private_ts->client);
			ektf2136_ts_get_fw_version();
			return FW_VERSION;
			break;
		case IOCTL_X_RESOLUTION:
			//fw_packet_handler(private_ts->client);
			return X_RESOLUTION;
			break;
		case IOCTL_Y_RESOLUTION:
			//fw_packet_handler(private_ts->client);
			return Y_RESOLUTION;
			break;
		case IOCTL_FW_ID:
			//fw_packet_handler(private_ts->client);
			return FW_ID;
			break;
		case IOCTL_I2C_INT:
			put_user(mt_get_gpio_in(private_ts->intr_gpio), ip);
			break;			
		default:            
			break;   
	}       
	return 0;
}

#endif

static int __devinit tpd_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    TPD_DEBUG("[elan::] tpd_probe... \n");
    int retval = 0;
    unsigned char data[4] = {0}; 
    i2c_client = client;
    hwPowerOn(MT65XX_POWER_LDO_VGP4, VOL_2800, "TP");
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
    if (i2c_master_recv(i2c_client, data, 4) <= 0){ 
        TPD_DEBUG("I2C transfer error, line: %d\n", __LINE__);
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
        TPD_DEBUG(TPD_DEVICE " failed to create update_firmware_thread thread: %d\n", retval);
    }
#endif

	
    thread = kthread_run(touch_event_handler, 0, TPD_DEVICE);

    if (IS_ERR(thread)) {
        retval = PTR_ERR(thread);
        TPD_DEBUG(" failed to create kernel thread: %d\n", retval);
    }

#ifdef SW_FIRMWARE_UPDATE
	struct elan_ktf2k_ts_data *ts;
	ts = kzalloc(sizeof(struct elan_ktf2k_ts_data), GFP_KERNEL);
	if (ts == NULL) {
		TPD_DEBUG(KERN_ERR "[elan::] %s: allocate elan_ktf2k_ts_data failed\n", __func__);
        return 0;
	}
	private_ts = ts;
    ts->client = i2c_client;
    ts->intr_gpio = GPIO_CTP_EINT_PIN;
    ts->firmware.minor = MISC_DYNAMIC_MINOR;
    ts->firmware.name = "elan-iap";
    ts->firmware.fops = &elan_touch_fops;
    ts->firmware.mode = S_IRWXUGO; 
	   	
    if (misc_register(&ts->firmware) < 0) {
        TPD_DEBUG("[elan::]misc_register failed!!");
    }else {
        TPD_DEBUG("[elan::]misc_register finished!!");
    }
#endif
    mt65xx_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);
    return 0;
err_detect_failed:
    printk("tpd_i2c_probe ERROR\n");
    i2c_client = NULL;
    //tinno_tp_power_off();
    hwPowerDown(MT65XX_POWER_LDO_VGP4, "TP");
   return 1;
}

static int __devexit tpd_remove(struct i2c_client *client)
{
    TPD_DEBUG("[elan::] TPD removed\n");
    return 0;
}
static int tpd_local_init(void)
{
    ektf_boot_mode = get_boot_mode();
    // Software reset mode will be treated as normal boot
    if(ektf_boot_mode==3) ektf_boot_mode = NORMAL_BOOT;
//Ivan
    TPD_DEBUG("tpd_local_init boot mode = %d\n",ektf_boot_mode); 
    if(i2c_add_driver(&tpd_i2c_driver) != 0)
    {
        TPD_DEBUG("[elan::] unable to add i2c driver.\n");
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
    TPD_DEBUG("[elan::] end %s, %d\n", __FUNCTION__, __LINE__);
    return 0;
}
static int tpd_resume(struct i2c_client *client)
{
    //unsigned char sleep_out[] = {0x54, 0x58, 0x00, 0x01};
    int retval = 0;
	 if (i2c_client == NULL)
	return;
	if(!ctp_suspend){
		TPD_DEBUG("ctp not suspend, resume depends on suspend...!\n");
		 return retval;
	}
    hwPowerOn(MT65XX_POWER_LDO_VGP4, VOL_2800, "TP");
    tpd_reset(); 
    mt65xx_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);
    //tpd_wr_datas(sleep_out, sizeof(sleep_out));
    TPD_DEBUG("[elan::] TPD wake up\n");
    int i = 0; int err = -1;
	for(i;i<5;i++){
        err = ektf2136_ts_get_fw_version();
        if (err == 1)
            break;
		else
            tpd_reset();
	 msleep(200); 
    }
	ctp_suspend = -1;
    return retval;
}
static int tpd_suspend(struct i2c_client *client, pm_message_t message)
{
     if (i2c_client == NULL)
	return;
	if(work_lock) //tp is upgrading...
		return 0;
    unsigned char sleep_in[] = {0x54, 0x50, 0x00, 0x01};
    int retval = 0;    
    TPD_DEBUG("[elan::] enter sleep!!\n");
    tpd_wr_datas(sleep_in, sizeof(sleep_in));
	//edit by Magnum 2012-10-25 solve idle current > 3mA
	mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ZERO);
    mt65xx_eint_mask(CUST_EINT_TOUCH_PANEL_NUM);
    hwPowerDown(MT65XX_POWER_LDO_VGP4, "TP");
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
    TPD_DEBUG("MediaTek elan touch panel driver exit\n");
    tpd_driver_remove(&tpd_device_driver);
}
module_init(tpd_driver_init);
module_exit(tpd_driver_exit);
MODULE_DESCRIPTION("CTP driver for elong on MT6575 platform");
MODULE_AUTHOR("tinno");
MODULE_LICENSE("GPL");
