#include "tpd_custom_ektf2136.h"
extern  int ektf2136_ts_get_fw_version(void);

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


#ifdef      SUPPORT_UPDATE_FW
static unsigned char s_ctp_fw[] =
{
    #include TP_LIB_NAME
};
#define PAGE_SIZE       132
#define ACK_OK          0xaa

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

int update_firmware_thread(void *priv)
{
    TPD_DEBUG("[elan::] enter update_firmware_thread\n");
    work_lock=1;
    update_firmware();
	kthread_should_stop();
    work_lock=0;

	return NULL;
}
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

struct file_operations elan_touch_fops = {    
    .open =         elan_iap_open,    
    .write =        elan_iap_write,    
    .read = 	elan_iap_read,    
    .release =	elan_iap_release,    
	.unlocked_ioctl = elan_iap_ioctl, 
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
			 return ektf_vendor_version;
			break;        
		case IOCTL_MINOR_FW_VER:   
			 return ektf_panel_version;
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

int  ektf2136_register_misc(void)
{
	struct elan_ktf2k_ts_data *ts;
	ts = kzalloc(sizeof(struct elan_ktf2k_ts_data), GFP_KERNEL);
	if (ts == NULL) {
		TPD_DEBUG(KERN_ERR "[elan::] %s: allocate elan_ktf2k_ts_data failed\n", __func__);
	    return -1;
	}
	private_ts = ts;
	ts->client = ektf2136_i2c_client;
	ts->intr_gpio = GPIO_CTP_EINT_PIN;
	ts->firmware.minor = MISC_DYNAMIC_MINOR;
	ts->firmware.name = "elan-iap";
	ts->firmware.fops = &elan_touch_fops;
	ts->firmware.mode = S_IRWXUGO; 
	   	
	if (misc_register(&ts->firmware) < 0) {
	    TPD_DEBUG("[elan::]misc_register failed!!");
	    return -2;
	}else {
	    TPD_DEBUG("[elan::]misc_register finished!!");
	     return 1;
	}
}
#endif


