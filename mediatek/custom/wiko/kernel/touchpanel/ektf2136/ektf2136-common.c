#include "tpd_custom_ektf2136.h"

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
    //msleep(30);
    msleep(10);
    mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ONE);
}

void tpd_wr_datas(unsigned char *buff, int count)
{
    int ret;
    ret = i2c_master_send(ektf2136_i2c_client, buff, count);

    if(ret <= 0) {
        TPD_DEBUG("[elan::] tpd_wr_datas error!\n");
    }
}

void tpd_rd_datas(unsigned char *buff, int count)
{
    int ret;
    ret = i2c_master_recv(ektf2136_i2c_client, buff, count);

    if(ret <= 0) {
        TPD_DEBUG("[elan::] tpd_rd_datas error!\n");
    }
}

int ektf2136_ts_get_fw_version(void)
{
	unsigned char fwbuf[4];
	kal_uint8 rfwver[4] = {0x53, 0x00, 0x00, 0x01};
	tpd_wr_datas(rfwver,4);
	msleep(10);
	tpd_rd_datas(fwbuf,4);	
    TPD_DEBUG("[elan::] ver related datas iapbuf[0]=0x%x, iapbuf[1]=0x%x, iapbuf[2]=0x%x, iapbuf[3]=0x%x\n", fwbuf[0], fwbuf[1], fwbuf[2], fwbuf[3]);
	if (fwbuf[0] == 0x52){
		ektf_vendor_version =((fwbuf[1] & 0x0f) << 4) | ((fwbuf[2] & 0xf0) >> 4);
		ektf_panel_version = ((fwbuf[2] & 0x0f) << 4) | ((fwbuf[3] & 0xf0) >> 4);
		#ifdef SW_FIRMWARE_UPDATE
		FW_VERSION = ektf_vendor_version<< 8 | ektf_panel_version; 
		#endif
		TPD_DEBUG("*** ektf_vendor_version = %x ***\n", ektf_vendor_version);
	    TPD_DEBUG("*** ektf_panel_version = %x ***\n", ektf_panel_version);
		return 1;
	}
	else{
		TPD_DEBUG("*** read version FAIL ***\n");
		return 0;
	}
}