/*===========================================================================

                        EDIT HISTORY FOR V11

when              comment tag        who                  what, where, why                           
----------    ------------     -----------      --------------------------      

===========================================================================*/
/* Copyright (c) 2010, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
*/

#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <asm/atomic.h>
	
#include "kd_camera_hw.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"

#include "imx091mipiraw_Sensor.h"
#include "imx091mipiraw_Camera_Sensor_para.h"
#include "imx091mipiraw_CameraCustomized.h"

#define IMX091_OTP_DEBUG

#define IMX091_SUNNY_MID 0x01

extern int iWriteReg(u16 a_u2Addr , u32 a_u4Data , u32 a_u4Bytes , u16 i2cId);
#define IMX091MIPI_write_cmos_sensor(addr, para) iWriteReg((u16) addr , (u32) para , 1, IMX091MIPI_WRITE_ID)
extern kal_uint16 IMX091MIPI_read_cmos_sensor(kal_uint32 addr);

struct imx091_otp_struct {
    uint year;
    uint date;
    uint customer_id;
    uint lens_id;
    uint32_t rg_ratio;
    uint32_t bg_ratio;
    uint32_t gg_ratio;
};

uint32_t imx091_R_gain1=0x100,imx091_B_gain1=0x100,imx091_G_gain1=0x100;

// R/G and B/G of typical camera module is defined here
//add for distinguish diffrent moudle Truly or Liteon
uint32_t IMX091_RG_Ratio_typical=0x00;
uint32_t IMX091_BG_Ratio_typical=0x00;
uint32_t IMX091_GG_Ratio_typical=0x00;

//uint32_t IMX091_RG_Ratio_typical_Sunny=0x0275;//629
//uint32_t IMX091_BG_Ratio_typical_Sunny=0x025B;//603
//uint32_t IMX091_GG_Ratio_typical_Sunny=0x0400;//1024

//new blue lens
uint32_t IMX091_RG_Ratio_typical_Sunny=0x0246;//582
uint32_t IMX091_BG_Ratio_typical_Sunny=0x0261;//609
uint32_t IMX091_GG_Ratio_typical_Sunny=0x0400;//1024


// index: index of otp group. (0, 1)
// return: 0, group index is empty
// return: 1, group index has valid data
int imx091_check_otp_awb(uint index)
{
    printk("[Sunny] imx091_check_otp_awb!\n");
    uint flag, bank, address = 0x3500;

    // select bank
    bank = 0x01 + 3*index;//{0x01,0x04,0x07,0x0A,0x0D},AF reserve,AWB valid.//3 banks in 1 group.
    address = address + 8*bank;
    IMX091MIPI_write_cmos_sensor(0x34C9, bank);

    // read flag
    flag = IMX091MIPI_read_cmos_sensor(address);
    if (!flag) {
        printk("[Sunny] imx091_check_otp_awb error,no valid data!\n");
        return 0;
    }
    else {
        printk("[Sunny] imx091_check_otp_awb success,have valid data!\n");
        return 1;
    }
}


// index: index of otp group. (0, 1, 2)
// return:0,
int imx091_read_otp_wb(uint index, struct imx091_otp_struct* otp)
{
    printk("[Sunny] imx091_read_otp_wb++!\n");
    uint32_t address = 0x3500, bank;
    uint32_t arr[8]={0};
    int i,j;

    for(i=0;i<2;i++)
    {
        // select bank
        bank = 3*index + i;
        address = address + 8*bank;
        j = 4*i;
        IMX091MIPI_write_cmos_sensor(0x34C9, bank);

        arr[0+j] = (IMX091MIPI_read_cmos_sensor(address)<<8)+IMX091MIPI_read_cmos_sensor(address+1);
        arr[1+j] = (IMX091MIPI_read_cmos_sensor(address+2)<<8)+IMX091MIPI_read_cmos_sensor(address+3);
        arr[2+j] = (IMX091MIPI_read_cmos_sensor(address+4)<<8)+IMX091MIPI_read_cmos_sensor(address+5);
        arr[3+j] = (IMX091MIPI_read_cmos_sensor(address+6)<<8)+IMX091MIPI_read_cmos_sensor(address+7);
    }

    #ifdef IMX091_OTP_DEBUG
    printk("[Sunny] imx091_read_otp_wb:arr[0] == %d\n",arr[0]);
    printk("[Sunny] imx091_read_otp_wb:arr[1] == %d\n",arr[1]);
    printk("[Sunny] imx091_read_otp_wb:arr[2] == %d\n",arr[2]);
    printk("[Sunny] imx091_read_otp_wb:arr[3] == %d\n",arr[3]);
    printk("[Sunny] imx091_read_otp_wb:arr[4] == %d\n",arr[4]);
    printk("[Sunny] imx091_read_otp_wb:arr[5] == %d\n",arr[5]);
    printk("[Sunny] imx091_read_otp_wb:arr[6] == %d\n",arr[6]);
    printk("[Sunny] imx091_read_otp_wb:arr[7] == %d\n",arr[7]);
    #endif

    otp->year = arr[0];
    otp->date = arr[1];
    otp->customer_id = arr[2];
    otp->lens_id = arr[3];
    otp->rg_ratio = arr[4];
    otp->bg_ratio = arr[5];
    otp->gg_ratio = arr[6];

    printk("[Sunny] imx091_read_otp_wb--!\n");
    return 0;
}


// call this function after imx091 initialization
// return value: 0, update success
//               1, no OTP
int imx091_update_otp_wb(void)
{
    printk("[Sunny] imx091_update_otp_wb++\n");
    uint i, temp, otp_index;
    struct imx091_otp_struct current_otp;
    uint32_t R_gain, G_gain, B_gain, G_gain_R, G_gain_B;
	
    // R/G and B/G of current camera module is read out from sensor OTP
    // check first wb OTP with valid data
    for(i=4;i>=0;i--) {
        temp = imx091_check_otp_awb(i);
        if (temp == 1) {
            otp_index = i;
            break;
        }
    }
    if(i<0) {
        // no valid wb OTP data
        printk("[Sunny] imx091_update_otp_wb! no valid wb OTP data\n");
        return 1;
    }

    memset(&current_otp, 0, sizeof(current_otp));
    imx091_read_otp_wb(otp_index, &current_otp);

    //add for distinguish diffrent moudle Truly or Sunny
    printk("[Sunny] #########current_otp->customer_id = 0x%x\n",current_otp.customer_id);

    if(current_otp.customer_id == IMX091_SUNNY_MID)
    {
        IMX091_RG_Ratio_typical = IMX091_RG_Ratio_typical_Sunny;
        IMX091_BG_Ratio_typical = IMX091_BG_Ratio_typical_Sunny;
        IMX091_GG_Ratio_typical = IMX091_GG_Ratio_typical_Sunny;
    }
    else
    {
        printk("[Sunny] imx091_update_otp_wb,read module_id err.\n");
        return 1;
    }

    //calculate gain
    //0x400 = 1x gain
    printk("[Sunny] #########current_otp.rg_ratio = 0x%x\n",current_otp.rg_ratio);
    printk("[Sunny] #########current_otp.bg_ratio = 0x%x\n",current_otp.bg_ratio);
    printk("[Sunny] #########current_otp.gg_ratio = 0x%x\n",current_otp.gg_ratio);
    printk("[Sunny] #########IMX091_RG_Ratio_typical = 0x%x\n",IMX091_RG_Ratio_typical);
    printk("[Sunny] #########IMX091_BG_Ratio_typical = 0x%x\n",IMX091_BG_Ratio_typical);
    printk("[Sunny] #########IMX091_GG_Ratio_typical = 0x%x\n",IMX091_GG_Ratio_typical);

    if(current_otp.bg_ratio < IMX091_BG_Ratio_typical)
    {
        if (current_otp.rg_ratio < IMX091_RG_Ratio_typical)
        {
            G_gain = 0x100;
            B_gain = 0x100 * IMX091_BG_Ratio_typical / current_otp.bg_ratio;
            R_gain = 0x100 * IMX091_RG_Ratio_typical / current_otp.rg_ratio;
        }
        else
        {
            R_gain = 0x100;
            G_gain = 0x100 * current_otp.rg_ratio / IMX091_RG_Ratio_typical;
            B_gain = G_gain * IMX091_BG_Ratio_typical / current_otp.bg_ratio;
        }
    }
    else
    {
        if (current_otp.rg_ratio < IMX091_RG_Ratio_typical)
        {
            B_gain = 0x100;
            G_gain = 0x100 * current_otp.bg_ratio / IMX091_BG_Ratio_typical;
            R_gain = G_gain * IMX091_RG_Ratio_typical / current_otp.rg_ratio;
        }
        else
        {
            G_gain_B = 0x100 * current_otp.bg_ratio / IMX091_BG_Ratio_typical;
            G_gain_R = 0x100 * current_otp.rg_ratio / IMX091_RG_Ratio_typical;
            if(G_gain_B > G_gain_R )
            {
                B_gain = 0x100;
                G_gain = G_gain_B;
                R_gain = G_gain * IMX091_RG_Ratio_typical / current_otp.rg_ratio;
            }
            else
            {
                R_gain = 0x100;
                G_gain = G_gain_R;
                B_gain = G_gain * IMX091_BG_Ratio_typical / current_otp.bg_ratio;
            }
        }
    }

    if (R_gain < 0x100)
    {
        R_gain = 0x100;
    }
    if (G_gain < 0x100)
    {
        G_gain = 0x100;
    }
    if (B_gain < 0x100)
    {
        B_gain = 0x100;
    }

    // write sensor wb gain to registers
    imx091_R_gain1=R_gain&0xFF;
    imx091_G_gain1=G_gain&0xFF;
    imx091_B_gain1=B_gain&0xFF;

    #ifdef IMX091_OTP_DEBUG
    printk("[Sunny] R_gain == 0x%x\n",R_gain);
    printk("[Sunny] G_gain == 0x%x\n",G_gain);
    printk("[Sunny] B_gain == 0x%x\n",B_gain);

    printk("[Sunny] imx091_R_gain1 == 0x%x\n",imx091_R_gain1);
    printk("[Sunny] imx091_G_gain1 == 0x%x\n",imx091_G_gain1);
    printk("[Sunny] imx091_B_gain1 == 0x%x\n",imx091_B_gain1);
    #endif

    printk("[Sunny] imx091_update_otp_wb--\n");
    return 0;
}


// R_gain: red gain of sensor AWB, 0x400 = 1
// G_gain: green gain of sensor AWB, 0x400 = 1
// B_gain: blue gain of sensor AWB, 0x400 = 1
// return 0
int imx091_update_awb_gain()//uint32_t R_gain, uint32_t G_gain, uint32_t B_gain)
{
    printk("[Sunny] imx091_update_awb_gain++!\n");

    if (imx091_update_otp_wb())
    {
        printk("[Sunny] imx091_update_awb_gain, update_otp_wb fail!\n");
        return 1;
    }

    //reset the digital gain
    IMX091MIPI_write_cmos_sensor(0x020F, imx091_G_gain1);
    IMX091MIPI_write_cmos_sensor(0x0211, imx091_R_gain1);
    IMX091MIPI_write_cmos_sensor(0x0213, imx091_B_gain1);
    IMX091MIPI_write_cmos_sensor(0x0215, imx091_G_gain1);
    printk("[Sunny] imx091_update_awb_gain--!\n");
    return 0;
}


int imx091_check_mid(uint mid, uint lens_id)
{
    printk("[Sunny] imx091_check_mid++,mid == %d,lens_id == %d\n",mid,lens_id);
    uint i,temp, otp_index;
    struct imx091_otp_struct current_otp;
	
    // R/G and B/G of current camera module is read out from sensor OTP
    // check first wb OTP with valid data
    for(i=4;i>=0;i--) {
        temp = imx091_check_otp_awb(i);
        if (temp == 1) {
            otp_index = i;
            break;
        }
    }
    if(i<0) {
        // no valid wb OTP data
        printk("[Sunny] imx091_check_mid! no valid wb OTP data\n");
        return 1;
    }

    memset(&current_otp, 0, sizeof(current_otp));
    imx091_read_otp_wb(otp_index, &current_otp);

    //add for distinguish diffrent moudle Truly or Sunny
    printk("[Sunny] #########current_otp->customer_id = 0x%x\n",current_otp.customer_id);
    printk("[Sunny] #########current_otp->lens_id = 0x%x\n",current_otp.lens_id);

    if((current_otp.customer_id == mid)&&((current_otp.lens_id == lens_id)||(current_otp.lens_id == 0x02)))
    {
        return 0;
    }
    else
        return 1;
}

