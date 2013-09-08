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

#include "imx092mipiraw_Sensor.h"
#include "imx092mipiraw_Camera_Sensor_para.h"
#include "imx092mipiraw_CameraCustomized.h"

#define IMX092_OTP_DEBUG

#define IMX092_TRULY_MID 0x01

extern int iWriteReg(u16 a_u2Addr , u32 a_u4Data , u32 a_u4Bytes , u16 i2cId);
#define IMX092MIPI_write_cmos_sensor(addr, para) iWriteReg((u16) addr , (u32) para , 1, IMX092MIPI_WRITE_ID)
extern kal_uint16 IMX092MIPI_read_cmos_sensor(kal_uint32 addr);

struct imx092_otp_struct {
    uint customer_id;
    uint year;
    uint month;
    uint day;
    uint lens_id;
    uint vcm_id;
    uint driver_ic;
    uint32_t current_r;
    uint32_t current_gr;
    uint32_t current_gb;
    uint32_t current_b;
    uint32_t golden_r;
    uint32_t golden_gr;
    uint32_t golden_gb;
    uint32_t golden_b;
    uint32_t rg_ratio;
    uint32_t bg_ratio;
    uint32_t gg_ratio;
};

uint32_t imx092_R_gain1=0x100,imx092_B_gain1=0x100,imx092_G_gain1=0x100;

// R/G and B/G of typical camera module is defined here
//add for distinguish diffrent moudle Truly or Liteon
uint32_t IMX092_RG_Ratio_typical=0x00;
uint32_t IMX092_BG_Ratio_typical=0x00;
uint32_t IMX092_GG_Ratio_typical=0x00;

uint32_t IMX092_RG_Ratio_typical_Truly=0x0222;//546
uint32_t IMX092_BG_Ratio_typical_Truly=0x022F;//559
uint32_t IMX092_GG_Ratio_typical_Truly=0x0400;//1024


// index: index of otp group. (0, 1, 2)
// return: 0, group index is empty
// return: 1, group index has invalid data
// return: 2, group index has valid data
int imx092_check_otp_awb(uint index)
{
    printk("[Truly] imx092_check_otp_awb!\n");
    uint flag, bank, address = 0x3507;

    // select bank
    bank = 0x04 + 5*index;//{0x00,0x05,0x0A},AF reserve,AWB valid.//5 banks in 1 group.
    address = address + 8*bank;
    printk("[Truly] test address = 0x%x\n",address);
    IMX092MIPI_write_cmos_sensor(0x34C9, bank);

    // read flag
    flag = IMX092MIPI_read_cmos_sensor(address) & 0xC0;
    if (!flag)
    {
        printk("[Truly] imx092_check_otp_awb error, empty data!\n");
        return 0;
    }
    else if (flag == 0x40)
    {
        printk("[Truly] imx092_check_otp_awb success,have valid data!\n");
        return 2;
    }
    else
    {
        printk("[Truly] imx092_check_otp_awb err,have invalid data!\n");
        return 1;
    }
}


// index: index of otp group. (0, 1, 2)
// return:0,
int imx092_read_otp_wb(uint index, struct imx092_otp_struct* otp)
{
    printk("[Truly] imx092_read_otp_wb++!\n");
    uint32_t address = 0x3500, bank;
    uint32_t arr[40]={0};
    int i,j;

    for (i=0; i<5; i++)
    {
        // select bank
        bank = 5*index + i;
        address = address + 8*bank;
        j = 8*i;
        IMX092MIPI_write_cmos_sensor(0x34C9, bank);

        arr[0+j] = IMX092MIPI_read_cmos_sensor(address);
        arr[1+j] = IMX092MIPI_read_cmos_sensor(address+1);
        arr[2+j] = IMX092MIPI_read_cmos_sensor(address+2);
        arr[3+j] = IMX092MIPI_read_cmos_sensor(address+3);
        arr[4+j] = IMX092MIPI_read_cmos_sensor(address+4);
        arr[5+j] = IMX092MIPI_read_cmos_sensor(address+5);
        arr[6+j] = IMX092MIPI_read_cmos_sensor(address+6);
        arr[7+j] = IMX092MIPI_read_cmos_sensor(address+7);
    }
#ifdef IMX092_OTP_DEBUG
    for (i=0; i<40; i++)
    printk("[Truly] imx092_read_otp_wb:arr[%d] == %d\n",i,arr[i]);
#endif

    otp->customer_id = arr[0];
    otp->year = arr[1];
    otp->month = arr[2];
    otp->day = arr[3];
    otp->lens_id = arr[4];
    otp->vcm_id = arr[5];
    otp->driver_ic = arr[6];
    otp->current_r = (arr[16]<<8) + arr[17];
    otp->current_gr = (arr[18]<<8) + arr[19];
    otp->current_gb = (arr[20]<<8) + arr[21];
    otp->current_b = (arr[22]<<8) + arr[23];
    otp->golden_r = (arr[24]<<8) + arr[25];
    otp->golden_gr = (arr[26]<<8) + arr[27];
    otp->golden_gb = (arr[28]<<8) + arr[29];
    otp->golden_b = (arr[30]<<8) + arr[31];
    otp->rg_ratio = (arr[32]<<8) + arr[33];
    otp->bg_ratio = (arr[34]<<8) + arr[35];
    otp->gg_ratio = (arr[36]<<8) + arr[37];

    printk("[Truly] imx092_read_otp_wb--!\n");
    return 0;
}


// call this function after imx092 initialization
// return value: 0, update success
//               1, no OTP
int imx092_update_otp_wb(void)
{
    printk("[Truly] imx092_update_otp_wb++\n");
    uint i, temp, otp_index;
    struct imx092_otp_struct current_otp;
    uint32_t R_gain, G_gain, B_gain, G_gain_R, G_gain_B;
    uint32_t golden_g, current_g;
	
    // R/G and B/G of current camera module is read out from sensor OTP
    // check first wb OTP with valid data
    for(i=0;i<3;i++) {
        temp = imx092_check_otp_awb(i);
        if (temp == 2) {
            otp_index = i;
            break;
        }
    }
    if(i<0) {
        // no valid wb OTP data
        printk("[Truly] imx092_update_otp_wb! no valid wb OTP data\n");
        return 1;
    }

    memset(&current_otp, 0, sizeof(current_otp));
    imx092_read_otp_wb(otp_index, &current_otp);

#ifdef IMX092_OTP_DEBUG
    printk("[Truly] #########current_otp.customer_id = 0x%x\n",current_otp.customer_id);
    printk("[Truly] #########current_otp.year = 0x%x\n",current_otp.year);
    printk("[Truly] #########current_otp.month = 0x%x\n",current_otp.month);
    printk("[Truly] #########current_otp.day = 0x%x\n",current_otp.day);
    printk("[Truly] #########current_otp.lens_id = 0x%x\n",current_otp.lens_id);
    printk("[Truly] #########current_otp.vcm_id = 0x%x\n",current_otp.vcm_id);
    printk("[Truly] #########current_otp.driver_ic = 0x%x\n",current_otp.driver_ic);
    printk("[Truly] #########current_otp.current_r = 0x%x\n",current_otp.current_r);
    printk("[Truly] #########current_otp.current_gr = 0x%x\n",current_otp.current_gr);
    printk("[Truly] #########current_otp.current_gb = 0x%x\n",current_otp.current_gb);
    printk("[Truly] #########current_otp.current_b = 0x%x\n",current_otp.current_b);
    printk("[Truly] #########current_otp.golden_r = 0x%x\n",current_otp.golden_r);
    printk("[Truly] #########current_otp.golden_gr = 0x%x\n",current_otp.golden_gr);
    printk("[Truly] #########current_otp.golden_gb = 0x%x\n",current_otp.golden_gb);
    printk("[Truly] #########current_otp.golden_b = 0x%x\n",current_otp.golden_b);
#endif

    //add for distinguish diffrent moudle Truly or other
    if(current_otp.customer_id == IMX092_TRULY_MID)
    {
        IMX092_RG_Ratio_typical = IMX092_RG_Ratio_typical_Truly;
        IMX092_BG_Ratio_typical = IMX092_BG_Ratio_typical_Truly;
        IMX092_GG_Ratio_typical = IMX092_GG_Ratio_typical_Truly;
    }
    else
    {
        printk("[Truly] imx092_update_otp_wb,read module_id err.\n");
        return 1;
    }

    //calculate gain
    //0x400 = 1x gain
    printk("[Truly] #########current_otp.rg_ratio = 0x%x\n",current_otp.rg_ratio);
    printk("[Truly] #########current_otp.bg_ratio = 0x%x\n",current_otp.bg_ratio);
    printk("[Truly] #########current_otp.gg_ratio = 0x%x\n",current_otp.gg_ratio);
    printk("[Truly] #########IMX092_RG_Ratio_typical = 0x%x\n",IMX092_RG_Ratio_typical);
    printk("[Truly] #########IMX092_BG_Ratio_typical = 0x%x\n",IMX092_BG_Ratio_typical);
    printk("[Truly] #########IMX092_GG_Ratio_typical = 0x%x\n",IMX092_GG_Ratio_typical);

#if 0   //These awb calibration code from truly
    golden_g = (current_otp.golden_gr + current_otp.golden_gb) / 2;
    current_g = (current_otp.current_gr + current_otp.current_gb) / 2;

    if(!golden_g || !current_g || !current_otp.golden_r || !current_otp.golden_b || !current_otp.current_r || !current_otp.current_b)
    {
        printk("[Truly] ######### WB update Err !");
        return 1;
    }

    current_otp.rg_ratio = 512 * current_otp.golden_r * current_g /( golden_g * current_otp.current_r );
    current_otp.bg_ratio = 512 * current_otp.golden_b * current_g /( golden_g * current_otp.current_b );
    printk("[Truly] #########current_otp.rg_ratio = 0x%x\n",current_otp.rg_ratio);
    printk("[Truly] #########current_otp.bg_ratio = 0x%x\n",current_otp.bg_ratio);

    if(current_otp.rg_ratio >= 512)
    {
        if (current_otp.bg_ratio >= 512)
        {
            R_gain = 0x100 * current_otp.rg_ratio / 512;
            G_gain = 0x100;
            B_gain = 0x100 * current_otp.bg_ratio / 512;
        }
        else
        {
            R_gain = 0x100 * current_otp.rg_ratio / current_otp.bg_ratio;
            G_gain = 0x100 * 512 / current_otp.bg_ratio;
            B_gain = 0x100;
        }
    }
    else
    {
        if (current_otp.bg_ratio >= 512)
        {
            R_gain = 0x100;
            G_gain = 0x100 * 512 / current_otp.rg_ratio;
            B_gain = 0x100 * current_otp.bg_ratio / current_otp.rg_ratio;
        }
        else
        {
            G_gain_R = 0x100 * 512 / current_otp.rg_ratio;
            G_gain_B = 0x100 * 512 / current_otp.bg_ratio;
            if(G_gain_R > G_gain_B )
            {
                R_gain = 0x100;
                G_gain = G_gain_R;
                B_gain = 0x100 * current_otp.bg_ratio / current_otp.rg_ratio;
            }
            else
            {
                R_gain = 0x100 * current_otp.rg_ratio / current_otp.bg_ratio;
                G_gain = G_gain_B;
                B_gain = 0x100;
            }
        }
    }

#else
    if(current_otp.bg_ratio < IMX092_BG_Ratio_typical)
    {
        if (current_otp.rg_ratio < IMX092_RG_Ratio_typical)
        {
            G_gain = 0x100;
            B_gain = 0x100 * IMX092_BG_Ratio_typical / current_otp.bg_ratio;
            R_gain = 0x100 * IMX092_RG_Ratio_typical / current_otp.rg_ratio;
        }
        else
        {
            R_gain = 0x100;
            G_gain = 0x100 * current_otp.rg_ratio / IMX092_RG_Ratio_typical;
            B_gain = G_gain * IMX092_BG_Ratio_typical / current_otp.bg_ratio;
        }
    }
    else
    {
        if (current_otp.rg_ratio < IMX092_RG_Ratio_typical)
        {
            B_gain = 0x100;
            G_gain = 0x100 * current_otp.bg_ratio / IMX092_BG_Ratio_typical;
            R_gain = G_gain * IMX092_RG_Ratio_typical / current_otp.rg_ratio;
        }
        else
        {
            G_gain_B = 0x100 * current_otp.bg_ratio / IMX092_BG_Ratio_typical;
            G_gain_R = 0x100 * current_otp.rg_ratio / IMX092_RG_Ratio_typical;
            if(G_gain_B > G_gain_R )
            {
                B_gain = 0x100;
                G_gain = G_gain_B;
                R_gain = G_gain * IMX092_RG_Ratio_typical / current_otp.rg_ratio;
            }
            else
            {
                R_gain = 0x100;
                G_gain = G_gain_R;
                B_gain = G_gain * IMX092_BG_Ratio_typical / current_otp.bg_ratio;
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
#endif
    // write sensor wb gain to registers
    imx092_R_gain1=R_gain&0xFF;
    imx092_G_gain1=G_gain&0xFF;
    imx092_B_gain1=B_gain&0xFF;

#ifdef IMX092_OTP_DEBUG
    printk("[Truly] R_gain == 0x%x\n",R_gain);
    printk("[Truly] G_gain == 0x%x\n",G_gain);
    printk("[Truly] B_gain == 0x%x\n",B_gain);

    printk("[Truly] imx092_R_gain1 == 0x%x\n",imx092_R_gain1);
    printk("[Truly] imx092_G_gain1 == 0x%x\n",imx092_G_gain1);
    printk("[Truly] imx092_B_gain1 == 0x%x\n",imx092_B_gain1);
#endif

    printk("[Truly] imx092_update_otp_wb--\n");
    return 0;
}


// R_gain: red gain of sensor AWB, 0x400 = 1
// G_gain: green gain of sensor AWB, 0x400 = 1
// B_gain: blue gain of sensor AWB, 0x400 = 1
// return 0
int imx092_update_awb_gain()//uint32_t R_gain, uint32_t G_gain, uint32_t B_gain)
{
    printk("[Truly] imx092_update_awb_gain++!\n");

    if (imx092_update_otp_wb())
    {
        printk("[Truly] imx092_update_awb_gain, update_otp_wb fail!\n");
        return 1;
    }

    //reset the digital gain
    IMX092MIPI_write_cmos_sensor(0x020F, imx092_G_gain1);
    IMX092MIPI_write_cmos_sensor(0x0211, imx092_R_gain1);
    IMX092MIPI_write_cmos_sensor(0x0213, imx092_B_gain1);
    IMX092MIPI_write_cmos_sensor(0x0215, imx092_G_gain1);
    printk("[Truly] imx092_update_awb_gain--!\n");
    return 0;
}


int imx092_check_mid(uint mid, uint lens_id)
{
    printk("[Truly] imx092_check_mid++,mid == %d,lens_id == %d\n",mid,lens_id);
    uint i,temp, otp_index;
    struct imx092_otp_struct current_otp;
	
    // R/G and B/G of current camera module is read out from sensor OTP
    // check first wb OTP with valid data
    for(i=4;i>=0;i--) {
        temp = imx092_check_otp_awb(i);
        if (temp == 2) {
            otp_index = i;
            break;
        }
    }
    if(i<0) {
        // no valid wb OTP data
        printk("[Truly] imx092_check_mid! no valid wb OTP data\n");
        return 1;
    }

    memset(&current_otp, 0, sizeof(current_otp));
    imx092_read_otp_wb(otp_index, &current_otp);

    //add for distinguish diffrent moudle Truly or Truly
    printk("[Truly] #########current_otp->customer_id = 0x%x\n",current_otp.customer_id);
    printk("[Truly] #########current_otp->lens_id = 0x%x\n",current_otp.lens_id);

    if(((current_otp.customer_id == mid)||(current_otp.customer_id == 0x02))&&(current_otp.lens_id == lens_id))
    {
        return 0;
    }
    else
        return 1;
}

