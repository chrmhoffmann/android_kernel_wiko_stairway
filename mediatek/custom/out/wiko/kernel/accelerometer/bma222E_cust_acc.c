#include <linux/types.h>
#include <cust_acc.h>
#include <mach/mt_pm_ldo.h>
int bma222E_cust_acc_power(struct acc_hw *hw, unsigned int on, char* devname)
{
    if (hw->power_id == MT65XX_POWER_NONE)
        return 0;
    if (on)
        return hwPowerOn(hw->power_id, hw->power_vol, devname);
    else
        return hwPowerDown(hw->power_id, devname); 
}
/*---------------------------------------------------------------------------*/
static struct acc_hw bma222E_cust_acc_hw = {
    .i2c_num = 3,
    .direction = 4,			//Ivan original = 6
    .power_id = MT65XX_POWER_NONE,  /*!< LDO is not used */
    .power_vol= VOL_DEFAULT,        /*!< LDO is not used */
    .firlen = 1,                   /*!< don't enable low pass fileter */
    .power = bma222E_cust_acc_power,
};
/*---------------------------------------------------------------------------*/
struct acc_hw* bma222e_get_cust_acc_hw(void) 
{
    return &bma222E_cust_acc_hw;
}
