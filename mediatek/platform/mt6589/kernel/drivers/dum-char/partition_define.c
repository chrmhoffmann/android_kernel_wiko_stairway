#include <linux/module.h>
#include "partition_define.h"
struct excel_info PartInfo[PART_NUM]={
			{"preloader",6291456,0x0, EMMC, 0,BOOT_1},
			{"mbr",524288,0x600000, EMMC, 0,USER},
			{"ebr1",524288,0x680000, EMMC, 1,USER},
			{"pmt",4194304,0x700000, EMMC, 0,USER},
			{"pro_info",3145728,0xb00000, EMMC, 0,USER},
			{"nvram",5242880,0xe00000, EMMC, 0,USER},
			{"protect_f",10485760,0x1300000, EMMC, 2,USER},
			{"protect_s",10485760,0x1d00000, EMMC, 3,USER},
			{"seccfg",131072,0x2700000, EMMC, 0,USER},
			{"uboot",393216,0x2720000, EMMC, 0,USER},
			{"bootimg",6291456,0x2780000, EMMC, 0,USER},
			{"recovery",6291456,0x2d80000, EMMC, 0,USER},
			{"sec_ro",6291456,0x3380000, EMMC, 4,USER},
			{"misc",524288,0x3980000, EMMC, 0,USER},
			{"logo",3145728,0x3a00000, EMMC, 0,USER},
			{"ebr2",524288,0x3d00000, EMMC, 0,USER},
			{"expdb",10485760,0x3d80000, EMMC, 0,USER},
			{"android",681574400,0x4780000, EMMC, 5,USER},
			{"cache",132120576,0x2d180000, EMMC, 6,USER},
			{"usrdata",1073741824,0x34f80000, EMMC, 7,USER},
			{"fat",0,0x74f80000, EMMC, 8,USER},
			{"bmtpool",22020096,0xFFFF00a8, EMMC, 0,USER},
 };
EXPORT_SYMBOL(PartInfo);

#ifdef  MTK_EMMC_SUPPORT
struct MBR_EBR_struct MBR_EBR_px[MBR_COUNT]={
	{"mbr", {1, 2, 3, 4, }},
	{"ebr1", {5, 6, 7, }},
	{"ebr2", {8, }},
};

EXPORT_SYMBOL(MBR_EBR_px);
#endif

