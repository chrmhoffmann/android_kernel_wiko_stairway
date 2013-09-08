# Display parameters shown under Settings -> About Phone
ccflags-y += -I$(MTK_ROOT_CUSTOM)/$(MTK_PROJECT)/common
ifeq ($(CONFIG_MTK_MMC),y)
  CONFIG_MTK_USIF = y
else
	ifeq ($(CONFIG_MTK_MTD_NAND),y)
  	CONFIG_MTK_USIF = y
  	else
  	CONFIG_MTK_USIF = n
  	endif
endif

obj-$(CONFIG_MTK_USIF) := DumChar.o
DumChar-objs :=  dumchar.o partition_define.o env.o
