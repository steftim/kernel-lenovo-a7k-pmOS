include $(srctree)/drivers/misc/mediatek/Makefile.custom

# Linux driver folder
ccflags-y += -I$(srctree)/drivers/misc/mediatek/mach/$(MTK_PLATFORM)/$(ARCH_MTK_PROJECT)/touchpanel/GT9XX_aio/
ccflags-y += -I$(srctree)/arch/arm/mach-$(MTK_PLATFORM)/$(ARCH_MTK_PROJECT)/touchpanel/GT9XX_aio/
ccflags-y += -I$(srctree)/drivers/input/touchscreen/mediatek/GT9XX_aio/
ccflags-y += -I$(srctree)/drivers/input/touchscreen/mediatek/

obj-y	+=  goodix_tool.o
obj-y	+=  gt9xx_driver.o
obj-y	+=  gt9xx_update.o
obj-y	+=  touchscreen.o
obj-$(CONFIG_TOUCHSCREEN_SMARTWAKE)		+= smartwake.o
#obj-$(CONFIG_TOUCHSCREEN_DOUBLETAP2WAKE)	+= doubletap2wake.o

