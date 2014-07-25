
obj-y += misc.o
obj-y += mmc.o
obj-$(CONFIG_REGULATOR_DA9024) += regulator.o
obj-$(CONFIG_REGULATOR_RICOH619) += board-power-619.o

obj-$(CONFIG_LCD_TRULY_TFT240240_2_E) += lcd-truly_tft240240_2_e.o
obj-$(CONFIG_LCD_BYD_8991FTGF) += lcd-8991.o
obj-$(CONFIG_LCD_CV90_M5377_P30) += lcd-mirasol.o
obj-$(CONFIG_LCD_OTM8012A_TFT1P6449) += lcd-otm8012a.o
obj-$(CONFIG_LCD_BYD_9177AA) += lcd-9177aa.o
obj-$(CONFIG_LCD_TRULY_TDO_HD0499K) += lcd-truly_tdo_hd0499k.o
obj-$(CONFIG_LCD_LH155) += lcd-lh155.o

obj-$(CONFIG_SOUND_OSS_XBURST)		    += sound.o
obj-$(CONFIG_VIDEO_OVISP) += camera.o

obj-$(CONFIG_BCM_PM_CORE) += bcm_power_control.o

ifdef CONFIG_SLCD_SUSPEND_ALARM_WAKEUP_REFRESH
obj-y += pm_slcd.o
else
obj-y += pm.o
endif
