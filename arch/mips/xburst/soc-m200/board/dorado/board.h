#ifndef __BOARD_H__
#define __BOARD_H__
#include <gpio.h>
#include <soc/gpio.h>
//#include <linux/jz_dwc.h>

/* lcd pdata and display panel */
#ifdef CONFIG_FB_JZ_V12
extern struct jzfb_platform_data jzfb_pdata;
#endif
#ifdef CONFIG_LCD_KFM701A21_1A
extern struct platform_device kfm701a21_1a_device;
#endif
#ifdef CONFIG_LCD_LH155
extern struct mipi_dsim_lcd_device	lh155_device;
#endif
#ifdef CONFIG_LCD_BYD_9177AA
extern struct mipi_dsim_lcd_device	byd_9177aa_device;
#endif
#ifdef CONFIG_LCD_TRULY_TDO_HD0499K
extern struct mipi_dsim_lcd_device	truly_tdo_hd0499k_device;
#endif
#ifdef CONFIG_LCD_CV90_M5377_P30
extern struct platform_device cv90_m5377_p30_device;
#endif
#ifdef CONFIG_LCD_BYD_BM8766U
extern struct platform_device byd_bm8766u_device;
#endif
#ifdef CONFIG_LCD_BYD_8991FTGF
extern struct platform_device byd_8991_device;
#endif
#ifdef CONFIG_LCD_TRULY_TFT240240_2_E
extern struct platform_device truly_tft240240_device;
#endif

/* PMU ricoh619 */
#ifdef CONFIG_REGULATOR_RICOH619
#define PMU_IRQ_N		GPIO_PA(3)
#endif /* CONFIG_REGULATOR_RICOH619 */

/* pmu d2041 or 9024 gpio def*/
#define GPIO_PMU_IRQ		GPIO_PA(3)
#define GPIO_GSENSOR_INT1       GPIO_PA(15)

/* about lcdc gpio */
#define GPIO_LCD_PWM		GPIO_PE(1)
#define GPIO_LCD_DISP		GPIO_PE(10)
#define GPIO_LCD_RST		GPIO_PA(12)
#define GPIO_LCD_CS		GPIO_PA(11)
#define GPIO_LCD_RD		GPIO_PC(8)
#define GPIO_BL_PWR_EN		GPIO_PD(26)
#define GPIO_MIPI_IF_SEL	GPIO_PC(1)
#define GPIO_MIPI_RST		GPIO_PE(29)
#define GPIO_LCD_B_SYNC		GPIO_PE(20)
#define GPIO_LCD_SPI_CLK	GPIO_PD(28)
#define GPIO_LCD_SPI_CS		GPIO_PA(11)
#define GPIO_LCD_SPI_DT		GPIO_PE(0)
#define GPIO_LCD_SPI_DR		GPIO_PE(3)
#define GPIO_LCD_EXCLKO		GPIO_PD(15)

extern struct platform_device backlight_device;
/* Digital pulse backlight*/
#ifdef CONFIG_BACKLIGHT_DIGITAL_PULSE
extern struct platform_device digital_pulse_backlight_device;
extern struct platform_digital_pulse_backlight_data bl_data;
#endif
#ifdef CONFIG_BACKLIGHT_PWM
extern struct platform_device backlight_device;
#endif
#ifndef CONFIG_BOARD_NAME
#define CONFIG_BOARD_NAME "dorado"
#endif


extern struct jzmmc_platform_data inand_pdata;
extern struct jzmmc_platform_data tf_pdata;
extern struct jzmmc_platform_data sdio_pdata;

/**
 * KEY gpio
 **/
/* #define GPIO_HOME		GPIO_PD(18) */
/* #define ACTIVE_LOW_HOME		1 */

#define GPIO_VOLUMEDOWN         GPIO_PD(18)
#define ACTIVE_LOW_VOLUMEDOWN	0

#define GPIO_ENDCALL            GPIO_PA(30)
#define ACTIVE_LOW_ENDCALL      1

/**
 * TP gpio
 **/
#define GPIO_TP_WAKE		GPIO_PA(12)
#define GPIO_TP_INT		GPIO_PB(0)

#define GPIO_HP_MUTE		-1	/*hp mute gpio*/
#define GPIO_HP_MUTE_LEVEL		-1		/*vaild level*/

#define GPIO_SPEAKER_EN			-1/*speaker enable gpio*/
#define GPIO_SPEAKER_EN_LEVEL	-1

#define GPIO_HANDSET_EN		  -1		/*handset enable gpio*/
#define GPIO_HANDSET_EN_LEVEL -1

#define	GPIO_HP_DETECT	-1		/*hp detect gpio*/
#define GPIO_HP_INSERT_LEVEL    1
#define GPIO_MIC_SELECT		-1		/*mic select gpio*/
#define GPIO_BUILDIN_MIC_LEVEL	-1		/*builin mic select level*/
#define GPIO_MIC_DETECT		-1
#define GPIO_MIC_INSERT_LEVEL -1
#define GPIO_MIC_DETECT_EN		-1  /*mic detect enable gpio*/
#define GPIO_MIC_DETECT_EN_LEVEL	-1 /*mic detect enable gpio*/

/*
 * For BCM2079X NFC
 */
#define NFC_REQ		GPIO_PC(26)
#define NFC_REG_PU	GPIO_PC(27)
#define HOST_WAKE_NFC   GPIO_PA(11)

/* BT gpio */
#define HOST_WAKE_BT	GPIO_PA(1)
#define BT_WAKE_HOST	GPIO_PA(0)
#define BT_REG_EN	GPIO_PA(2)
#define BT_UART_RTS	GPIO_PF(2)
#if 0
#define GPIO_BT_REG_ON      GPIO_PB(30)
#define GPIO_BT_WAKE        GPIO_PB(20)
#define GPIO_BT_INT    	    GPIO_PB(31)
//#define GPIO_BT_RST_N       GPIO_PB(28)
#define GPIO_BT_UART_RTS    GPIO_PF(2)
#define GPIO_PB_FLGREG      (0x10010158)
#define GPIO_BT_INT_BIT	    (1 << (GPIO_BT_INT % 32))
#endif

/* wifi gpio */
#define HOST_WAKE_WL	GPIO_PA(10)
#define WL_WAKE_HOST	GPIO_PA(9)
#define WL_REG_EN	GPIO_PA(8)
#if 0
#define GPIO_WLAN_REG_ON	GPIO_PG(7)
#define GPIO_WLAN_INT	        GPIO_PG(8)
#define GPIO_WLAN_WAKE	        GPIO_PB(28)
//#define GPIO_WIFI_RST_N     GPIO_PB(20)
#endif

#define WLAN_PWR_EN	(-1)
//#define WLAN_PWR_EN	GPIO_PE(3)

/**
 * USB detect pin
 **/
#define GPIO_USB_ID			GPIO_PA(13)
#define GPIO_USB_ID_LEVEL		LOW_ENABLE
#define GPIO_USB_DETE			GPIO_PA(14)
#define GPIO_USB_DETE_LEVEL		HIGH_ENABLE
#define GPIO_USB_DRVVBUS		GPIO_PE(10)
#define GPIO_USB_DRVVBUS_LEVEL		HIGH_ENABLE

extern struct ovisp_camera_platform_data ovisp_camera_info;

/**
 * sound platform data
 **/
extern struct snd_codec_data codec_data;

#ifdef CONFIG_BCM_PM_CORE
extern struct platform_device bcm_power_platform_device;
#endif
#ifdef CONFIG_BROADCOM_RFKILL
extern struct platform_device bt_power_device;
extern struct platform_device bluesleep_device;
#endif /* CONFIG_BROADCOM_RFKILL */
#ifdef CONFIG_BCM2079X_NFC
extern struct bcm2079x_platform_data bcm2079x_pdata;
#endif

#endif /* __BOARD_H__ */
