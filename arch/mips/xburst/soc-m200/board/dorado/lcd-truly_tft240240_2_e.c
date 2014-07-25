/*
 * Copyright (c) 2014 Engenic Semiconductor Co., Ltd.
 *              http://www.ingenic.com/
 *
 * JZ-M200 orion board lcd setup routines.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/mm.h>
#include <linux/console.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/pwm_backlight.h>
#include <linux/lcd.h>
#include <linux/regulator/consumer.h>
#include <mach/jzfb.h>

#include "board.h"

/******GPIO PIN************/
#undef GPIO_LCD_CS
#undef GPIO_LCD_RD
#undef GPIO_BL_PWR_EN

#define GPIO_LCD_CS            GPIO_PC(14)
#define GPIO_LCD_RD            GPIO_PC(17)
#define GPIO_BL_PWR_EN         GPIO_PC(18)

/*ifdef is 18bit,6-6-6 ,ifndef default 5-6-6*/
#define CONFIG_SLCD_TRULY_18BIT

#ifdef	CONFIG_SLCD_TRULY_18BIT
static int slcd_inited = 1;
#else
static int slcd_inited = 0;
#endif

struct truly_tft240240_power{
	struct regulator *vlcdio;
	struct regulator *vlcdvcc;
	int inited;
};

static struct truly_tft240240_power lcd_power = {
	NULL,
	NULL,
	0
};

int truly_tft240240_power_init(struct lcd_device *ld)
{
	int ret ;
	printk("======truly_tft240240_power_init==============\n");

	ret = gpio_request(GPIO_LCD_RST, "lcd rst");
	if (ret) {
		printk(KERN_ERR "can's request lcd rst\n");
		return ret;
	}

	ret = gpio_request(GPIO_LCD_CS, "lcd cs");
	if (ret) {
		printk(KERN_ERR "can's request lcd cs\n");
		return ret;
	}

	ret = gpio_request(GPIO_LCD_RD, "lcd rd");
	if (ret) {
		printk(KERN_ERR "can's request lcd rd\n");
		return ret;
	}

	printk("set lcd_power.inited  =======1 \n");
	lcd_power.inited = 1;
	return 0;
}

int truly_tft240240_power_reset(struct lcd_device *ld)
{
	if (!lcd_power.inited)
		return -EFAULT;
	gpio_direction_output(GPIO_LCD_RST, 0);
	mdelay(20);
	gpio_direction_output(GPIO_LCD_RST, 1);
	mdelay(10);

	return 0;
}

int truly_tft240240_power_on(struct lcd_device *ld, int enable)
{
	if (!lcd_power.inited && truly_tft240240_power_init(ld))
		return -EFAULT;

	if (enable) {
		gpio_direction_output(GPIO_LCD_CS, 1);
		gpio_direction_output(GPIO_LCD_RD, 1);

		truly_tft240240_power_reset(ld);

		mdelay(5);
		gpio_direction_output(GPIO_LCD_CS, 0);

	} else {
		gpio_direction_output(GPIO_BL_PWR_EN, 0);
		mdelay(5);
		gpio_direction_output(GPIO_LCD_CS, 0);
		gpio_direction_output(GPIO_LCD_RD, 0);
		gpio_direction_output(GPIO_LCD_RST, 0);
		gpio_direction_output(GPIO_LCD_DISP, 0);
		slcd_inited = 0;
	}
	return 0;
}

struct lcd_platform_data truly_tft240240_pdata = {
	.reset    = truly_tft240240_power_reset,
	.power_on = truly_tft240240_power_on,
};

/* LCD Panel Device */
struct platform_device truly_tft240240_device = {
	.name		= "truly_tft240240_slcd",
	.dev		= {
		.platform_data	= &truly_tft240240_pdata,
	},
};

static struct smart_lcd_data_table truly_tft240240_data_table[] = {
	/* LCD init code */
	{0x01, 0x01, 1, 120000},  //soft reset, 120 ms = 120 000 us
	{0x11, 0x11, 1, 5000},	  /* sleep out 5 ms  */

	{0x36, 0x36, 1, 0},
#ifdef	CONFIG_TRULY_240X240_ROTATE_180
	/*{0x36, 0xc0, 2, 0}, //40*/
	{0x36, 0xd0, 2, 0}, //40
#else
	{0x36, 0x00, 2, 0}, //40
#endif

	{0x2a, 0x2a, 1, 0},
	{0x2a, 0x00, 2, 0},
	{0x2a, 0x00, 2, 0},
	{0x2a, 0x00, 2, 0},
	{0x2a, 0xef, 2, 0},

	{0x2b, 0x2b, 1, 0},
	{0x2b, 0x00, 2, 0},
	{0x2b, 0x00, 2, 0},
	{0x2b, 0x00, 2, 0},
	{0x2b, 0xef, 2, 0},


	{0x3a, 0x3a, 1, 0},
#if defined(CONFIG_SLCD_TRULY_18BIT)  //if 18bit/pixel unusual. try to use 16bit/pixel
	{0x3a, 0x06, 2, 0}, //6-6-6
#else
	{0x3a, 0x05, 2, 0}, //5-6-5
#endif
//	{0x3a, 0x55, 2, 0},

	{0xb2, 0xb2, 1, 0},
	{0xb2, 0x7f, 2, 0},
	{0xb2, 0x7f, 2, 0},
	{0xb2, 0x01, 2, 0},
	{0xb2, 0xde, 2, 0},
	{0xb2, 0x33, 2, 0},

	{0xb3, 0xb3, 1, 0},
	{0xb3, 0x10, 2, 0},
	{0xb3, 0x05, 2, 0},
	{0xb3, 0x0f, 2, 0},

	{0xb4, 0xb4, 1, 0},
	{0xb4, 0x0b, 2, 0},

	{0xb7, 0xb7, 1, 0},
	{0xb7, 0x35, 2, 0},

	{0xbb, 0xbb, 1, 0},
	{0xbb, 0x28, 2, 0}, //23

	{0xbc, 0xbc, 1, 0},
	{0xbc, 0xec, 2, 0},

	{0xc0, 0xc0, 1, 0},
	{0xc0, 0x2c, 2, 0},

	{0xc2, 0xc2, 1, 0},
	{0xc2, 0x01, 2, 0},

	{0xc3, 0xc3, 1, 0},
	{0xc3, 0x1e, 2, 0}, //14

	{0xc4, 0xc4, 1, 0},
	{0xc4, 0x20, 2, 0},

	{0xc6, 0xc6, 1, 0},
	{0xc6, 0x14, 2, 0},

	{0xd0, 0xd0, 1, 0},
	{0xd0, 0xa4, 2, 0},
	{0xd0, 0xa1, 2, 0},

	{0xe0, 0xe0, 1, 0},
	{0xe0, 0xd0, 2, 0},
	{0xe0, 0x00, 2, 0},
	{0xe0, 0x00, 2, 0},
	{0xe0, 0x08, 2, 0},
	{0xe0, 0x07, 2, 0},
	{0xe0, 0x05, 2, 0},
	{0xe0, 0x29, 2, 0},
	{0xe0, 0x54, 2, 0},
	{0xe0, 0x41, 2, 0},
	{0xe0, 0x3c, 2, 0},
	{0xe0, 0x17, 2, 0},
	{0xe0, 0x15, 2, 0},
	{0xe0, 0x1a, 2, 0},
	{0xe0, 0x20, 2, 0},

	{0xe1, 0xe1, 1, 0},
	{0xe1, 0xd0, 2, 0},
	{0xe1, 0x00, 2, 0},
	{0xe1, 0x00, 2, 0},
	{0xe1, 0x08, 2, 0},
	{0xe1, 0x07, 2, 0},
	{0xe1, 0x04, 2, 0},
	{0xe1, 0x29, 2, 0},
	{0xe1, 0x44, 2, 0},
	{0xe1, 0x42, 2, 0},
	{0xe1, 0x3b, 2, 0},
	{0xe1, 0x16, 2, 0},
	{0xe1, 0x15, 2, 0},
	{0xe1, 0x1b, 2, 0},
	{0xe1, 0x1f, 2, 0},

	{0x35, 0x35, 1, 0}, // TE on
	{0x35, 0x00, 2, 0}, // TE mode: 0, mode1; 1, mode2
//	{0x34, 0x34, 1, 0}, // TE off

	{0x29, 0x29, 1, 0}, //Display ON

	/* set window size*/
//	{0xcd, 0xcd, 1, 0},
	{0x2a, 0x2a, 1, 0},
	{0x2a, 0, 2, 0},
	{0x2a, 0, 2, 0},
	{0x2a, (239>> 8) & 0xff, 2, 0},
	{0x2a, 239 & 0xff, 2, 0},
#ifdef	CONFIG_TRULY_240X240_ROTATE_180
	{0x2b, 0x2b, 1, 0},
	{0x2b, ((320-240)>>8)&0xff, 2, 0},
	{0x2b, ((320-240)>>0)&0xff, 2, 0},
	{0x2b, ((320-1)>>8) & 0xff, 2, 0},
	{0x2b, ((320-1)>>0) & 0xff, 2, 0},
#else
	{0x2b, 0x2b, 1, 0},
	{0x2b, 0, 2, 0},
	{0x2b, 0, 2, 0},
	{0x2b, (239>> 8) & 0xff, 2, 0},
	{0x2b, 239 & 0xff, 2, 0},
#endif

//	{0xcd, 0xcd, 1, 0},
//	{0x2c, 0x2c, 1, 0},
};

struct fb_videomode jzfb0_videomode = {
	.name = "240x240",
	.refresh = 60,
	.xres = 240,
	.yres = 240,
	.pixclock = KHZ2PICOS(30000),
	.left_margin = 0,
	.right_margin = 0,
	.upper_margin = 0,
	.lower_margin = 0,
	.hsync_len = 0,
	.vsync_len = 0,
	.sync = FB_SYNC_HOR_HIGH_ACT | FB_SYNC_VERT_HIGH_ACT,
	.vmode = FB_VMODE_NONINTERLACED,
	.flag = 0,
};


struct jzfb_platform_data jzfb_pdata = {
	.num_modes = 1,
	.modes = &jzfb0_videomode,
	.lcd_type = LCD_TYPE_LCM,
	.bpp    = 18,
	.width  = 31,
	.height = 31,
	.pinmd  = 0,
	.pixclk_falling_edge    = 0,
	.date_enable_active_low = 0,
	.alloc_vidmem = 1,

	.smart_config.smart_type      = SMART_LCD_TYPE_PARALLEL,
	.smart_config.cmd_width       = SMART_LCD_CWIDTH_8_BIT_ONCE,           //8bit, according to the 8bit command
	.smart_config.data_width      = SMART_LCD_DWIDTH_24_BIT_ONCE_PARALLEL, //due to new slcd mode, must be 24bit
	.smart_config.data_new_width  = SMART_LCD_NEW_DWIDTH_8_BIT,          //9bit, according to panel bus width
	.smart_config.data_new_times  = SMART_LCD_NEW_DTIMES_ONCE,   //8bit once, for 8bit command

#if defined(CONFIG_SLCD_TRULY_18BIT)  //if 18bit/pixel unusual. try to use 16bit/pixel
	.smart_config.data_new_times2 = SMART_LCD_NEW_DTIMES_THICE, //18bit three times, for 6-6-6
#else
	.smart_config.data_new_times2 = SMART_LCD_NEW_DTIMES_TWICE,//16bit two times,for 5-6-5
#endif

	.smart_config.clkply_active_rising = 0,
	.smart_config.rsply_cmd_high       = 0,
	.smart_config.csply_active_high    = 0,
	/* write graphic ram command, in word, for example 8-bit bus, write_gram_cmd=C3C2C1C0. */
	.smart_config.newcfg_fmt_conv =  1,
	.smart_config.write_gram_cmd = 0x2c2c2c2c,
	.smart_config.bus_width = 8,
	.smart_config.length_data_table =  ARRAY_SIZE(truly_tft240240_data_table),
	.smart_config.data_table = truly_tft240240_data_table,
	.smart_config.init = 0,
	.dither_enable = 0,
};
/**************************************************************************************************/
#ifdef CONFIG_BACKLIGHT_PWM
static int backlight_init(struct device *dev)
{
	int ret;
	ret = gpio_request(GPIO_LCD_PWM, "Backlight");
	if (ret) {
		printk(KERN_ERR "failed to request GPF for PWM-OUT1\n");
		return ret;
	}

	ret = gpio_request(GPIO_BL_PWR_EN, "BL PWR");
	if (ret) {
		printk(KERN_ERR "failed to reqeust BL PWR\n");
		return ret;
	}
	gpio_direction_output(GPIO_BL_PWR_EN, 1);
	return 0;
}

static int backlight_notify(struct device *dev, int brightness)
{
	if (brightness)
		gpio_direction_output(GPIO_BL_PWR_EN, 1);
	else
		gpio_direction_output(GPIO_BL_PWR_EN, 0);

	return brightness;
}

static void backlight_exit(struct device *dev)
{
	gpio_free(GPIO_LCD_PWM);
}

static struct platform_pwm_backlight_data backlight_data = {
	.pwm_id		= 1,
	.max_brightness	= 255,
	.dft_brightness	= 120,
	.pwm_period_ns	= 30000,
	.init		= backlight_init,
	.exit		= backlight_exit,
	.notify		= backlight_notify,
};

struct platform_device backlight_device = {
	.name		= "pwm-backlight",
	.dev		= {
		.platform_data	= &backlight_data,
	},
};
#endif
