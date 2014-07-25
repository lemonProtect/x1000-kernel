/*
 * Copyright (c) 2012 Ingenic Semiconductor Co., Ltd.
 *              http://www.ingenic.com/
 *
 * M200 dorado board lcd setup routines.
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

#undef  GPIO_LCD_RST
#undef  GPIO_LCD_RD
#undef  SLCD_NBUSY_PIN


#define GPIO_LCD_BLK		GPIO_PC(17)
#define GPIO_LCD_RST        GPIO_PA(12)
#define GPIO_LCD_NRD_E		GPIO_PC(8)
#define GPIO_LCD_NWR_SCL	GPIO_PC(25)
#define GPIO_LCD_DNC		GPIO_PC(26)
#define SLCD_NBUSY_PIN		GPIO_PA(11)

struct cv90_m5377_p30_power{
	struct regulator *vlcdio;
	struct regulator *vlcdvcc;
	int inited;
};

static struct cv90_m5377_p30_power lcd_power = {
	NULL,
	NULL,
	0
};

int cv90_m5377_p30_power_init(struct lcd_device *ld)
{
	int ret ;

	printk("======cv90_m5377_p30_power_init==============\n");

	ret = gpio_request(GPIO_LCD_RST, "lcd rst");
	if (ret) {
		printk(KERN_ERR "can's request lcd rst\n");
		return ret;
	}
	ret = gpio_request(GPIO_LCD_BLK, "lcd blk");
	if (ret) {
		printk(KERN_ERR "can's request lcd rst\n");
		return ret;
	}

	ret = gpio_request(SLCD_NBUSY_PIN, "lcd nbusy pin");
	if (ret) {
		printk(KERN_ERR "can's request lcd nbusy pin\n");
		return ret;
	}

#ifdef GPIO_DEBUG
	ret = gpio_request(GPIO_LCD_NRD_E, "lcd nrd_e");
	if (ret) {
		printk(KERN_ERR "can's request lcd nrd_e\n");
		return ret;
	}

	ret = gpio_request(GPIO_LCD_DNC, "lcd dnc");
	if (ret) {
		printk(KERN_ERR "can's request lcd dnc\n");
		return ret;
	}
	ret = gpio_request(GPIO_LCD_NWR_SCL, "lcd dwr_scl");
	if (ret) {
		printk(KERN_ERR "can's request lcd dwr_scl\n");
		return ret;
	}
#endif
	lcd_power.inited = 1;
	return 0;
}

int cv90_m5377_p30_power_reset(struct lcd_device *ld)
{
	printk("cv90_m5377_p30_power_reset==============\n");
	if (!lcd_power.inited)
		return -EFAULT;
	gpio_direction_output(GPIO_LCD_RST, 0);  //reset active low
	mdelay(20);
	gpio_direction_output(GPIO_LCD_RST, 1);
	mdelay(250);
	return 0;
}

static int slcd_cv90_m5377_p30_p1_waiting_NBUSY_rising_high(void)
{

		int count = 0;
		int ret = 0;
		while ( count++ < 100 ) {
			ret = gpio_get_value_cansleep(SLCD_NBUSY_PIN);
			if (ret)
				return 1;
			else
				printk(KERN_ERR "%s -> gpio_get_value_cansleep() return val= %d\n",__func__, ret);

			mdelay(1);
		}
		printk(KERN_ERR "%s wait the <NBUSY> timeout count=%d\n", __func__, count);

		return 0;
}


int cv90_m5377_p30_power_on(struct lcd_device *ld, int enable)
{
	printk("======cv90_m5377_p30_power_on ======\n");
	if (!lcd_power.inited && cv90_m5377_p30_power_init(ld))
		return -EFAULT;

	/*enable backlight, set it according to your board */
	gpio_direction_output(GPIO_LCD_BLK, 1);
	/*NRD_E, must be high, either setting gpio or setting high on your board.*/
	//gpio_direction_output(GPIO_LCD_NRD_E, 1);
	/*reset your panel*/
    cv90_m5377_p30_power_reset(ld);

#ifdef  GPIO_DEBUG
	printk("start important gpio test!\n");
	while(1){
		gpio_direction_output(GPIO_LCD_DNC, 1);
		gpio_direction_output(GPIO_LCD_NWR_SCL, 0);
		mdelay(10);
		gpio_direction_output(GPIO_LCD_DNC, 0);
		gpio_direction_output(GPIO_LCD_NWR_SCL, 1);
		mdelay(2);
	}
#endif


	if (enable) {
		/*fixed*/
		printk("======cv90_m5377_p30_power_on ==enable==\n");
	} else {
		/*fixed*/
		printk("======cv90_m5377_p30_power_on ====disable==\n");
	}
	return 0;
}

struct lcd_platform_data cv90_m5377_p30_pdata = {
	.reset    = cv90_m5377_p30_power_reset,
	.power_on = cv90_m5377_p30_power_on,
};

/* LCD Panel Device */
struct platform_device cv90_m5377_p30_device = {
	.name		= "cv90_m5377_p30_slcd",
	.dev		= {
		.platform_data	= &cv90_m5377_p30_pdata,
	},
};

static struct smart_lcd_data_table cv90_m5377_p30_data_table[] = {

	/* Extended CMD enable CMD */
         {0xB9, 0xB9, 1, 0},

         {0xB9, 0xff, 2, 0},
         {0xB9, 0x52, 2, 0},
         {0xB9, 0x52, 2, 0},

         /* sleep out command, and wait > 35ms */
         {0x11, 0x11, 1, 200000},

	 /* Set Pixel Format Cmd */
         {0x3a, 0x3a, 1, 0},
         //{0x3a, 0x01, 2, 0},	/* 6bit */
      //   {0x3a, 0x05, 2, 0},	/* 16bit */
         {0x3a, 0x06, 2, 0},	/* 24bit? */

	 /* Pixel Format = 6bit? */

#if 1
	/* Enable CP */
         {0xf4, 0xF4, 1, 0},
         {0x00, 0x01, 2, 0},	/* Enable CP */
         {0x00, 0x01, 2, 0},	/* Photo Mode */
         {0x00, 0x02, 2, 0},	/* Floyd Steinberg */
#else
	/* Disable CP */
         {0xf4, 0xF4, 1, 0},
         {0x00, 0x00, 2, 0},	/* Disable CP */
         {0x00, 0x00, 2, 0},	/* Photo Mode */
         {0x00, 0x00, 2, 0},	/* Floyd Steinberg */
#endif

	 /* RAM Command */
         {0x00, 0xbd, 1, 0},
         {0x00, 0x00, 2, 0},


	 /* Display On */
         {0x29, 0x29, 1, 0},



	 /* polling NBUSY, proceed on the rising edge. */
         //{0xcd, 0xcd, 1, 50000},
        // {0x2c, 0x2c, 1, 0},


};

static int cv90_m5377_p30_dma_transfer_begin_callback(void*jzfb)
{
	int ret = 0;
	ret = slcd_cv90_m5377_p30_p1_waiting_NBUSY_rising_high();
	return ret;
}

static int cv90_m5377_p30_dma_transfer_end_callback(void*jzfb)
{
	return 0;
}



struct fb_videomode jzfb_videomode = {
	.name = "288x192",
	.refresh = 45,
	.xres = 288,
	.yres = 192,
	.pixclock = KHZ2PICOS(5000*3), /* SLCDC WR_SCL = LCDC_PIXEL_CLOCK/2, so wr_scl is about 7.5MHZ*/
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
		.modes = &jzfb_videomode,

        .lcd_type = LCD_TYPE_LCM,
        .bpp = 24,

        .pixclk_falling_edge = 0,
        .date_enable_active_low = 0,

		.alloc_vidmem = 1,
        .dither_enable = 0,

        .smart_config.smart_type = SMART_LCD_TYPE_PARALLEL,
		.smart_config.cmd_width = SMART_LCD_CWIDTH_8_BIT_ONCE, //8bit, according to the 8bit command
		.smart_config.data_width = SMART_LCD_DWIDTH_24_BIT_ONCE_PARALLEL, //due to new slcd mode, must be 24bit
        .smart_config.data_new_width = SMART_LCD_NEW_DWIDTH_8_BIT,  //8bit, according to panel bus width
        .smart_config.data_new_times = SMART_LCD_NEW_DTIMES_ONCE,   //8bit once, for 8bit command
        .smart_config.data_new_times2 = SMART_LCD_NEW_DTIMES_THICE,   //8bit three times, for 24bit RGB color
        .smart_config.clkply_active_rising = 0,
        .smart_config.rsply_cmd_high = 0,
        .smart_config.csply_active_high = 0,
		/* write graphic ram command, in word, for example 8-bit bus, write_gram_cmd=C3C2C1C0. */
		.smart_config.write_gram_cmd = 0x2c2c2ccd,

        .smart_config.bus_width = 8,  //due to panel bus width
		.smart_config.length_data_table = ARRAY_SIZE(cv90_m5377_p30_data_table),
        .smart_config.data_table = cv90_m5377_p30_data_table,
		.smart_config.init = 0,//cv90_m5377_p30_init,

		.lcd_callback_ops.dma_transfer_begin = cv90_m5377_p30_dma_transfer_begin_callback,
		.lcd_callback_ops.dma_transfer_end = cv90_m5377_p30_dma_transfer_end_callback,
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

//	gpio_direction_output(GPIO_BL_PWR_EN, 1);

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
