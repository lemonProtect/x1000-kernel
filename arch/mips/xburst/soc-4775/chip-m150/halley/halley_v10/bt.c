#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/bt-rfkill.h>
#include <mach/jzmmc.h>
#include <asm/atomic.h>

#include "board.h"
#define BLUETOOTH_UPORT_NAME  "ttyS1"

atomic_t clk_32k = ATOMIC_INIT(0);
atomic_t wl_pw_en = ATOMIC_INIT(0);
static int power_en;


int bluesleep_tty_strcmp(const char* name)
{
	if(!strcmp(name,BLUETOOTH_UPORT_NAME)){
		return 0;
	}else{
		return -1;
	}
}
EXPORT_SYMBOL(bluesleep_tty_strcmp);

void clk_32k_on(void)
{
	jzrtc_enable_clk32k();
	atomic_inc(&clk_32k);
	if (atomic_read(&clk_32k) > 3){
		atomic_set(&clk_32k, 3);
	}
	printk("clk_32k_on:num = %d\n",atomic_read(&clk_32k));

}
EXPORT_SYMBOL(clk_32k_on);

void clk_32k_off(void)
{
	atomic_dec(&clk_32k);
	if(atomic_read(&clk_32k) < 0){
		atomic_set(&clk_32k, 0);
	}
	if(atomic_read(&clk_32k) == 0){
		jzrtc_disable_clk32k();
	}
	printk("clk_32k_off:num = %d\n",atomic_read(&clk_32k));

}
EXPORT_SYMBOL(clk_32k_off);

void wlan_pw_en_enable(struct regulator *power)
{
	//gpio_set_value(power_en,1);
	atomic_inc(&wl_pw_en);
	if(atomic_read(&wl_pw_en) > 3){
		atomic_set(&wl_pw_en, 3);
	}
	printk("wl_pw_en = %d\n",atomic_read(&wl_pw_en));
	printk("wl_pw_en gpio = %d\n",power_en);
}
EXPORT_SYMBOL(wlan_pw_en_enable);

void wlan_pw_en_disable(void)
{
	atomic_dec(&wl_pw_en);
	if(atomic_read(&wl_pw_en) < 0){
		atomic_set(&wl_pw_en, 0);
	}
	if(atomic_read(&wl_pw_en) == 0){
//		gpio_set_value(power_en,0);
	}
	printk("wl_pw_en = %d\n",atomic_read(&wl_pw_en));
}
EXPORT_SYMBOL(wlan_pw_en_disable);

int bt_power_init(void)
{
	power_en = GPIO_WLAN_PW_EN;
	if (gpio_request(GPIO_WLAN_PW_EN, "wlan_pw_en")) {
		pr_err("no wlan_pw_en pin available\n");
		return -EINVAL;
	}else {
		gpio_direction_output(power_en, 0);
	}

	return 0;
}

static struct bt_rfkill_platform_data  gpio_data = {
	.gpio = {
		.bt_rst_n = -1,
		.bt_reg_on = BT_REG_EN,
		.bt_wake = HOST_WAKE_BT,
		.bt_int = BT_WAKE_HOST,
#if 0
		.bt_int_flagreg = -1,
		.bt_int_bit = -1,
#endif
		.bt_uart_rts = BT_UART_RTS,
	},

	.restore_pin_status = NULL,
	.set_pin_status = NULL,
};

struct platform_device bt_power_device  = {
	.name = "bt_power" ,
	.id = -1 ,
	.dev   = {
		.platform_data = &gpio_data,
	},
};

struct platform_device bluesleep_device = {
	.name = "bluesleep" ,
	.id = -1 ,
	.dev   = {
		.platform_data = &gpio_data,
	},

};

static int __init m150_bt_power_init(void)
{
	bt_power_init();
	return 0;
}
module_init(m150_bt_power_init);
