/*
 *  Copyright (C) 2014,  King liuyang <liuyang@ingenic.cn>
 *  jz_PWM support
 *
 *
 */

#include <linux/clk.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/pwm.h>
#include <mach/jztcu.h>
#include <soc/extal.h>
#include <soc/gpio.h>
#include <linux/slab.h>

#define NUM_PWM 8

struct jz_pwm_device{
	short id;
	const char *label;
	struct tcu_device *tcu_cha;
};

struct jz_pwm_chip {
	struct clk *clk;
	struct jz_pwm_device pwm_chrs[NUM_PWM];
	struct pwm_chip chip;
};

static inline struct jz_pwm_chip *to_jz(struct pwm_chip *chip)
{
	return container_of(chip, struct jz_pwm_chip, chip);
}

static int jz_pwm_request(struct pwm_chip *chip, struct pwm_device *pwm)
{
	struct jz_pwm_chip *jz = to_jz(chip);
	struct tcu_device *tcu_pwm;
	int id = pwm -> hwpwm;
	if (id < 0 || id > 7)
		return -ENODEV;

	tcu_pwm = jz->pwm_chrs[id].tcu_cha;

	tcu_pwm= tcu_request(id, NULL);

	if(IS_ERR(tcu_pwm)) {
		printk("-+-+-+-pwm_tcu_request failed!!-+-+-+\n");
		return -ENODEV;
	}

	jz->pwm_chrs[id].id = id;
	jz->pwm_chrs[id].label = pwm->label;

	tcu_pwm->pwm_flag = pwm->flags;
	tcu_pwm->irq_type = NULL_IRQ_MODE;
	tcu_pwm->init_level = 0;
	printk("request pwm channel %d successfully\n", id);

	return 0;
}

static void jz_pwm_free(struct pwm_chip *chip, struct pwm_device *pwm)
{
	struct jz_pwm_chip *jz = to_jz(chip);
	struct tcu_device *tcu_pwm = jz->pwm_chrs[pwm->hwpwm].tcu_cha;
	tcu_free(tcu_pwm);
	kfree (tcu_pwm);
	kfree (jz);
}

static int jz_pwm_enable(struct pwm_chip *chip, struct pwm_device *pwm)
{
	struct jz_pwm_chip *jz = to_jz(chip);
	struct tcu_device *tcu_pwm = jz->pwm_chrs[pwm->hwpwm].tcu_cha;

	tcu_enable(tcu_pwm);

	jzgpio_set_func(GPIO_PORT_E, GPIO_FUNC_0, 0x1 << pwm->hwpwm);

	return 0;
}

static void jz_pwm_disable(struct pwm_chip *chip, struct pwm_device *pwm)
{
	struct jz_pwm_chip *jz = to_jz(chip);
	struct tcu_device *tcu_pwm = jz->pwm_chrs[pwm->hwpwm].tcu_cha;

	tcu_disable(tcu_pwm);

	gpio_direction_output((GPIO_PORT_E * 32 + pwm->hwpwm), 0);
}

static int jz_pwm_config(struct pwm_chip *chip, struct pwm_device *pwm,
			     int duty_ns, int period_ns)
{
	unsigned long long tmp;
	unsigned long period, duty;
	struct jz_pwm_chip *jz = to_jz(chip);
	int prescaler = 0; /*prescale = 0,1,2,3,4,5*/

	int id = pwm->hwpwm;
	struct tcu_device *tcu_pwm = jz->pwm_chrs[id].tcu_cha;

	if (duty_ns < 0 || duty_ns > period_ns)
		return -EINVAL;
	if (period_ns < 200 || period_ns > 1000000000)
		return -EINVAL;

	tcu_pwm->id = id;
	tcu_pwm->irq_type = NULL_IRQ_MODE;
	tcu_pwm->pwm_flag = pwm->flags;

#ifndef CONFIG_SLCD_SUSPEND_ALARM_WAKEUP_REFRESH
	tmp = JZ_EXTAL;
#else
	tmp = JZ_EXTAL;//32768 RTC CLOCK failure!
#endif
	tmp = tmp * period_ns;

	do_div(tmp, 1000000000);
	period = tmp;

	while (period > 0xffff && prescaler < 6) {
		period >>= 2;
		++prescaler;
	}
	if (prescaler == 6)
		return -EINVAL;

	tmp = (unsigned long long)period * duty_ns;
	do_div(tmp, period_ns);
	duty = tmp;

	if (duty >= period)
		duty = period - 1;
	tcu_pwm->full_num = period;
	tcu_pwm->half_num = (period - duty);
	tcu_pwm->divi_ratio = prescaler;
#ifdef CONFIG_SLCD_SUSPEND_ALARM_WAKEUP_REFRESH
	tcu_pwm->clock = RTC_EN;
#else
	tcu_pwm->clock = EXT_EN;
#endif
	tcu_pwm->count_value = 0;
	tcu_pwm->pwm_shutdown = 1;

	if(tcu_as_timer_config(tcu_pwm) != 0)
		return -EINVAL;

	return 0;
}

static const struct pwm_ops jz_pwm_ops = {
	.request = jz_pwm_request,
	.free = jz_pwm_free,
	.config = jz_pwm_config,
	.enable = jz_pwm_enable,
	.disable = jz_pwm_disable,
	.owner = THIS_MODULE,
};

static int jz_pwm_probe(struct platform_device *pdev)
{
	struct jz_pwm_chip *jz;
	struct tcu_device *tcu_pwm;
	int ret, i;

	jz = devm_kzalloc(&pdev->dev, sizeof(*jz), GFP_KERNEL);
	tcu_pwm = devm_kzalloc(&pdev->dev,(NUM_PWM * sizeof(*tcu_pwm)),GFP_KERNEL);

	if (!jz || !tcu_pwm)
		return -ENOMEM;

	jz->clk = clk_get(NULL, "tcu");
	if (IS_ERR(jz->clk))
		return PTR_ERR(jz->clk);

	jz->chip.dev = &pdev->dev;
	jz->chip.ops = &jz_pwm_ops;
	jz->chip.npwm = NUM_PWM;
	jz->chip.base = -1;

	for(i=0; i < NUM_PWM; i++){
		jz->pwm_chrs[i].tcu_cha = tcu_pwm++;
	}

	ret = pwmchip_add(&jz->chip);
	if (ret < 0) {
		clk_put(jz->clk);
		return ret;
	}

	platform_set_drvdata(pdev, jz);

	return 0;
}

static int jz_pwm_remove(struct platform_device *pdev)
{
	struct jz_pwm_chip *jz = platform_get_drvdata(pdev);
	int ret;

	ret = pwmchip_remove(&jz->chip);
	if (ret < 0)
		return ret;

	clk_put(jz->clk);

	return 0;
}

static struct platform_driver jz_pwm_driver = {
	.driver = {
		.name = "jz-pwm",
		.owner = THIS_MODULE,
	},
	.probe = jz_pwm_probe,
	.remove = jz_pwm_remove,
};
module_platform_driver(jz_pwm_driver);

MODULE_AUTHOR("King liuyang <liuyang@ingenic.cn>");
MODULE_DESCRIPTION("Ingenic jz PWM driver");
MODULE_ALIAS("platform:jz-pwm");
MODULE_LICENSE("GPL");
