/*
 *  CPU frequency scaling for JZ47XX SOCS
 *
 *  Copyright (C) 2012 Ingenic Corporation
 *  Written by ztyan<ztyan@ingenic.cn>
 *
 *  Based on cpu-sa1110.c, Copyright (C) 2001 Russell King
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/cpufreq.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/opp.h>
#include <linux/cpu.h>
#include <linux/module.h>
#include <linux/regulator/consumer.h>
#include <linux/syscalls.h>
#include <linux/kthread.h>
#include <linux/slab.h>


static struct jz_cpufreq {
	struct clk *cpu_clk;
	struct cpufreq_frequency_table *freq_table;
} *jz_cpufreq;
/*
 *  unit: kHz
 */
static unsigned long set_cpu_freqs[] = {
	12000,
	24000  ,60000 ,120000,
	200000 ,300000 ,600000,
	792000,1008000,1200000,
	CPUFREQ_TABLE_END
};
#define SUSPEMD_FREQ_INDEX 0
static int m200_verify_speed(struct cpufreq_policy *policy)
{
	return cpufreq_frequency_table_verify(policy, jz_cpufreq->freq_table);
}

static unsigned int m200_getspeed(unsigned int cpu)
{
	if (cpu >= NR_CPUS)
		return 0;
	return clk_get_rate(jz_cpufreq->cpu_clk) / 1000;
}

static int m200_target(struct cpufreq_policy *policy,
			 unsigned int target_freq,
			 unsigned int relation)
{
	int index;
	int ret = 0;
	struct cpufreq_freqs freqs;
	ret = cpufreq_frequency_table_target(policy, jz_cpufreq->freq_table, target_freq, relation, &index);
	if (ret) {
		printk("%s: cpu%d: no freq match for %d(ret=%d)\n",
		       __func__, policy->cpu, target_freq, ret);
		return ret;
	}
	freqs.new = jz_cpufreq->freq_table[index].frequency;
	if (!freqs.new) {
		printk("%s: cpu%d: no match for freq %d\n", __func__,
		       policy->cpu, target_freq);
		return -EINVAL;
	}

	freqs.old = m200_getspeed(policy->cpu);
	freqs.cpu = policy->cpu;

	if (freqs.old == freqs.new && policy->cur == freqs.new)
		return ret;

	cpufreq_notify_transition(policy, &freqs, CPUFREQ_POSTCHANGE);
	//printk("set speed = %d\n",freqs.new);
	ret = clk_set_rate(jz_cpufreq->cpu_clk, freqs.new * 1000);

	freqs.new = m200_getspeed(policy->cpu);

	/*
	 * Note that loops_per_jiffy is not updated on SMP systems in
	 * cpufreq driver. So, update the per-CPU loops_per_jiffy value
	 * on frequency transition. We need to update all dependent CPUs.
	 */

	cpu_data[freqs.cpu].udelay_val = cpufreq_scale(cpu_data[freqs.cpu].udelay_val,freqs.old, freqs.new);

	loops_per_jiffy = cpufreq_scale(loops_per_jiffy,freqs.old, freqs.new);
	/* notifiers */
	cpufreq_notify_transition(policy, &freqs, CPUFREQ_POSTCHANGE);
	return ret;
}
static void init_freq_table(struct cpufreq_frequency_table *table)
{
	int i;
	for(i = 0;i < ARRAY_SIZE(set_cpu_freqs);i++) {
		table[i].index = i;
		table[i].frequency = set_cpu_freqs[i];

	}
}
static int __cpuinit m200_cpu_init(struct cpufreq_policy *policy)
{
	jz_cpufreq = (struct jz_cpufreq *)kzalloc(sizeof(struct jz_cpufreq) +
						  sizeof(struct cpufreq_frequency_table) * ARRAY_SIZE(set_cpu_freqs), GFP_KERNEL);
	if(!jz_cpufreq) {
		pr_err("kzalloc fail!!!\n");
		return -1;
	}
	jz_cpufreq->freq_table = (struct cpufreq_frequency_table *)(jz_cpufreq + 1);
	init_freq_table(jz_cpufreq->freq_table);
	jz_cpufreq->cpu_clk = clk_get(NULL, "cclk");
	if (IS_ERR(jz_cpufreq->cpu_clk))
		goto cpu_clk_err;

	if(cpufreq_frequency_table_cpuinfo(policy, jz_cpufreq->freq_table))
		goto freq_table_err;
	cpufreq_frequency_table_get_attr(jz_cpufreq->freq_table, policy->cpu);
/* #ifdef SUSPEMD_FREQ_INDEX */
/* 	if(SUSPEMD_FREQ_INDEX < ARRAY_SIZE(set_cpu_freqs)) */
/* 		jz_cpufreq->suspend_rate = set_cpu_freqs[SUSPEMD_FREQ_INDEX]; */
/* #endif */
	policy->min = policy->cpuinfo.min_freq;
	policy->max = policy->cpuinfo.max_freq;
	policy->cur = m200_getspeed(policy->cpu);
	/*
	 * On JZ47XX SMP configuartion, both processors share the voltage
	 * and clock. So both CPUs needs to be scaled together and hence
	 * needs software co-ordination. Use cpufreq affected_cpus
	 * interface to handle this scenario.
	 */
	policy->shared_type = CPUFREQ_SHARED_TYPE_ANY;
	cpumask_setall(policy->cpus);
	/* 300us for latency. FIXME: what's the actual transition time? */
	policy->cpuinfo.transition_latency = 500 * 1000;
	printk("cpu freq init ok!\n");
	return 0;
freq_table_err:
	printk("init freq_table_err fail!\n");
	clk_put(jz_cpufreq->cpu_clk);
cpu_clk_err:
	printk("init cpu_clk_err fail!\n");
	kfree(jz_cpufreq);
	return -1;
}

/* static int m200_cpu_suspend(struct cpufreq_policy *policy) */
/* { */
/* 	if(jz_cpufreq->cpu_clk && jz_cpufreq->suspend_rate) { */
/* 		jz_cpufreq->suspend_save_rate = clk_get_rate(jz_cpufreq->cpu_clk); */
/* 		clk_set_rate(jz_cpufreq->cpu_clk,jz_cpufreq->suspend_rate * 1000); */
/* 	} */
/* 	return 0; */
/* } */

/* int m200_cpu_resume(struct cpufreq_policy *policy) */
/* { */
/* 	if(jz_cpufreq->cpu_clk && jz_cpufreq->suspend_save_rate) { */
/* 		clk_set_rate(jz_cpufreq->cpu_clk, jz_cpufreq->suspend_save_rate); */
/* 	} */
/* 	return 0; */
/* } */

static struct freq_attr *m200_cpufreq_attr[] = {
	&cpufreq_freq_attr_scaling_available_freqs,
	NULL,
};

static struct cpufreq_driver m200_driver = {
	.name		= "m200",
	.flags		= CPUFREQ_STICKY,
	.verify		= m200_verify_speed,
	.target		= m200_target,
	.get		= m200_getspeed,
	.init		= m200_cpu_init,
	/* .suspend	= m200_cpu_suspend, */
	/* .resume		= m200_cpu_resume, */
	.attr		= m200_cpufreq_attr,
};

static int __init m200_cpufreq_init(void)
{
	return cpufreq_register_driver(&m200_driver);
}

MODULE_AUTHOR("ztyan<ztyan@ingenic.cn>");
MODULE_DESCRIPTION("cpufreq driver for JZ47XX SoCs");
MODULE_LICENSE("GPL");
arch_initcall(m200_cpufreq_init);
