/*
 * JZSOC Clock and Power Manager
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright (C) 2006 Ingenic Semiconductor Inc.
 */
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/err.h>
#include <linux/proc_fs.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/syscore_ops.h>
#include <linux/vmalloc.h>
#include <linux/suspend.h>
#include <linux/seq_file.h>
#include <soc/cpm.h>
#include <soc/base.h>
#include <soc/extal.h>
#include <jz_proc.h>

#include "clk.h"

static struct clk clk_srcs[] = {
#define GATE(x)  (((x)<<24) | CLK_FLG_GATE)
#define CPCCR(x) (((x)<<24) | CLK_FLG_CPCCR)
#define CGU(no)  (((no)<<24) | CLK_FLG_CGU)
#define PLL(no)  (((no)<<24) | CLK_FLG_PLL)
#define PWC(no)  (((no)<<24) | CLK_FLG_PWC)
#define PARENT(P)  (((CLK_ID_##P)<<16) | CLK_FLG_PARENT)
#define RELATIVE(P)  (((CLK_ID_##P)<<16) | CLK_FLG_RELATIVE)
#define DEF_CLK(N,FLAG)						\
	[CLK_ID_##N] = { .name = CLK_NAME_##N, .flags = FLAG, }

	DEF_CLK(EXT0,  		CLK_FLG_NOALLOC),
	DEF_CLK(EXT1,  		CLK_FLG_NOALLOC),
	DEF_CLK(OTGPHY,         CLK_FLG_NOALLOC),

	DEF_CLK(APLL,  		PLL(CPM_CPAPCR)),
	DEF_CLK(MPLL,  		PLL(CPM_CPMPCR)),

	DEF_CLK(SCLKA,		CPCCR(SCLKA)),
	DEF_CLK(CCLK,  		CPCCR(CDIV)),
	DEF_CLK(L2CLK,  	CPCCR(L2CDIV)),
	DEF_CLK(H0CLK,  	CPCCR(H0DIV)),
	DEF_CLK(H2CLK, 		CPCCR(H2DIV)),
	DEF_CLK(PCLK, 		CPCCR(PDIV)),

	DEF_CLK(NFI,   		GATE(0) | PARENT(H2CLK)),
	DEF_CLK(NEMC,  		GATE(1) | PARENT(H2CLK)),
	DEF_CLK(BCH,   		GATE(2) | PARENT(H2CLK)),
	DEF_CLK(OTG,   		GATE(3) | PARENT(CGU_USB)),
	DEF_CLK(MSC0,  		GATE(4) | PARENT(PCLK)),
	DEF_CLK(MSC1,  		GATE(5) | PARENT(PCLK)),
	DEF_CLK(SSI0,  		GATE(6) | PARENT(PCLK)),
	DEF_CLK(I2C0,  		GATE(7) | PARENT(PCLK)),

	DEF_CLK(I2C1,  		GATE(8) | PARENT(PCLK)),
	DEF_CLK(I2C2,  		GATE(9) | PARENT(PCLK)),
	DEF_CLK(I2C3,  		GATE(10) | PARENT(PCLK)),
	DEF_CLK(AIC,  		GATE(11) | PARENT(PCLK)),
	DEF_CLK(MSC2,  		GATE(12) | PARENT(PCLK)),
	DEF_CLK(SADC,  		GATE(13) | PARENT(PCLK)),
	DEF_CLK(UART0,  	GATE(14) | PARENT(EXT1)),
	DEF_CLK(UART1,  	GATE(15) | PARENT(EXT1)),

	DEF_CLK(UART2,  	GATE(16) | PARENT(EXT1)),
	DEF_CLK(UART3,  	GATE(17) | PARENT(EXT1)),
	DEF_CLK(UART4,  	GATE(18) | PARENT(EXT1)),
	DEF_CLK(SSI1,  		GATE(19) | PARENT(PCLK)),
	DEF_CLK(SSI2,  		GATE(20) | PARENT(PCLK)),
	DEF_CLK(PDMA,  		GATE(21) | PARENT(PCLK)),
	DEF_CLK(UHC,  		GATE(22) | PARENT(PCLK)),
	DEF_CLK(ISP,  		GATE(23) | PARENT(PCLK)),

	DEF_CLK(LCD,  		GATE(24) | PARENT(H0CLK)),
	DEF_CLK(CSI,  		GATE(25) | PARENT(PCLK)),
	DEF_CLK(DSI,  		GATE(26) | PARENT(PCLK)),
	DEF_CLK(PCM,  		GATE(27) | PARENT(H0CLK)),
	DEF_CLK(DES,  		GATE(28) | PARENT(PCLK)),
	DEF_CLK(RTC,  		GATE(29) | PARENT(EXT0)),
	DEF_CLK(TCU,  		GATE(30) | PARENT(PCLK)),
	DEF_CLK(DDR,  		GATE(31) | PARENT(PCLK)),

	DEF_CLK(VPU,  		GATE(32 + 0) | PARENT(PCLK)),
	DEF_CLK(GPU,  		GATE(32 + 1) | PARENT(PCLK)),
	DEF_CLK(IPU,  		GATE(32 + 2) | PARENT(LCD)),
	DEF_CLK(AHB_MON, 	GATE(32 + 3) | PARENT(PCLK)),
	DEF_CLK(EPD, 	        GATE(32 + 4) | PARENT(PCLK)),
	DEF_CLK(AES, 	        GATE(32 + 5) | PARENT(PCLK)),
	DEF_CLK(HASH, 	        GATE(32 + 6) | PARENT(PCLK)),
	DEF_CLK(DMIC, 	        GATE(32 + 7) | PARENT(PCLK)),

	DEF_CLK(P1,	        GATE(32 + 8)),
	DEF_CLK(P0,	        GATE(32 + 9)),
	DEF_CLK(AHB0,	        GATE(32 + 10)),
	DEF_CLK(SYS_OST,	GATE(32 + 11)),
	DEF_CLK(TCU_EXCLK,	GATE(32 + 12)),
	DEF_CLK(DLINE, 		GATE(32 + 13)),
	DEF_CLK(APB0,  		GATE(32 + 14)),
	DEF_CLK(CPU,  		GATE(32 + 15)),

	DEF_CLK(CGU_MSC_MUX,  	CGU(CGU_MSC_MUX)),
	DEF_CLK(CGU_BCH,	CGU(CGU_BCH)),
	DEF_CLK(CGU_ISP,	CGU(CGU_ISP)),
	DEF_CLK(CGU_GPU,	CGU(CGU_GPU)),
	DEF_CLK(CGU_PCM,	CGU(CGU_PCM)),
	DEF_CLK(CGU_CIM,	CGU(CGU_CIM)),
	DEF_CLK(CGU_SSI,	CGU(CGU_SSI)),
	DEF_CLK(CGU_UHC,	CGU(CGU_UHC)),
	DEF_CLK(CGU_MSC2,	CGU(CGU_MSC2) | PARENT(CGU_MSC_MUX)),
	DEF_CLK(CGU_MSC1,	CGU(CGU_MSC1) | PARENT(CGU_MSC_MUX)),
	DEF_CLK(CGU_MSC0,	CGU(CGU_MSC0) | PARENT(CGU_MSC_MUX)),
	DEF_CLK(CGU_LPC,	CGU(CGU_LPC)),
	DEF_CLK(CGU_I2S,	CGU(CGU_I2S)),
	DEF_CLK(CGU_USB,	CGU(CGU_USB)),
	DEF_CLK(CGU_VPU,	CGU(CGU_VPU)),
	DEF_CLK(CGU_DDR,	CGU(CGU_DDR)),

	DEF_CLK(PWC_P0,         PWC(PWC_P0) | RELATIVE(P0)),
	DEF_CLK(PWC_P1,         PWC(PWC_P1) | RELATIVE(P1)),
	DEF_CLK(PWC_VPU,        PWC(PWC_VPU) | RELATIVE(VPU)),
	DEF_CLK(PWC_GPU,        PWC(PWC_GPU) | RELATIVE(GPU)),
	DEF_CLK(PWC_ISP,        PWC(PWC_ISP) | RELATIVE(ISP)),
	DEF_CLK(PWC_IPU,        PWC(PWC_IPU) | RELATIVE(IPU)),
	DEF_CLK(PWC_DMIC,       PWC(PWC_DMIC) | RELATIVE(DMIC)),
	DEF_CLK(PWC_BCH,        PWC(PWC_BCH)  | RELATIVE(BCH)),
	DEF_CLK(PWC_HASH,       PWC(PWC_HASH) | RELATIVE(HASH)),
	DEF_CLK(PWC_LCD,        PWC(PWC_LCD) | RELATIVE(LCD)),
	DEF_CLK(PWC_USB,        PWC(PWC_USB) | RELATIVE(CGU_USB)),
	DEF_CLK(PWC_UHC,        PWC(PWC_UHC) | RELATIVE(CGU_UHC)),

#undef GATE
#undef CPCCR
#undef CGU
#undef PWC
#undef PARENT
#undef DEF_CLK
#undef RELATIVE
};
int clk_suspend(void)
{
	printk("clk suspend!\n");
	return 0;
}

void clk_resume(void)
{
	printk("clk resume!\n");
}
int clk_sleep_pm_callback(struct notifier_block *nfb,unsigned long action,void *ignored)
{
	switch (action) {
	case PM_SUSPEND_PREPARE:
		printk("clk_sleep_pm_callback PM_SUSPEND_PREPARE\n");
		cpm_pwc_suspend();
		break;
	case PM_POST_SUSPEND:
		printk("clk_sleep_pm_callback PM_POST_SUSPEND\n");
		cpm_pwc_resume();
		break;
	}
	return NOTIFY_OK;
}
static struct notifier_block clk_sleep_pm_notifier = {
	.notifier_call = clk_sleep_pm_callback,
	.priority = 0,
};
struct syscore_ops clk_pm_ops = {
	.suspend = clk_suspend,
	.resume = clk_resume,
};
static void init_clk_parent(struct clk *p) {
	int init = 0;
	if(!p)
		return;
	if(p->init_state) {
		p->count = 1;
		p->init_state = 0;
		init = 1;
	}
	if(p->count == 0) {
		printk("%s clk should be opened!\n",p->name);
		p->count = 1;
	}
	if(!init)
		p->count++;
}

void __init init_all_clk(void)
{
	int i;
	struct clk *clk_srcs = get_clk_from_id(0);
	int clk_srcs_size = get_clk_sources_size();
	cpm_pwc_init();
	for(i = 0; i < clk_srcs_size; i++) {
		if(clk_srcs[i].flags & CLK_FLG_CPCCR) {
			init_cpccr_clk(&clk_srcs[i]);
		}
		if(clk_srcs[i].flags & CLK_FLG_CGU) {
			init_cgu_clk(&clk_srcs[i]);
		}
		if(clk_srcs[i].flags & CLK_FLG_PLL) {
			init_ext_pll(&clk_srcs[i]);
		}
		if(clk_srcs[i].flags & CLK_FLG_NOALLOC) {
			init_ext_pll(&clk_srcs[i]);
		}
		if(clk_srcs[i].flags & CLK_FLG_GATE) {
			init_gate_clk(&clk_srcs[i]);
		}
		if(clk_srcs[i].flags & CLK_FLG_ENABLE)
			clk_srcs[i].init_state = 1;

		if(clk_srcs[i].flags & CLK_FLG_PWC) {
			init_pwc_clk(&clk_srcs[i]);
		}

	}
	for(i = 0; i < clk_srcs_size; i++) {
		if(clk_srcs[i].parent && clk_srcs[i].init_state)
			init_clk_parent(clk_srcs[i].parent);
	}
	register_syscore_ops(&clk_pm_ops);
	register_pm_notifier(&clk_sleep_pm_notifier);
	printk("CCLK:%luMHz L2CLK:%luMhz H0CLK:%luMHz H2CLK:%luMhz PCLK:%luMhz\n",
			clk_srcs[CLK_ID_CCLK].rate/1000/1000,
			clk_srcs[CLK_ID_L2CLK].rate/1000/1000,
			clk_srcs[CLK_ID_H0CLK].rate/1000/1000,
			clk_srcs[CLK_ID_H2CLK].rate/1000/1000,
			clk_srcs[CLK_ID_PCLK].rate/1000/1000);

}
struct clk *clk_get(struct device *dev, const char *id)
{
	int i;
	struct clk *retval = NULL;
	struct clk *clk_srcs = get_clk_from_id(0);
	struct clk *parent_clk = NULL;
	for(i = 0; i < get_clk_sources_size(); i++) {
		if(id && clk_srcs[i].name && !strcmp(id,clk_srcs[i].name)) {
			if(clk_srcs[i].flags & CLK_FLG_NOALLOC)
				return &clk_srcs[i];
			retval = kzalloc(sizeof(struct clk),GFP_KERNEL);
			if(!retval)
				return ERR_PTR(-ENODEV);
			memcpy(retval,&clk_srcs[i],sizeof(struct clk));
			retval->source = &clk_srcs[i];
			if(CLK_FLG_RELATIVE & clk_srcs[i].flags)
			{
				parent_clk = get_clk_from_id(CLK_RELATIVE(clk_srcs[i].flags));
				parent_clk->child = NULL;
			}
			retval->count = 0;
			return retval;
		}
	}
	return ERR_PTR(-EINVAL);
}
EXPORT_SYMBOL(clk_get);

int clk_enable(struct clk *clk)
{
	if(!clk)
		return -EINVAL;
	if(clk->source) {
		clk->count = 1;
		clk = clk->source;

		if(clk->init_state) {
			clk->count = 1;
			clk->init_state = 0;
			return 0;
		}
	}
	if(clk->count == 0) {
		if(clk->parent)
			clk_enable(clk->parent);
		if(clk->ops && clk->ops->enable) {
			clk->ops->enable(clk,1);
			if(clk->child)
				cpm_pwc_enable_ctrl(clk->child,1);
		}
		clk->flags |= CLK_FLG_ENABLE;
	}
	clk->count++;
	return 0;
}
EXPORT_SYMBOL(clk_enable);

int clk_is_enabled(struct clk *clk)
{
	if(clk->source)
		clk = clk->source;
	return !!(clk->flags & CLK_FLG_ENABLE);
}
EXPORT_SYMBOL(clk_is_enabled);
void clk_disable(struct clk *clk)
{
	if(!clk)
		return;
	if(clk->count == 0)
		return;
	if(clk->source) {
		clk->count = 0;
		clk = clk->source;
	}
	clk->count--;
	if(clk->count > 0) {
		return;
	}else
		clk->count = 0;
	if(!clk->count) {
		if(clk->child)
			cpm_pwc_enable_ctrl(clk->child,0);
		if(clk->ops && clk->ops->enable)
			clk->ops->enable(clk,0);
		clk->flags &= ~CLK_FLG_ENABLE;
		if(clk->parent)
			clk_disable(clk->parent);
	}
}
EXPORT_SYMBOL(clk_disable);

unsigned long clk_get_rate(struct clk *clk)
{
	if (!clk)
		return -EINVAL;
	if(clk->source)
		clk = clk->source;
	return clk? clk->rate: 0;
}
EXPORT_SYMBOL(clk_get_rate);

void clk_put(struct clk *clk)
{
	struct clk *parent_clk;
	if(clk && !(clk->flags & CLK_FLG_NOALLOC)) {
		if(clk->source && clk->count && clk->source->count > 0) {
			clk->source->count--;
			if(clk->source->count == 0)
				clk->source->init_state = 1;

		}
		if(CLK_FLG_RELATIVE & clk->source->flags)
		{
			parent_clk = get_clk_from_id(CLK_RELATIVE(clk->source->flags));
			parent_clk->child = clk->source;
		}
		kfree(clk);
	}
}
EXPORT_SYMBOL(clk_put);

int clk_set_rate(struct clk *clk, unsigned long rate)
{
	int ret = 0;
	if (!clk)
		return -EINVAL;
	if(clk->source)
		clk = clk->source;
	if (!clk->ops || !clk->ops->set_rate)
		return -EINVAL;
	if(clk->rate != rate)
		ret = clk->ops->set_rate(clk, rate);
	return ret;
}
EXPORT_SYMBOL(clk_set_rate);

int clk_set_parent(struct clk *clk, struct clk *parent)
{
	int err;

	if (!clk)
		return -EINVAL;
	if(clk->source)
		clk = clk->source;
	if (!clk->ops || !clk->ops->set_rate)
		return -EINVAL;

	err = clk->ops->set_parent(clk, parent);
	clk->rate = clk->ops->get_rate(clk);
	return err;
}
EXPORT_SYMBOL(clk_set_parent);
struct clk *clk_get_parent(struct clk *clk)
{
	if (!clk)
		return NULL;
	return clk->source->parent;
}
EXPORT_SYMBOL(clk_get_parent);
//////////////////////////clk_proc_fops////////////////////////////
static int clk_proc_show(struct seq_file *m, void *v)
{
	int i;
	int len = 0;
	len += seq_printf(m,"ID NAME       FRE        stat       count     parent\n");
	for(i = 0; i < ARRAY_SIZE(clk_srcs); i++) {
		if (clk_srcs[i].name == NULL) {
			len += seq_printf(m ,"--------------------------------------------------------\n");
		} else {
			unsigned int mhz = clk_srcs[i].rate / 10000;
			len += seq_printf(m,"%2d %-10s %4d.%02dMHz %3sable   %d %s\n",i,clk_srcs[i].name
				    , mhz/100, mhz%100
				    , clk_srcs[i].flags & CLK_FLG_ENABLE? "en": "dis"
				    , clk_srcs[i].count
				    , clk_srcs[i].parent? clk_srcs[i].parent->name: "root");
		}
	}
	len += seq_printf(m ,"CLKGR\t: %08x\n",cpm_inl(CPM_CLKGR));
	len += seq_printf(m ,"CLKGR1\t: %08x\n",cpm_inl(CPM_CLKGR1));
	len += seq_printf(m ,"LCR1\t: %08x\n",cpm_inl(CPM_LCR));
	len += seq_printf(m ,"PGR\t: %08x\n",cpm_inl(CPM_PGR));
	len += seq_printf(m ,"SPCR0\t: %08x\n",cpm_inl(CPM_SPCR0));
	return len;
}
static int clk_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, clk_proc_show, PDE_DATA(inode));
}

/////////////////////////enable_proc_fops////////////////////////
static int clk_enable_proc_write(struct file *file, const char __user *buffer,size_t count, loff_t *data)
{
	struct clk *clk = file->private_data;
	if(clk) {
		if(count && (buffer[0] == '1'))
			clk_enable(clk);
		else if(count && (buffer[0] == '0'))
			clk_disable(clk);
		else
			printk("\"echo 1 > enable\" or \"echo 0 > enable \" ");
	}
	return count;
}

static int clk_enable_proc_show(struct seq_file *m, void *v)
//static int clk_enable_proc_read(char *page, char **start, off_t off,int count, int *eof, void *data)
{
	struct clk *clk = m->private;
	int len;
	len = seq_printf(m,"%s\n",clk_is_enabled(clk) ? "enabled" : "disabled");
	return len;
}

static int clk_enable_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, clk_enable_proc_show, PDE_DATA(inode));
}
///////////////////////rate_proc_fops////////////////////////////
static int clk_set_rate_proc_write(struct file *file, const char __user *buffer,size_t count, loff_t *data)
{
	struct clk *clk = (struct clk *)data;
	long rate;
	if(clk) {
		if(kstrtol_from_user(buffer,count,0,&rate) >= 0) {
			clk_set_rate(clk,rate);
		}else
			printk("\"echo 100000000 > rate");
	}
	return count;
}

static int clk_set_rate_proc_show(struct seq_file *m, void *v)
//static int clk_set_rate_proc_read(struct file *file, char __user *user, size_t length,loff_t *loff)
{
	struct clk *clk = m->private;
	int len;
	len = seq_printf(m,"rate: %ld\n",clk_get_rate(clk));
	return len;
}
static int clk_set_rate_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, clk_set_rate_proc_show, PDE_DATA(inode));
}
///////////////////////////////////////////////////////////////////
static const struct file_operations enable_proc_fops ={
	.read = seq_read,
	.open = clk_enable_proc_open,
	.write = clk_enable_proc_write,
	.llseek = seq_lseek,
	.release = single_release,
};

static const struct file_operations rate_proc_fops ={
	.read = seq_read,
        .open = clk_set_rate_proc_open,
	.write = clk_set_rate_proc_write,
	.llseek = seq_lseek,
	.release = single_release,
};

struct list_clk
{
	struct list_head head;
	struct clk *clk;
	struct proc_dir_entry *p_entry;
};
LIST_HEAD(ctrl_clk_top);

static struct list_clk* ctrl_clk_get_exist(char *name)
{
	struct list_clk *pos_clk;
	list_for_each_entry(pos_clk,&ctrl_clk_top,head) {
		if(strcmp(pos_clk->clk->name,name) == 0) {
			return pos_clk;
		}
	}
	return NULL;
}

static int clk_get_proc_show(struct seq_file *m, void *v)
//static int clk_get_proc_read(char *page, char **start, off_t off,
//		int count, int *eof, void *data)
{
	struct list_clk *pos_clk;
	int len = 0;
	list_for_each_entry(pos_clk,&ctrl_clk_top,head) {
		len += seq_printf(m,"%s \t",pos_clk->clk->name);
	}
	len += seq_printf(m ,"\n");
	return len;
}


static int clk_get_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, clk_get_proc_show, PDE_DATA(inode));
}


static void get_str_from_user(unsigned char *str,int strlen,const char *buffer,unsigned long count)
{
	int len = count > strlen-1 ? strlen-1 : count;
	int i;
	if(len == 0) {
		str[0] = 0;
		return;
	}
	copy_from_user(str,buffer,len);
	str[len] = 0;
	for(i = len;i >= 0;i--) {
		if((str[i] == '\r') || (str[i] == '\n'))
			str[i] = 0;
	}
}

static int clk_get_proc_write(struct file *file, const char __user *buffer,
		size_t count, loff_t *data)
{
	struct list_clk *l_clk;
	struct list_head *c_clk_top = &ctrl_clk_top;
	struct proc_dir_entry *p = (struct proc_dir_entry *)data;
	char str[21];
	get_str_from_user(str,21,buffer,count);
	if(strlen(str) > 0 && !ctrl_clk_get_exist(str)) {
		l_clk = vmalloc(sizeof(struct list_clk));
		list_add_tail(&l_clk->head,c_clk_top);
		l_clk->clk = clk_get(NULL,str);
		if(IS_ERR(l_clk->clk)) {
			list_del(&l_clk->head);
			vfree(l_clk);
		}else
		{
			p = proc_mkdir(str,p);
			l_clk->p_entry = p;
			//			res = create_proc_entry("enable", 0666, p);
			proc_create("enable",0666,p,&enable_proc_fops);
			/*			if (res) {
						res->read_proc = clk_enable_proc_read;
						res->write_proc = clk_enable_proc_write;
						res->data = (void *)l_clk->clk;
						}
						*/
			//			res = create_proc_entry("rate", 0666, p);
			proc_create("rate",0666,p,&rate_proc_fops);
			/*			if (res) {
						res->read_proc = clk_set_rate_proc_read;
						res->write_proc = clk_set_rate_proc_write;
						res->data = (void *)l_clk->clk;
						*/

		}
	}


	return count;
}

/*
static int clk_put_proc_show(struct seq_file *m, void *v)

static int proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, clk_put_proc_show, PDE_DATA(inode));
}
*/

static ssize_t clk_put_proc_write(struct file *file, const char __user *buffer,
		size_t count, loff_t *data)
{
	struct list_clk *l_clk;
	struct proc_dir_entry *p = (struct proc_dir_entry *)data;
	char str[21];
	get_str_from_user(str,21,buffer,count);
	l_clk = ctrl_clk_get_exist(str);
	if(strlen(str) > 0 && l_clk) {
		list_del(&l_clk->head);
		clk_put(l_clk->clk);
		remove_proc_entry("enable",l_clk->p_entry);
		remove_proc_entry("rate",l_clk->p_entry);
		vfree(l_clk);
		remove_proc_entry(str,p);
	}
	return count;
}

static const struct file_operations clocks_proc_fops ={
	.read = seq_read,
	.open = clk_proc_open,
//	.write = clk_put_proc_write,
	.llseek = seq_lseek,
	.release = single_release,
};

static const struct file_operations getclk_proc_fops ={
	.read = seq_read,
	.open = clk_get_proc_open,
	.write = clk_get_proc_write,
	.llseek = seq_lseek,
	.release = single_release,
};

static const struct file_operations putclk_proc_fops ={
//	.read = seq_read,
//	.open = proc_open,
	.write = clk_put_proc_write,
	.llseek = seq_lseek,
	.release = single_release,
};

static int __init init_clk_proc(void)
{
	struct proc_dir_entry *p;

	p = jz_proc_mkdir("clock");
	if (!p) {
		pr_warning("create_proc_entry for common clock failed.\n");
		return -ENODEV;
	}
	proc_create("clocks", 0444,p,&clocks_proc_fops);
/*	res = create_proc_entry("clocks", 0444, p);
	if (res) {
		res->read_proc = clk_read_proc;
		res->write_proc = NULL;
		res->data = NULL;
	}
*/	
	proc_create("get_clk", 0600,p,&getclk_proc_fops);
/*	res = create_proc_entry("get_clk", 0600, p);
	if (res) {
		res->read_proc = clk_get_proc_read;
		res->write_proc = clk_get_proc_write;
		res->data = (void *)p;
	}
*/
	proc_create("put_clk", 0600,p,&putclk_proc_fops);
/*	res = create_proc_entry("put_clk", 0600, p);
	if (res) {
		res->read_proc = NULL;
		res->write_proc = clk_put_proc_write;
		res->data = (void *)p;
	}
*/
	return 0;
}

module_init(init_clk_proc);
