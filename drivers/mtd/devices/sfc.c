/*
 * SFC controller for SPI protocol, use FIFO and DMA;
 *
 * Copyright (c) 2015 Ingenic
 * Author: <xiaoyang.fu@ingenic.com>
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
*/

#include <linux/init.h>
#include <linux/spinlock.h>
#include <linux/workqueue.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/dmaengine.h>
#include <linux/dma-mapping.h>
#include <linux/sched.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/gpio.h>
#include <linux/platform_device.h>
#include <linux/spi/spi.h>
#include <linux/spi/spi_bitbang.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>

#include <asm/uaccess.h>
#include <asm/irq.h>
#include <asm/io.h>

#include "sfc.h"




#define GET_PHYADDR(a)  \
({						\
	unsigned int v;        \
	if (unlikely((unsigned int)(a) & 0x40000000)) {    \
	v = page_to_phys(vmalloc_to_page((const void *)(a))) | ((unsigned int)(a) & ~PAGE_MASK); \
	} else     \
	v = ((unsigned int)(a) & 0x1fffffff);                   \
	v;                                             \
 })
static void sfc_writel(struct sfc *sfc, unsigned short offset, u32 value)
{
	writel(value, sfc->iomem + offset);
}

static unsigned int sfc_readl(struct sfc *sfc, unsigned short offset)
{
	return readl(sfc->iomem + offset);
}

void dump_sfc_reg(struct sfc *sfc)
{
	int i = 0;
	printk("SFC_GLB			:%08x\n", sfc_readl(sfc, SFC_GLB ));
	printk("SFC_DEV_CONF	:%08x\n", sfc_readl(sfc, SFC_DEV_CONF ));
	printk("SFC_DEV_STA_EXP	:%08x\n", sfc_readl(sfc, SFC_DEV_STA_EXP));
	printk("SFC_DEV_STA_RT	:%08x\n", sfc_readl(sfc, SFC_DEV_STA_RT ));
	printk("SFC_DEV_STA_MSK	:%08x\n", sfc_readl(sfc, SFC_DEV_STA_MSK ));
	printk("SFC_TRAN_LEN		:%08x\n", sfc_readl(sfc, SFC_TRAN_LEN ));

	for(i = 0; i < 6; i++)
		printk("SFC_TRAN_CONF(%d)	:%08x\n", i,sfc_readl(sfc, SFC_TRAN_CONF(i)));

	for(i = 0; i < 6; i++)
		printk("SFC_DEV_ADDR(%d)	:%08x\n", i,sfc_readl(sfc, SFC_DEV_ADDR(i)));

	printk("SFC_MEM_ADDR :%08x\n", sfc_readl(sfc, SFC_MEM_ADDR ));
	printk("SFC_TRIG	 :%08x\n", sfc_readl(sfc, SFC_TRIG));
	printk("SFC_SR		 :%08x\n", sfc_readl(sfc, SFC_SR));
	printk("SFC_SCR		 :%08x\n", sfc_readl(sfc, SFC_SCR));
	printk("SFC_INTC	 :%08x\n", sfc_readl(sfc, SFC_INTC));
	printk("SFC_FSM		 :%08x\n", sfc_readl(sfc, SFC_FSM ));
	printk("SFC_CGE		 :%08x\n", sfc_readl(sfc, SFC_CGE ));
//	printk("SFC_RM_DR 	 :%08x\n", sfc_readl(spi, SFC_RM_DR));
}
static void dump_reg(struct sfc *sfc)
{
	printk("SFC_GLB = %08x\n",sfc_readl(sfc,0x0000));
	printk("SFC_DEV_CONF = %08x\n",sfc_readl(sfc,0x0004));
	printk("SFC_DEV_STA_EXP = %08x\n",sfc_readl(sfc,0x0008));
	printk("SFC_DEV_STA_RT	 = %08x\n",sfc_readl(sfc,0x000c));
	printk("SFC_DEV_STA_MASK = %08x\n",sfc_readl(sfc,0x0010));
	printk("SFC_TRAN_CONF0 = %08x\n",sfc_readl(sfc,0x0014));
	printk("SFC_TRAN_LEN = %08x\n",sfc_readl(sfc,0x002c));
	printk("SFC_DEV_ADDR0 = %08x\n",sfc_readl(sfc,0x0030));
	printk("SFC_DEV_ADDR_PLUS0 = %08x\n",sfc_readl(sfc,0x0048));
	printk("SFC_MEM_ADDR = %08x\n",sfc_readl(sfc,0x0060));
	printk("SFC_TRIG = %08x\n",sfc_readl(sfc,0x0064));
	printk("SFC_SR = %08x\n",sfc_readl(sfc,0x0068));
	printk("SFC_SCR = %08x\n",sfc_readl(sfc,0x006c));
	printk("SFC_INTC = %08x\n",sfc_readl(sfc,0x0070));
	printk("SFC_FSM = %08x\n",sfc_readl(sfc,0x0074));
	printk("SFC_CGE = %08x\n",sfc_readl(sfc,0x0078));
//	printk("SFC_DR = %08x\n",sfc_readl(sfc,0x1000));
}


void sfc_init(struct sfc *sfc)
{
	int n;
	for(n = 0; n < N_MAX; n++) {
		sfc_writel(sfc, SFC_TRAN_CONF(n), 0);
		sfc_writel(sfc, SFC_DEV_ADDR(n), 0);
		sfc_writel(sfc, SFC_DEV_ADDR_PLUS(n), 0);
	}

	//sfc_writel(sfc, SFC_GLB, ((1 << 7) | (1 << 3)));
	sfc_writel(sfc, SFC_DEV_CONF, 0);
	sfc_writel(sfc, SFC_DEV_STA_EXP, 0);
	sfc_writel(sfc, SFC_DEV_STA_MSK, 0);
	sfc_writel(sfc, SFC_TRAN_LEN, 0);
	sfc_writel(sfc, SFC_MEM_ADDR, 0);
	sfc_writel(sfc, SFC_TRIG, 0);
	sfc_writel(sfc, SFC_SCR, 0);
	sfc_writel(sfc, SFC_INTC, 0);
	sfc_writel(sfc, SFC_CGE, 0);
	sfc_writel(sfc, SFC_RM_DR, 0);
}

void sfc_stop(struct sfc*sfc)
{
	unsigned int tmp;
	tmp = sfc_readl(sfc, SFC_TRIG);
	tmp |= TRIG_STOP;
	sfc_writel(sfc, SFC_TRIG, tmp);
}

void sfc_start(struct sfc *sfc)
{
	unsigned int tmp;
	tmp = sfc_readl(sfc, SFC_TRIG);
	tmp |= TRIG_START;
	sfc_writel(sfc, SFC_TRIG, tmp);
}

void sfc_flush_fifo(struct sfc *sfc)
{
	unsigned int tmp;
	tmp = sfc_readl(sfc, SFC_TRIG);
	tmp |= TRIG_FLUSH;
	sfc_writel(sfc, SFC_TRIG, tmp);
}

void sfc_ce_invalid_value(struct sfc *sfc, int value)
{
	if(value == 0) {
		unsigned int tmp;
		tmp = sfc_readl(sfc, SFC_DEV_CONF);
		tmp &= ~DEV_CONF_CEDL;
		sfc_writel(sfc, SFC_DEV_CONF, tmp);
	} else {
		unsigned int tmp;
		tmp = sfc_readl(sfc, SFC_DEV_CONF);
		tmp |= DEV_CONF_CEDL;
		sfc_writel(sfc, SFC_DEV_CONF, tmp);
	}
}

void sfc_hold_invalid_value(struct sfc *sfc, int value)
{
	if(value == 0) {
		unsigned int tmp;
		tmp = sfc_readl(sfc, SFC_DEV_CONF);
		tmp &= ~DEV_CONF_HOLDDL;
		sfc_writel(sfc, SFC_DEV_CONF, tmp);
	} else {
		unsigned int tmp;
		tmp = sfc_readl(sfc, SFC_DEV_CONF);
		tmp |= DEV_CONF_HOLDDL;
		sfc_writel(sfc, SFC_DEV_CONF, tmp);
	}
}

void sfc_wp_invalid_value(struct sfc *sfc, int value)
{
	if(value == 0) {
		unsigned int tmp;
		tmp = sfc_readl(sfc, SFC_DEV_CONF);
		tmp &= ~DEV_CONF_WPDL;
		sfc_writel(sfc, SFC_DEV_CONF, tmp);
	} else {
		unsigned int tmp;
		tmp = sfc_readl(sfc, SFC_DEV_CONF);
		tmp |= DEV_CONF_WPDL;
		sfc_writel(sfc, SFC_DEV_CONF, tmp);
	}
}

void sfc_clear_end_intc(struct sfc *sfc)
{
	int tmp = 0;
	tmp = sfc_readl(sfc, SFC_SCR);
	tmp |= CLR_END;
	sfc_writel(sfc, SFC_SCR, tmp);
	tmp = sfc_readl(sfc, SFC_SCR);
}

void sfc_clear_treq_intc(struct sfc *sfc)
{
	int tmp = 0;
	tmp = sfc_readl(sfc, SFC_SCR);
	tmp |= CLR_TREQ;
	sfc_writel(sfc, SFC_SCR, tmp);
}

void sfc_clear_rreq_intc(struct sfc *sfc)
{
	int tmp = 0;
	tmp = sfc_readl(sfc, SFC_SCR);
	tmp |= CLR_RREQ;
	sfc_writel(sfc, SFC_SCR, tmp);
}

void sfc_clear_over_intc(struct sfc *sfc)
{
	int tmp = 0;
	tmp = sfc_readl(sfc, SFC_SCR);
	tmp |= CLR_OVER;
	sfc_writel(sfc, SFC_SCR, tmp);
}

void sfc_clear_under_intc(struct sfc *sfc)
{
	int tmp = 0;
	tmp = sfc_readl(sfc, SFC_SCR);
	tmp |= CLR_UNDER;
	sfc_writel(sfc, SFC_SCR, tmp);
}

void sfc_clear_all_intc(struct sfc *sfc)
{
	sfc_writel(sfc, SFC_SCR, 0x1f);
}

void sfc_mask_all_intc(struct sfc *sfc)
{
	sfc_writel(sfc, SFC_INTC, 0x1f);
}

void sfc_mode(struct sfc *sfc, int channel, int value)
{
	unsigned int tmp;
	tmp = sfc_readl(sfc, SFC_TRAN_CONF(channel));
	tmp &= ~(TRAN_CONF_TRAN_MODE_MSK << TRAN_CONF_TRAN_MODE_OFFSET);
	tmp |= (value << TRAN_CONF_TRAN_MODE_OFFSET);
	sfc_writel(sfc, SFC_TRAN_CONF(channel), tmp);
}
void sfc_set_phase_num(struct sfc *sfc,int num)
{
	unsigned int tmp;

	tmp = sfc_readl(sfc, SFC_GLB);
	tmp &= ~GLB_PHASE_NUM_MSK;
	tmp |= num << GLB_PHASE_NUM_OFFSET;
	sfc_writel(sfc, SFC_GLB, tmp);
}
void sfc_clock_phase(struct sfc *sfc, int value)
{
	if(value == 0) {
		unsigned int tmp;
		tmp = sfc_readl(sfc, SFC_DEV_CONF);
		tmp &= ~DEV_CONF_CPHA;
		sfc_writel(sfc, SFC_DEV_CONF, tmp);
	} else {
		unsigned int tmp;
		tmp = sfc_readl(sfc, SFC_DEV_CONF);
		tmp |= DEV_CONF_CPHA;
		sfc_writel(sfc, SFC_DEV_CONF, tmp);
	}
}

void sfc_clock_polarity(struct sfc *sfc, int value)
{
	if(value == 0) {
		unsigned int tmp;
		tmp = sfc_readl(sfc, SFC_DEV_CONF);
		tmp &= ~DEV_CONF_CPOL;
		sfc_writel(sfc, SFC_DEV_CONF, tmp);
	} else {
		unsigned int tmp;
		tmp = sfc_readl(sfc, SFC_DEV_CONF);
		tmp |= DEV_CONF_CPOL;
		sfc_writel(sfc, SFC_DEV_CONF, tmp);
	}
}

void sfc_threshold(struct sfc *sfc, int value)
{
	unsigned int tmp;
	tmp = sfc_readl(sfc, SFC_GLB);
	tmp &= ~GLB_THRESHOLD_MSK;
	tmp |= value << GLB_THRESHOLD_OFFSET;
	sfc_writel(sfc, SFC_GLB, tmp);
}


void sfc_smp_delay(struct sfc *sfc, int value)
{
	unsigned int tmp;
	tmp = sfc_readl(sfc, SFC_DEV_CONF);
	tmp &= ~DEV_CONF_SMP_DELAY_MSK;
	tmp |= value << DEV_CONF_SMP_DELAY_OFFSET | 1 << 9;
	sfc_writel(sfc, SFC_DEV_CONF, tmp);
}


void sfc_transfer_direction(struct sfc *sfc, int value)
{
	if(value == GLB_TRAN_DIR_READ) {
		unsigned int tmp;
		tmp = sfc_readl(sfc, SFC_GLB);
		tmp &= ~GLB_TRAN_DIR;
		sfc_writel(sfc, SFC_GLB, tmp);
	} else {
		unsigned int tmp;
		tmp = sfc_readl(sfc, SFC_GLB);
		tmp |= GLB_TRAN_DIR;
		sfc_writel(sfc, SFC_GLB, tmp);
	}
}

void sfc_set_length(struct sfc *sfc, int value)
{
	sfc_writel(sfc, SFC_TRAN_LEN, value);
}

void sfc_transfer_mode(struct sfc *sfc, int value)
{
	if(value == 0) {
		unsigned int tmp;
		tmp = sfc_readl(sfc, SFC_GLB);
		tmp &= ~GLB_OP_MODE;
		sfc_writel(sfc, SFC_GLB, tmp);
	} else {
		unsigned int tmp;
		tmp = sfc_readl(sfc, SFC_GLB);
		tmp |= GLB_OP_MODE;
		sfc_writel(sfc, SFC_GLB, tmp);
	}
}

void sfc_read_data(struct sfc *sfc, unsigned int *value)
{
	*value = sfc_readl(sfc, SFC_RM_DR);
}

void sfc_write_data(struct sfc *sfc, const unsigned int value)
{
	sfc_writel(sfc, SFC_RM_DR, value);
}

unsigned int sfc_fifo_num(struct sfc *sfc)
{
	unsigned int tmp;
	tmp = sfc_readl(sfc, SFC_SR);
	tmp &= (0x7f << 16);
	tmp = tmp >> 16;
	return tmp;
}

static unsigned int cpu_read_rxfifo(struct sfc *sfc)
{
	int i;
	unsigned long align_len = 0;
	unsigned int fifo_num = 0;
	unsigned int ndummy = 0;
	unsigned int data[1] = {0};
	unsigned int last_word = 0;

	align_len = ALIGN(sfc->transfer->len, 4);

	if (((align_len - sfc->transfer->tmp_len) / 4) > THRESHOLD){
		fifo_num = THRESHOLD;
		last_word = 0;
		ndummy = 0;
	} else {
		/* last aligned THRESHOLD data*/
		if(sfc->transfer->len % 4) {
			fifo_num = (align_len - sfc->transfer->tmp_len) / 4 - 1 ;
			last_word = 1;
		} else {
			fifo_num = (align_len - sfc->transfer->tmp_len) / 4;
			last_word = 0;
		}
		ndummy = THRESHOLD - fifo_num - last_word;
	}

	for(i = 0; i < fifo_num; i++) {
		sfc_read_data(sfc, (unsigned int *)sfc->transfer->data);
		sfc->transfer->data += 4;
		sfc->transfer->tmp_len += 4;
	}

	/* last word */
	if(last_word == 1) {
		sfc_read_data(sfc, data);
		memcpy((void *)sfc->transfer->data, data, sfc->transfer->len % 4);

		sfc->transfer->data += sfc->transfer->len % 4;
		sfc->transfer->tmp_len += 4;

	}
	for(i = 0; i < ndummy; i++) {
		sfc_read_data(sfc, data);
	}

	return 0;
}

static unsigned int cpu_write_txfifo(struct sfc *sfc)
{
	int i;
	unsigned long align_len = 0;
	unsigned int fifo_num = 0;


	align_len = ALIGN(sfc->transfer->len , 4);

	if (((align_len - sfc->transfer->tmp_len) / 4) > THRESHOLD){
		fifo_num = THRESHOLD;
	} else {
		fifo_num = (align_len - sfc->transfer->tmp_len) / 4;
	}

	for(i = 0; i < fifo_num; i++) {
		sfc_write_data(sfc, *(unsigned int *)sfc->transfer->data);
		sfc->transfer->data += 4;
		sfc->transfer->tmp_len += 4;
	}

	return 0;
}


int ssi_underrun(struct sfc *sfc)
{
	unsigned int tmp;
	tmp = sfc_readl(sfc, SFC_SR);
	if(tmp & CLR_UNDER)
		return 1;
	else
		return 0;
}

int ssi_overrun(struct sfc *sfc)
{
	unsigned int tmp;
	tmp = sfc_readl(sfc, SFC_SR);
	if(tmp & CLR_OVER)
		return 1;
	else
		return 0;
}

int rxfifo_rreq(struct sfc *sfc)
{
	unsigned int tmp;
	tmp = sfc_readl(sfc, SFC_SR);
	if(tmp & CLR_RREQ)
		return 1;
	else
		return 0;
}
int txfifo_treq(struct sfc *sfc)
{
	unsigned int tmp;
	tmp = sfc_readl(sfc, SFC_SR);
	if(tmp & CLR_TREQ)
		return 1;
	else
		return 0;
}
int sfc_end(struct sfc *sfc)
{
	unsigned int tmp;
	tmp = sfc_readl(sfc, SFC_SR);
	if(tmp & CLR_END)
		return 1;
	else
		return 0;
}
unsigned int sfc_get_sta_rt(struct sfc *sfc)
{
	return sfc_readl(sfc,SFC_DEV_STA_RT);
}
unsigned int sfc_get_fsm(struct sfc *sfc)
{
	return sfc_readl(sfc,SFC_FSM);
}
void sfc_set_addr_length(struct sfc *sfc, int channel, unsigned int value)
{
	unsigned int tmp;
	tmp = sfc_readl(sfc, SFC_TRAN_CONF(channel));
	tmp &= ~(ADDR_WIDTH_MSK);
	tmp |= (value << ADDR_WIDTH_OFFSET);
	sfc_writel(sfc, SFC_TRAN_CONF(channel), tmp);
}

void sfc_cmd_enble(struct sfc *sfc, int channel, unsigned int value)
{
	if(value == ENABLE) {
		unsigned int tmp;
		tmp = sfc_readl(sfc, SFC_TRAN_CONF(channel));
		tmp |= TRAN_CONF_CMDEN;
		sfc_writel(sfc, SFC_TRAN_CONF(channel), tmp);
	} else {
		unsigned int tmp;
		tmp = sfc_readl(sfc, SFC_TRAN_CONF(channel));
		tmp &= ~TRAN_CONF_CMDEN;
		sfc_writel(sfc, SFC_TRAN_CONF(channel), tmp);
	}
}

void sfc_data_en(struct sfc *sfc, int channel, unsigned int value)
{
	if(value == 1) {
		unsigned int tmp;
		tmp = sfc_readl(sfc, SFC_TRAN_CONF(channel));
		tmp |= TRAN_CONF_DATEEN;
		sfc_writel(sfc, SFC_TRAN_CONF(channel), tmp);
	} else {
		unsigned int tmp;
		tmp = sfc_readl(sfc, SFC_TRAN_CONF(channel));
		tmp &= ~TRAN_CONF_DATEEN;
		sfc_writel(sfc, SFC_TRAN_CONF(channel), tmp);
	}
}

void sfc_write_cmd(struct sfc *sfc, int channel, unsigned int value)
{
	unsigned int tmp;
	tmp = sfc_readl(sfc, SFC_TRAN_CONF(channel));
	tmp &= ~TRAN_CONF_CMD_MSK;
	tmp |= value;
	sfc_writel(sfc, SFC_TRAN_CONF(channel), tmp);
}

void sfc_dev_addr(struct sfc *sfc, int channel, unsigned int value)
{
	sfc_writel(sfc, SFC_DEV_ADDR(channel), value);
}


void sfc_dev_addr_dummy_bytes(struct sfc *sfc, int channel, unsigned int value)
{
	unsigned int tmp;
	tmp = sfc_readl(sfc, SFC_TRAN_CONF(channel));
	tmp &= ~TRAN_CONF_DMYBITS_MSK;
	tmp |= value << DMYBITS_OFFSET;
	sfc_writel(sfc, SFC_TRAN_CONF(channel), tmp);
}

void sfc_dev_addr_plus(struct sfc *sfc, int channel, unsigned int value)
{
	sfc_writel(sfc, SFC_DEV_ADDR_PLUS(channel), value);
}

void sfc_dev_pollen(struct sfc *sfc, int channel, unsigned int value)
{
	unsigned int tmp;
	tmp = sfc_readl(sfc, SFC_TRAN_CONF(channel));
	if(value == 1)
		tmp |= TRAN_CONF_POLLEN;
	else
		tmp &= ~(TRAN_CONF_POLLEN);

	sfc_writel(sfc, SFC_TRAN_CONF(channel), tmp);
}

void sfc_dev_sta_exp(struct sfc *sfc, unsigned int value)
{
	sfc_writel(sfc, SFC_DEV_STA_EXP, value);
}

void sfc_dev_sta_msk(struct sfc *sfc, unsigned int value)
{
	sfc_writel(sfc, SFC_DEV_STA_MSK, value);
}

void sfc_enable_all_intc(struct sfc *sfc)
{
	sfc_writel(sfc, SFC_INTC, 0);
}

void sfc_set_mem_addr(struct sfc *sfc,unsigned int addr )
{
	sfc_writel(sfc, SFC_MEM_ADDR, addr);
}
static int sfc_start_transfer(struct sfc *sfc)
{
	int err;
	sfc_clear_all_intc(sfc);
	sfc_enable_all_intc(sfc);
	sfc_start(sfc);
	err = wait_for_completion_timeout(&sfc->done,10*HZ);
	if (!err) {
		printk("line:%d Timeout for ACK from SFC device\n",__LINE__);
		return -ETIMEDOUT;
	}
	return 0;
}
static void sfc_phase_transfer(struct sfc *sfc,struct sfc_transfer *
		transfer,int channel)
{
	sfc_flush_fifo(sfc);
	sfc_set_addr_length(sfc,channel,transfer->cmd_info->addr_len);
	sfc_cmd_enble(sfc,channel,ENABLE);
	sfc_write_cmd(sfc,channel,transfer->cmd_info->cmd);
	sfc_dev_addr_dummy_bytes(sfc,channel,transfer->cmd_info->dummy_byte);
	sfc_data_en(sfc,channel,transfer->cmd_info->dataen);
	sfc_dev_addr(sfc, channel,transfer->addr);
	sfc_dev_addr_plus(sfc,channel,transfer->cmd_info->addr_plus);
	sfc_mode(sfc,channel,transfer->sfc_mode);

}
static void common_cmd_request_transfer(struct sfc *sfc,struct sfc_transfer *transfer,int channel)
{
	sfc_phase_transfer(sfc,transfer,channel);
	sfc_dev_sta_exp(sfc,0);
	sfc_dev_sta_msk(sfc,0);
	sfc_dev_pollen(sfc,channel,DISABLE);
}

static void poll_cmd_request_transfer(struct sfc *sfc,struct sfc_transfer *transfer,int channel)
{
	struct cmd_info *cmd = transfer->cmd_info;
	sfc_phase_transfer(sfc,transfer,channel);
	sfc_dev_sta_exp(sfc,cmd->sta_exp);
	sfc_dev_sta_msk(sfc,cmd->sta_msk);
	sfc_dev_pollen(sfc,channel,ENABLE);
}
static void sfc_glb_info_config(struct sfc *sfc,struct sfc_transfer *transfer)
{
	sfc_transfer_direction(sfc, transfer->direction);
	if((transfer->ops_mode == DMA_OPS)){
		sfc_set_length(sfc, transfer->len);
		dma_cache_sync(NULL, (void *)transfer->data,transfer->len,transfer->direction);
		sfc_set_mem_addr(sfc, GET_PHYADDR(transfer->data));
		sfc_transfer_mode(sfc, DMA_MODE);
	}else{
		if(transfer->direction == GLB_TRAN_DIR_READ)
			sfc_set_length(sfc, ALIGN(transfer->len, THRESHOLD * 4));
		else
			sfc_set_length(sfc, transfer->len);
		sfc_set_mem_addr(sfc, 0);
		sfc_transfer_mode(sfc, SLAVE_MODE);
	}
}
static void  dump_transfer(struct sfc_transfer *xfer,int num)
{
	printk("\n");
	printk("cmd[%d].cmd_type = %d\n",num,xfer->cmd_info->cmd_type);
	printk("cmd[%d].cmd = 0x%02x\n",num,xfer->cmd_info->cmd);
	printk("cmd[%d].addr_len = %d\n",num,xfer->cmd_info->addr_len);
	printk("cmd[%d].dummy_byte = %d\n",num,xfer->cmd_info->dummy_byte);
	printk("cmd[%d].dataen = %d\n",num,xfer->cmd_info->dataen);
	printk("cmd[%d].sta_exp = %d\n",num,xfer->cmd_info->sta_exp);
	printk("cmd[%d].sta_msk = %d\n",num,xfer->cmd_info->sta_msk);


	printk("transfer[%d].addr = 0x%08x\n",num,xfer->addr);
	printk("transfer[%d].len = %d\n",num,xfer->len);
	printk("transfer[%d].data = 0x%p\n",num,xfer->data);
	printk("transfer[%d].direction = %d\n",num,xfer->direction);
	printk("transfer[%d].sfc_mode = %d\n",num,xfer->sfc_mode);
	printk("transfer[%d].ops_mode = %d\n",num,xfer->ops_mode);
}
int sfc_sync(struct sfc *sfc, struct sfc_message *message)
{
	struct sfc_transfer *xfer;
	int phase_num = 0,ret = 0;


	list_for_each_entry(xfer, &message->transfers, transfer_list) {
		if(xfer->cmd_info->sta_msk == 0){
			common_cmd_request_transfer(sfc,xfer,phase_num);
		}else{
			poll_cmd_request_transfer(sfc,xfer,phase_num);
		}
		if(xfer->cmd_info->addr_len || xfer->len)
			sfc_glb_info_config(sfc,xfer);
		phase_num++;
		message->actual_length += xfer->len;
		if(xfer->len > 0)
			sfc->transfer = xfer;
	}
	sfc_set_phase_num(sfc,phase_num);
	ret = sfc_start_transfer(sfc);
	return ret;
}


void sfc_transfer_del(struct sfc_transfer *t)
{
	list_del(&t->transfer_list);
}

void sfc_message_add_tail(struct sfc_transfer *t, struct sfc_message *m)
{
	list_add_tail(&t->transfer_list, &m->transfers);
}

void sfc_message_init(struct sfc_message *m)
{
	memset(m, 0, sizeof *m);
	INIT_LIST_HEAD(&m->transfers);
}


static irqreturn_t jz_sfc_pio_irq_callback(struct sfc *sfc)
{
	if (ssi_underrun(sfc)) {
		sfc_clear_under_intc(sfc);
		printk("sfc UNDR !\n");
		complete(&sfc->done);
		return IRQ_HANDLED;
	}

	if (ssi_overrun(sfc)) {
		sfc_clear_over_intc(sfc);
		printk("sfc OVER !\n");
		complete(&sfc->done);
		return IRQ_HANDLED;
	}

	if (rxfifo_rreq(sfc)) {
		sfc_clear_rreq_intc(sfc);
		cpu_read_rxfifo(sfc);
		return IRQ_HANDLED;
	}

	if (txfifo_treq(sfc)) {
		sfc_clear_treq_intc(sfc);
		cpu_write_txfifo(sfc);
		return IRQ_HANDLED;
	}

	if(sfc_end(sfc)){
		sfc_mask_all_intc(sfc);
		sfc_clear_end_intc(sfc);
		complete(&sfc->done);
		return IRQ_HANDLED;
	}

	return IRQ_NONE;
}
static irqreturn_t jz_sfc_irq(int irq, void *dev)
{
	struct sfc *sfc = dev;
	return sfc->irq_callback(sfc);
}

static int jz_sfc_init_setup(struct sfc *sfc)
{
	sfc_init(sfc);
	sfc_stop(sfc);

	/*set hold high*/
	sfc_hold_invalid_value(sfc, 1);
	/*set wp high*/
	sfc_wp_invalid_value(sfc, 1);

	sfc_clear_all_intc(sfc);
	sfc_mask_all_intc(sfc);

	sfc_threshold(sfc, sfc->threshold);
	/*config the sfc pin init state*/
	sfc_clock_phase(sfc, 0);
	sfc_clock_polarity(sfc, 0);
	sfc_ce_invalid_value(sfc, 1);


	sfc_transfer_mode(sfc, SLAVE_MODE);
	if(sfc->src_clk >= 100000000){
		sfc_smp_delay(sfc,DEV_CONF_HALF_CYCLE_DELAY);
	}
	sfc->irq_callback = &jz_sfc_pio_irq_callback;
	return 0;
}
struct sfc *sfc_res_init(struct platform_device *pdev)
{
	struct sfc *sfc;
	struct resource *res;
	int err;
	sfc = kzalloc(sizeof(struct sfc), GFP_KERNEL);
	if (!sfc) {
		printk("ERROR: %s %d kzalloc() error !\n",__func__,__LINE__);
		return ERR_PTR(-ENOMEM);
	}
	/* find and map our resources */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res == NULL) {
		dev_err(&pdev->dev, "Cannot get IORESOURCE_MEM\n");
		err = -ENOENT;
		goto err_no_iores;
	}

	sfc->ioarea = request_mem_region(res->start, resource_size(res),
					pdev->name);
	if (sfc->ioarea == NULL) {
		dev_err(&pdev->dev, "Cannot reserve iomem region\n");
		err = -ENXIO;
		goto err_no_iores;
	}

	sfc->phys = res->start;

	sfc->iomem = ioremap(res->start, (res->end - res->start)+1);
	if (sfc->iomem == NULL) {
		dev_err(&pdev->dev, "Cannot map IO\n");
		err = -ENXIO;
		goto err_no_iomap;
	}

	sfc->irq = platform_get_irq(pdev, 0);
	if (sfc->irq <= 0) {
		dev_err(&pdev->dev, "No IRQ specified\n");
		err = -ENOENT;
		goto err_no_irq;
	}

	sfc->clk = clk_get(&pdev->dev, "cgu_ssi");
	if (IS_ERR(sfc->clk)) {
		dev_err(&pdev->dev, "Cannot get ssi clock\n");
		goto err_no_clk;
	}

	sfc->clk_gate = clk_get(&pdev->dev, "sfc");
	if (IS_ERR(sfc->clk_gate)) {
		dev_err(&pdev->dev, "Cannot get sfc clock\n");
		goto err_no_clk;
	}

	res = platform_get_resource(pdev, IORESOURCE_BUS, 0);
	if (res == NULL) {
		dev_err(&pdev->dev, "Cannot get IORESOURCE_BUS\n");
		err = -ENOENT;
		goto err_no_iores;
	}

	sfc->src_clk = res->start * 1000000;

	if (clk_get_rate(sfc->clk) >= sfc->src_clk) {
		clk_set_rate(sfc->clk, sfc->src_clk);
	} else {
		clk_set_rate(sfc->clk, sfc->src_clk);
	}

	clk_enable(sfc->clk);
	clk_enable(sfc->clk_gate);

	sfc->threshold = THRESHOLD;
	/* request SFC irq */
	err = request_irq(sfc->irq, jz_sfc_irq, 0, pdev->name, sfc);
	if (err) {
		dev_err(&pdev->dev, "Cannot claim IRQ\n");
		goto err_no_irq;
	}

	/* SFC controller initializations for SFC */
	jz_sfc_init_setup(sfc);
	init_completion(&sfc->done);
	return sfc;

err_no_clk:
	clk_put(sfc->clk_gate);
	clk_put(sfc->clk);
err_no_irq:
	free_irq(sfc->irq,sfc);
err_no_iomap:
	iounmap(sfc->iomem);
err_no_iores:
	release_resource(sfc->ioarea);
	kfree(sfc->ioarea);
	return ERR_PTR(err);
}
