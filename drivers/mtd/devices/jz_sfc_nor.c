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
#include <linux/kernel.h>


#include <asm/uaccess.h>
#include <asm/irq.h>
#include <asm/io.h>

#include "sfc.h"


/* Max time can take up to 3 seconds! */
#define MAX_READY_WAIT_TIME 3000    /* the time of erase BE(64KB) */

#define STATUS_SUSPND	(1<<0)


#define	tCHSH	5	//hold
#define tSLCH	5	//setup
#define tSHSL_RD	20	//interval
#define tSHSL_WR	30

static int sfc_transfer_mode;

static int set_flash_timing(struct sfc_flash *flash)
{
	unsigned int t_hold, c_hold;
	unsigned int t_setup, c_setup;
	unsigned int t_in, c_in, val;
	unsigned int cycle;

	cycle = 1000000000 / flash->sfc->src_clk;

	t_hold = tCHSH;
	c_hold = t_hold / cycle;
	if(c_hold > 0)
		val = c_hold - 1;
	sfc_hold_delay(flash->sfc, val);

	t_setup = tSLCH;
	c_setup = t_setup / cycle;
	if(c_setup > 0)
		val = c_setup - 1;
	sfc_setup_delay(flash->sfc, val);

	t_in = max(tSHSL_RD, tSHSL_WR);
	c_in = t_in / cycle;
	val = c_in - 1;
	sfc_interval_delay(flash->sfc, val);

	return 0;
}



struct sfc_flash *to_jz_spi_norflash(struct mtd_info *mtd_info)
{
	return container_of(mtd_info, struct sfc_flash, mtd);
}


static int sfc_nor_read_id(struct sfc_flash *flash, u8 command, unsigned int addr, int addr_len, size_t len, int dummy_byte)
{

	struct sfc_transfer transfer;
	struct sfc_message message;
	struct cmd_info cmd;
	int ret;
	unsigned int chip_id = 0;

	sfc_message_init(&message);
	memset(&transfer, 0, sizeof(transfer));
	memset(&cmd, 0, sizeof(cmd));

	cmd.cmd = command;
	cmd.addr_len = addr_len;
	cmd.dummy_byte = dummy_byte;
	cmd.dataen = ENABLE;

	transfer.addr = addr;
	transfer.len = len;
	transfer.data =(unsigned char *)&chip_id;
	transfer.ops_mode = CPU_OPS;

	transfer.sfc_mode = TM_STD_SPI;
	transfer.direction = GLB_TRAN_DIR_READ;
	transfer.cmd_info = &cmd;
	sfc_message_add_tail(&transfer, &message);


	ret = sfc_sync(flash->sfc, &message);
	if(ret) {
		dev_err(flash->dev,"sfc_sync error ! %s %s %d\n",__FILE__,__func__,__LINE__);
		ret=-EIO;
	}

	return chip_id & 0x00ffffff;
}



static unsigned int sfc_do_read(struct sfc_flash *flash,u8 command,unsigned int addr,int addr_len,unsigned char *buf,size_t len,int dummy_byte)
{
	struct sfc_transfer transfer;
	struct sfc_message message;
	struct cmd_info cmd;
	int ret;

	sfc_message_init(&message);
	memset(&transfer, 0, sizeof(transfer));
	memset(&cmd, 0, sizeof(cmd));

	cmd.cmd = command;
	cmd.addr_len = addr_len;
	cmd.dummy_byte = dummy_byte;
	cmd.dataen = ENABLE;

	transfer.addr = addr;
	transfer.len = len;
	transfer.data = buf;
	transfer.tmp_len = 0;
	if(len > THRESHOLD * 4)
		transfer.ops_mode = DMA_OPS;
	else
		transfer.ops_mode = CPU_OPS;

	transfer.sfc_mode = sfc_transfer_mode;
	transfer.direction = GLB_TRAN_DIR_READ;
	transfer.cmd_info = &cmd;
	sfc_message_add_tail(&transfer, &message);


	ret = sfc_sync(flash->sfc, &message);
	if(ret) {
		dev_err(flash->dev,"sfc_sync error ! %s %s %d\n",__FILE__,__func__,__LINE__);
		ret=-EIO;
	}
	/*fix the cache line problem,when use jffs2 filesystem must be flush cache twice*/
	if(transfer.ops_mode == DMA_OPS)
		dma_cache_sync(NULL, (void *)buf, len, DMA_FROM_DEVICE);

	return message.actual_length;
}
static unsigned  int sfc_do_write(struct sfc_flash *flash,u8 command,unsigned int addr,int addr_len,const unsigned char *buf,size_t len,int dummy_byte)
{
	struct sfc_transfer transfer[3];
	struct sfc_message message;
	struct cmd_info cmd[3];
	int ret;

	sfc_message_init(&message);
	memset(&transfer, 0, sizeof(transfer));
	memset(&cmd, 0, sizeof(cmd));

	/* write enable */
	cmd[0].cmd = CMD_WREN;
	cmd[0].dataen = DISABLE;

	transfer[0].cmd_info = &cmd[0];
	transfer[0].sfc_mode = sfc_transfer_mode;
	sfc_message_add_tail(&transfer[0], &message);

	/* write ops */
	cmd[1].cmd = command;
	cmd[1].dummy_byte = dummy_byte;
	cmd[1].dataen = ENABLE;
	cmd[1].addr_len = addr_len;

	transfer[1].addr = addr;
	transfer[1].len = len;
	transfer[1].tmp_len = 0;
	transfer[1].data = buf;
	transfer[1].ops_mode = DMA_OPS;
	transfer[1].sfc_mode = sfc_transfer_mode;
	transfer[1].direction = GLB_TRAN_DIR_WRITE;
	transfer[1].cmd_info = &cmd[1];
	sfc_message_add_tail(&transfer[1], &message);

	cmd[2].cmd = CMD_RDSR;
	cmd[2].dataen = DISABLE;
	cmd[2].sta_exp = 0;
	cmd[2].sta_msk = 0x1;

	transfer[2].cmd_info = &cmd[2];
	sfc_message_add_tail(&transfer[2], &message);

	ret = sfc_sync(flash->sfc, &message);
	if(ret) {
		dev_err(flash->dev,"sfc_sync error ! %s %s %d\n",__FILE__,__func__,__LINE__);
		ret=-EIO;
	}

	return message.actual_length;
}
#ifdef CONFIG_SPI_QUAD
static int sfc_flash_set_quad_mode(struct sfc_flash *flash)
{
	unsigned char command;
	unsigned char sent_data,len;
	int dummy_byte,ret;

	struct sfc_transfer transfer[3];
	struct sfc_message message;
	struct cmd_info cmd[3];

	if(flash->flash_info->quad_mode == NULL){
		printk("quad info is null, use standard spi mode\n");
		sfc_transfer_mode = TM_STD_SPI;
		return -1;
	}

	command = flash->flash_info->quad_mode->WRSR_CMD;
	sent_data = flash->flash_info->quad_mode->WRSR_DATE;
	len = flash->flash_info->quad_mode->WD_DATE_SIZE;
	dummy_byte = 0;

	sfc_message_init(&message);
	memset(&transfer, 0, sizeof(transfer));
	memset(&cmd, 0, sizeof(cmd));

	/* write enable */
	cmd[0].cmd = CMD_WREN;
	cmd[0].dataen = DISABLE;

	transfer[0].cmd_info = &cmd[0];
	transfer[0].sfc_mode = TM_STD_SPI;
	sfc_message_add_tail(&transfer[0], &message);

	/* write ops */
	cmd[1].cmd = command;
	cmd[1].dummy_byte = dummy_byte;
	cmd[1].dataen = ENABLE;

	transfer[1].len = len;
	transfer[1].data = &sent_data;
	transfer[1].ops_mode = DMA_OPS;
	transfer[1].sfc_mode = TM_STD_SPI;
	transfer[1].direction = GLB_TRAN_DIR_WRITE;
	transfer[1].cmd_info = &cmd[1];
	sfc_message_add_tail(&transfer[1], &message);

	cmd[2].cmd = flash->flash_info->quad_mode->RDSR_CMD;
	cmd[2].dataen = DISABLE;
	cmd[2].dummy_byte = 0;
	cmd[2].sta_exp = 0x2;
	cmd[2].sta_msk = 0x2;

	transfer[2].cmd_info = &cmd[2];
	sfc_message_add_tail(&transfer[2], &message);

	ret = sfc_sync(flash->sfc, &message);
	if(ret) {
		dev_err(flash->dev,"sfc_sync error ! %s %s %d\n",__FILE__,__func__,__LINE__);
		ret=-EIO;
	}

	sfc_transfer_mode = TM_QI_QO_SPI;
	return 0;
}
#endif
static int sfc_write(struct sfc_flash *flash,loff_t to,size_t len, const unsigned char *buf)
{
	unsigned char command;
	int dummy_byte = 0;

#ifdef CONFIG_SPI_QUAD
	if(sfc_transfer_mode == TM_QI_QO_SPI){
		command = SPINOR_OP_QPP;
	}
	else{
		command = SPINOR_OP_PP;
	}
#else
	command = SPINOR_OP_PP;
#endif


	sfc_do_write(flash,command,to,flash->flash_info->addrsize,buf,len,dummy_byte);
	return len;
}

static int sfc_read(struct sfc_flash *flash, loff_t from, size_t len, unsigned char *buf)
{

	unsigned char command;
	int dummy_byte;
	unsigned int ret;


#ifdef CONFIG_SPI_QUAD
	if(sfc_transfer_mode == TM_QI_QO_SPI){
		command = flash->flash_info->quad_mode->cmd_read;
		dummy_byte = flash->flash_info->quad_mode->dummy_byte;
	}
	else{
		command = SPINOR_OP_READ;
		dummy_byte = 0;
	}
#else
	command = SPINOR_OP_READ;
	dummy_byte = 0;
#endif


	ret = sfc_do_read(flash, command, from, flash->flash_info->addrsize, buf, len, dummy_byte);

	return ret;
}


static int jz_spi_norflash_read(struct mtd_info *mtd, loff_t from, size_t len,size_t *retlen, unsigned char *buf)
{
	struct sfc_flash *flash;
	flash = to_jz_spi_norflash(mtd);


	mutex_lock(&flash->lock);
	*retlen = sfc_read(flash, from, len, buf);
	mutex_unlock(&flash->lock);

	return 0;
}
static int jz_spi_norflash_read_params(struct sfc_flash *flash, loff_t from, size_t len, unsigned char *buf)
{
	struct sfc_transfer transfer;
	struct sfc_message message;
	struct cmd_info cmd;
	unsigned char command;
	int dummy_byte = 0,ret;


	command = SPINOR_OP_READ;
	mutex_lock(&flash->lock);

	sfc_message_init(&message);
	memset(&transfer, 0, sizeof(transfer));
	memset(&cmd, 0, sizeof(cmd));

	cmd.cmd = command;
	cmd.addr_len = DEFAULT_ADDRSIZE;
	cmd.dummy_byte = dummy_byte;
	cmd.dataen = ENABLE;

	transfer.addr = (unsigned int)from;
	transfer.len = len;
	transfer.data = buf;
	transfer.ops_mode = DMA_OPS;
	transfer.direction = GLB_TRAN_DIR_READ;
	transfer.sfc_mode = TM_STD_SPI;
	transfer.cmd_info = &cmd;
	sfc_message_add_tail(&transfer, &message);

	ret = sfc_sync(flash->sfc, &message);
	if(ret) {
		dev_err(flash->dev,"sfc_sync error ! %s %s %d\n",__FILE__,__func__,__LINE__);
		ret=-EIO;
	}

	/*fix the cache line problem,when use jffs2 filesystem must be flush cache twice*/
	if(transfer.ops_mode == DMA_OPS)
		dma_cache_sync(NULL, (void *)buf,len, DMA_FROM_DEVICE);
	mutex_unlock(&flash->lock);
	return 0;
}

static int set_flash_addr_width_4byte(struct sfc_flash *flash,int on)
{
	struct sfc_transfer transfer;
	struct sfc_message message;
	struct cmd_info cmd;
	int ret;

	sfc_message_init(&message);
	memset(&transfer, 0, sizeof(transfer));
	memset(&cmd, 0, sizeof(cmd));

	cmd.cmd = CMD_EN4B;
	cmd.dummy_byte = 0;
	cmd.dataen = DISABLE;

	transfer.cmd_info = &cmd;
	sfc_message_add_tail(&transfer, &message);

	ret = sfc_sync(flash->sfc, &message);
	if(ret) {
		dev_err(flash->dev,"sfc_sync error ! %s %s %d\n",__FILE__,__func__,__LINE__);
		ret=-EIO;
	}
	return 0;
}
static int jz_spi_norflash_erase_sector(struct sfc_flash *flash, uint32_t addr)
{
	u8 command;
	struct sfc_transfer transfer[3];
	struct sfc_message message;
	struct cmd_info cmd[3];
	int ret;

	sfc_message_init(&message);
	memset(&transfer, 0, sizeof(transfer));
	memset(&cmd, 0, sizeof(cmd));

	/* write enable */
	cmd[0].cmd = CMD_WREN;
	cmd[0].dataen = DISABLE;

	transfer[0].sfc_mode = TM_STD_SPI;
	transfer[0].cmd_info = &cmd[0];
	sfc_message_add_tail(&transfer[0], &message);

	switch(flash->mtd.erasesize) {
		case 0x1000:
			command = SPINOR_OP_BE_4K;
			break;
		case 0x8000:
			command = SPINOR_OP_BE_32K;
			break;
		case 0x10000:
			command = SPINOR_OP_SE;
			break;
	}

	/* erase ops */
	cmd[1].cmd = command;
	cmd[1].addr_len = flash->flash_info->addrsize;
	cmd[1].dummy_byte = 0;
	cmd[1].dataen = DISABLE;

	transfer[1].addr = addr;
	transfer[1].sfc_mode = TM_STD_SPI;
	transfer[1].direction = GLB_TRAN_DIR_WRITE;
	transfer[1].cmd_info = &cmd[1];
	sfc_message_add_tail(&transfer[1], &message);

	cmd[2].cmd = CMD_RDSR;
	cmd[2].dataen = DISABLE;
	cmd[2].sta_exp = 0;
	cmd[2].sta_msk = 0x1;

	transfer[2].cmd_info = &cmd[2];
	sfc_message_add_tail(&transfer[2], &message);

	ret = sfc_sync(flash->sfc, &message);
	if(ret) {
		dev_err(flash->dev,"sfc_sync error ! %s %s %d\n",__FILE__,__func__,__LINE__);
		ret=-EIO;
	}

	return 0;
}
static int jz_spi_norflash_erase(struct mtd_info *mtd, struct erase_info *instr)
{
	int ret;
	uint32_t addr, end;
	struct sfc_flash *flash;

	flash = to_jz_spi_norflash(mtd);
	mutex_lock(&flash->lock);

	addr = (instr->addr & (mtd->erasesize - 1));
	if (addr) {
		dev_err(flash->dev,"%s eraseaddr no align\n", __func__);
		mutex_unlock(&flash->lock);
		return -EINVAL;
	}
	end = (instr->len & (mtd->erasesize - 1));
	if (end) {
		dev_err(flash->dev,"%s erasesize no align\n", __func__);
		mutex_unlock(&flash->lock);
		return -EINVAL;
	}
	addr = (uint32_t)instr->addr;
	end = addr + (uint32_t)instr->len;

	while (addr < end){
		ret = jz_spi_norflash_erase_sector(flash, addr);
		if (ret) {
			dev_err(flash->dev,"erase error !\n");
			mutex_unlock(&flash->lock);
			instr->state = MTD_ERASE_FAILED;
			return ret;
		}
		addr += mtd->erasesize;
	}
	mutex_unlock(&flash->lock);
	instr->state = MTD_ERASE_DONE;

	mtd_erase_callback(instr);
	return 0;
}
static int jz_spi_norflash_write(struct mtd_info *mtd, loff_t to, size_t len,
		size_t *retlen, const unsigned char *buf)
{
	u32 page_offset, actual_len;
	struct sfc_flash *flash;
	int ret;
	flash = to_jz_spi_norflash(mtd);

	mutex_lock(&flash->lock);

	page_offset = to & (flash->flash_info->pagesize - 1);
	/* do all the bytes fit onto one page? */
	if (page_offset + len <= flash->flash_info->pagesize) {
		ret = sfc_write(flash,to,len,buf);
		*retlen = ret;

	}else {
		u32 i;

		/* the size of data remaining on the first page */
		actual_len = flash->flash_info->pagesize - page_offset;
		ret = sfc_write(flash,to,actual_len,buf);
		*retlen += ret;

		/* write everything in flash->page_size chunks */
		for (i = actual_len; i < len; i += mtd->writesize) {
			actual_len = len - i;
			if (actual_len >= mtd->writesize)
				actual_len = mtd->writesize;

			ret = sfc_write(flash,to + i,actual_len,buf + i);
			*retlen += ret;
		}
	}
	mutex_unlock(&flash->lock);
	return 0;
}
static int jz_spi_norflash_match_device(struct sfc_flash *flash)
{

	struct nor_sharing_params *params;

	params = (struct nor_sharing_params*)kmalloc(sizeof(struct nor_sharing_params),GFP_KERNEL);
	if(!params){
		dev_err(flash->dev,"malloc params mem error !\n");
		return -ENOMEM;
	}
	jz_spi_norflash_read_params(flash,SPIFLASH_PARAMER_OFFSET,sizeof(struct nor_sharing_params),(unsigned char *)params);
	if(params->magic == NOR_MAGIC) {
		if(params->version == NOR_VERSION){
		int i;
		struct spi_nor_platform_data *flash_info;

		flash_info = (struct spi_nor_platform_data *)kzalloc(sizeof(struct spi_nor_platform_data),GFP_KERNEL);
		flash_info->mtd_partition = (struct mtd_partition *)kzalloc(sizeof(struct mtd_partition) * params->norflash_partitions.num_partition_info,GFP_KERNEL);
		if(!(flash_info) | !(flash_info->mtd_partition)){
			dev_err(flash->dev,"flash_info or mtd_partition malloc mem error !\n");
			return -ENOMEM;
		}
		flash_info->pagesize       = params->norflash_params.pagesize;
		flash_info->sectorsize     = params->norflash_params.sectorsize;
		flash_info->chipsize       = params->norflash_params.chipsize;
		flash_info->erasesize      = params->norflash_params.erasesize;
		flash_info->id             = params->norflash_params.id;

		flash_info->addrsize       = params->norflash_params.addrsize;
		flash_info->pp_maxbusy     = params->norflash_params.pp_maxbusy;
		flash_info->se_maxbusy     = params->norflash_params.se_maxbusy;
		flash_info->ce_maxbusy     = params->norflash_params.ce_maxbusy;
		flash_info->st_regnum      = params->norflash_params.st_regnum;

		for(i = 0; i < params->norflash_partitions.num_partition_info; i++){
			flash_info->mtd_partition[i].name = params->norflash_partitions.nor_partition[i].name;
			flash_info->mtd_partition[i].offset = params->norflash_partitions.nor_partition[i].offset;

			if(params->norflash_partitions.nor_partition[i].size == -1){
				flash_info->mtd_partition[i].size = MTDPART_SIZ_FULL;
			}else{
				flash_info->mtd_partition[i].size = params->norflash_partitions.nor_partition[i].size;
			}

			if(params->norflash_partitions.nor_partition[i].mask_flags & NORFLASH_PART_RO){
				flash_info->mtd_partition[i].mask_flags = MTD_CAP_NORFLASH;
			}

		}
		flash_info->num_partition_info = params->norflash_partitions.num_partition_info;
#ifdef CONFIG_SPI_QUAD
		flash_info->quad_mode = &(params->norflash_params.quad_mode);
#endif
		flash->flash_info = flash_info;

		}else{
			dev_err(flash->dev,"norflash version miss match,the current version is %d.%d.%d,but the burner version is %d.%d.%d\n"\
					,NOR_MAJOR_VERSION_NUMBER,NOR_MINOR_VERSION_NUMBER,NOR_REVERSION_NUMBER\
					,params->version & 0xff,(params->version & 0xff00) >> 8,(params->version & 0xff0000) >> 16);
			return -1;
		}
	}else{
		unsigned char command;
		int dummy_byte = 0;
		int addr_len = 0;
		int len = 3;
		int addr = 0;
		int id;
		int i;
		struct spi_nor_platform_data *flash_info;
		struct spi_board_info *binfo;


		command = SPINOR_OP_RDID;
		id = sfc_nor_read_id(flash, command, addr, addr_len, len, dummy_byte);
		id = ((id & 0xff) << 16) | (((id >> 8) & 0xff) << 8) | ((id >> 16) & 0xff);

		binfo = (struct spi_board_info *)flash->pdata->board_info;
		for (i = 0; i < flash->pdata->board_info_size; i++) {

			flash_info = (struct spi_nor_platform_data *)&binfo[i];

			if (flash_info->id == id){
				printk("the id code = %x, the flash name is %s\n",id,flash_info->name);
				break;
			}
		}

		if (i == flash->pdata->board_info_size) {
			if ((id != 0)&&(id != 0xff)&&(flash_info->quad_mode == 0)){
				flash_info = (struct spi_nor_platform_data *)&binfo[i];
				printk("the id code = %x, the flash name is %s\n",id,flash_info->name);
				printk("#####unsupport ID is %04x if the id not be 0x00,the flash can be ok,but the quad mode may be not support!!!!! \n",id);
			}else{
				printk("error happen !!!!,ingenic: Unsupported ID %04x,the quad mode is not support\n", id);
				return EINVAL;
			}
		}

		flash->flash_info = flash_info;


		kfree(params);
	}
	return 0;
}

static ssize_t sfc_nor_partition_offset_show(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	return sprintf(buf,"0x%x\n",SPI_NORFLASH_PART_OFFSET);
}

static DEVICE_ATTR(sfc_nor_partition_offset, S_IRUGO | S_IWUSR,
		sfc_nor_partition_offset_show,
		NULL);

static ssize_t sfc_nor_params_offset_show(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	return sprintf(buf,"0x%x\n",SPIFLASH_PARAMER_OFFSET);
}

static DEVICE_ATTR(sfc_nor_params_offset, S_IRUGO | S_IWUSR,
		sfc_nor_params_offset_show,
		NULL);

/*add your attr in here*/
static struct attribute *sfc_norflash_info_attributes[] = {
	&dev_attr_sfc_nor_partition_offset.attr,
	&dev_attr_sfc_nor_params_offset.attr,
	NULL
};

static const struct attribute_group sfc_norflash_info_attr_group = {
	.attrs = sfc_norflash_info_attributes
};


static int __init jz_sfc_probe(struct platform_device *pdev)
{
	struct sfc_flash *flash;
	struct mtd_partition *jz_mtd_partition;
	const char *jz_probe_types[] = {"cmdlinepart",NULL};
	int num_partition_info = 0;
	int err = 0,ret = 0;

	flash = kzalloc(sizeof(struct sfc_flash), GFP_KERNEL);
	if (!flash) {
		printk("ERROR: %s %d kzalloc() error !\n",__func__,__LINE__);
		return -ENOMEM;
	}

	flash->dev = &pdev->dev;

	flash->pdata = pdev->dev.platform_data;
	if (flash->pdata == NULL) {
		dev_err(&pdev->dev, "No platform data supplied\n");
		err = -ENOENT;
		goto err_no_pdata;
	}

	flash->sfc = sfc_res_init(pdev);

	platform_set_drvdata(pdev, flash);
	spin_lock_init(&flash->lock_status);
	mutex_init(&flash->lock);

	ret = jz_spi_norflash_match_device(flash);
	if (ret) {
		flash->flash_info = flash->pdata->board_info;
		flash->flash_info_num = flash->pdata->board_info_size;
		dev_err(&pdev->dev,"unknow id ,the id not match the spi bsp config,we will use the default config for the norflash\n");
		return -ENODEV;
	}

	flash->mtd.name     = "sfc_mtd";
	flash->mtd.owner    = THIS_MODULE;
	flash->mtd.type     = MTD_NORFLASH;
	flash->mtd.flags    = MTD_CAP_NORFLASH;
	flash->mtd.erasesize    = flash->flash_info->erasesize;
	flash->mtd.writesize    = flash->flash_info->pagesize;
	flash->mtd.size     = flash->flash_info->chipsize;
	flash->mtd._erase   = jz_spi_norflash_erase;
	flash->mtd._read    = jz_spi_norflash_read;
	flash->mtd._write   = jz_spi_norflash_write;

	/* enable 4-byte addressing if the device exceeds 16MiB */
	if(flash->mtd.size > 0x1000000)
		set_flash_addr_width_4byte(flash,1);

	set_flash_timing(flash);

#ifdef CONFIG_SPI_QUAD
	sfc_flash_set_quad_mode(flash);
#endif

#ifdef CONFIG_SPI_QUAD
	if(sfc_transfer_mode == TM_DI_DO_SPI)
		dev_info(&pdev->dev,"the flash->flash_info->quad_mode = %x\n",flash->flash_info->quad_mode->cmd_read);
#endif
	jz_mtd_partition = flash->flash_info->mtd_partition;
	num_partition_info = flash->flash_info->num_partition_info;

	ret = mtd_device_parse_register(&flash->mtd, jz_probe_types, NULL, jz_mtd_partition, num_partition_info);
	if (ret) {
		kfree(flash);
		dev_set_drvdata(flash->dev, NULL);
		return -ENODEV;
	}
	dev_info(&pdev->dev,"SPI NOR MTD LOAD OK\n");

	ret = sysfs_create_group(&pdev->dev.kobj, &sfc_norflash_info_attr_group);
	if(err){
		dev_err(&pdev->dev, "failed to register sysfs\n");
		sysfs_remove_group(&pdev->dev.kobj, &sfc_norflash_info_attr_group);
		return -EIO;
	}

	return 0;


err_no_pdata:
	release_resource(flash->sfc->ioarea);
	kfree(flash->sfc->ioarea);
	return err;
}

static int __exit jz_sfc_remove(struct platform_device *pdev)
{
	struct sfc_flash *flash = platform_get_drvdata(pdev);
	struct sfc *sfc = flash->sfc;


	clk_disable(sfc->clk_gate);
	clk_put(sfc->clk_gate);

	clk_disable(sfc->clk);
	clk_put(sfc->clk);

	free_irq(sfc->irq, flash);

	iounmap(sfc->iomem);

	release_mem_region(flash->resource->start, resource_size(flash->resource));

	platform_set_drvdata(pdev, NULL);

	sysfs_remove_group(&pdev->dev.kobj, &sfc_norflash_info_attr_group);
	return 0;
}

static int jz_sfc_suspend(struct platform_device *pdev, pm_message_t msg)
{
	unsigned long flags;
	struct sfc_flash *flash = platform_get_drvdata(pdev);
	struct sfc *sfc = flash->sfc;

	spin_lock_irqsave(&flash->lock_status, flags);
	flash->status |= STATUS_SUSPND;
	disable_irq(sfc->irq);
	spin_unlock_irqrestore(&flash->lock_status, flags);

	clk_disable(sfc->clk_gate);

	clk_disable(sfc->clk);

	return 0;
}

static int jz_sfc_resume(struct platform_device *pdev)
{
	unsigned long flags;
	struct sfc_flash *flash = platform_get_drvdata(pdev);
	struct sfc *sfc = flash->sfc;

	clk_enable(sfc->clk);

	clk_enable(sfc->clk_gate);

	spin_lock_irqsave(&flash->lock_status, flags);
	flash->status &= ~STATUS_SUSPND;
	enable_irq(sfc->irq);
	spin_unlock_irqrestore(&flash->lock_status, flags);

	return 0;
}

void jz_sfc_shutdown(struct platform_device *pdev)
{
	unsigned long flags;
	struct sfc_flash *flash = platform_get_drvdata(pdev);
	struct sfc *sfc = flash->sfc;

	spin_lock_irqsave(&flash->lock_status, flags);
	flash->status |= STATUS_SUSPND;
	disable_irq(sfc->irq);
	spin_unlock_irqrestore(&flash->lock_status, flags);

	clk_disable(sfc->clk_gate);

	clk_disable(sfc->clk);

	return ;
}

static struct platform_driver jz_sfcdrv = {
	.driver		= {
		.name	= "jz-sfc",
		.owner	= THIS_MODULE,
	},
	.remove		= jz_sfc_remove,
	.suspend	= jz_sfc_suspend,
	.resume		= jz_sfc_resume,
//	.shutdown	= jz_sfc_shutdown,
};

static int __init jz_sfc_init(void)
{
	return platform_driver_probe(&jz_sfcdrv, jz_sfc_probe);
}

static void __exit jz_sfc_exit(void)
{
    platform_driver_unregister(&jz_sfcdrv);
}

module_init(jz_sfc_init);
module_exit(jz_sfc_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("JZ SFC Driver");
