/* drivers/input/touchscreen/ite7258_ts.c
 *
 * FocalTech ite7258 TouchScreen driver.
 *
 * Copyright (c) 2010  Focal tech Ltd.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/input/mt.h>
//#include <linux/earlysuspend.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
//#include <mach/irqs.h>
#include <linux/kernel.h>
#include <linux/semaphore.h>
#include <linux/mutex.h>
#include <linux/module.h>
#include <linux/gpio.h>
#include <soc/gpio.h>
#include <linux/syscalls.h>
#include <linux/unistd.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/string.h>
#include <linux/timer.h>
#include <linux/regulator/consumer.h>
#include <linux/device.h>
#include <linux/i2c/ite7258_tsc.h>

#include "ite7258_ts.h"

#define DEBUG_LCD_VCC_ALWAYS_ON
#ifdef FTS_CTL_IIC
#include "focaltech_ctl.h"
#endif
#ifdef SYSFS_DEBUG
#include "ite7258_ex_fun.h"
#endif


#define		CONFIG_ITE7258_MULTITOUCH
struct ite7258_ts_data {
        unsigned int irq;
        unsigned int rst;
        unsigned int x_max;
        unsigned int y_max;
        unsigned int x_pos;
        unsigned int y_pos;
	unsigned int is_suspend;
        struct i2c_client *client;
        struct input_dev *input_dev;
        struct jztsc_platform_data *pdata;
    //    struct early_suspend early_suspend;
	struct mutex lock;
        struct work_struct  work;
        struct workqueue_struct *workqueue;
        char *vcc_name;
        struct regulator *vcc_reg;
};

struct ite7258_update_data{
        struct i2c_client *client;
        unsigned int fw_length;
        unsigned int conf_length;
        char *fw_buf;
        char *conf_buf;
};

static struct ite7258_update_data *update;

static const struct attribute_group it7258_attr_group;

/*
 *ite7258_i2c_Read-read data and write data by i2c
 *@client: handle of i2c
 *@writebuf: Data that will be written to the slave
 *@writelen: How many bytes to write
 *@readbuf: Where to store data read from slave
 *@readlen: How many bytes to read
 *Returns negative errno, else the number of messages executed
 */
int ite7258_i2c_Read(struct i2c_client *client, char *writebuf,
		int writelen, char *readbuf, int readlen)
{
	int ret;
	struct ite7258_ts_data *ts = i2c_get_clientdata(client);
	if(ts->is_suspend)
		return 0;
	if (writelen > 0) {
		struct i2c_msg msgs[] = {
			{
				.addr = client->addr,
				.flags = 0,
				.len = writelen,
				.buf = writebuf,
			},
			{
				.addr = client->addr,
				.flags = I2C_M_RD,
				.len = readlen,
				.buf = readbuf,
			},
		};
		ret = i2c_transfer(client->adapter, msgs, 2);
		if (ret < 0)
			dev_err(&client->dev, "f%s: i2c read error.\n",
					__func__);
	} else {
		struct i2c_msg msgs[] = {
			{
				.addr = client->addr,
				.flags = I2C_M_RD,
				.len = readlen,
				.buf = readbuf,
			},
		};
		ret = i2c_transfer(client->adapter, msgs, 1);
		if (ret < 0)
			dev_err(&client->dev, "%s:i2c read error.\n", __func__);
	}
	return ret;
}

/*write data by i2c*/
int ite7258_i2c_Write(struct i2c_client *client, char *writebuf, int writelen)
{
        int ret;

        struct i2c_msg msg[] = {
                {
                        .addr = client->addr,
                        .flags = 0,
                        .len = writelen,
                        .buf = writebuf,
                },
        };

        ret = i2c_transfer(client->adapter, msg, 1);
        if (ret < 0)
                dev_err(&client->dev, "%s i2c write error.\n", __func__);

        return ret;
}

static void  ite7258_wait_command_done(struct i2c_client *client)
{
        unsigned char wbuffer[2];
        unsigned char rbuffer[2];
        unsigned int count = 0;

        do{
                wbuffer[0] = QUERY_BUF_ADDR;
                rbuffer[0] = 0x00;
                ite7258_i2c_Read(client, wbuffer, 1, rbuffer, 1);
                count++;
                msleep(1);
        }while(rbuffer[0] & 0x01 && count < 500);
}

static bool ite7258_enter_update_mode(struct i2c_client *client)
{
        unsigned char cmd_data_buf[MAX_BUFFER_SIZE];
        char cmd_response[2] = {0xFF, 0xFF};

        ite7258_wait_command_done(client);

        cmd_data_buf[1] = 0x60;
        cmd_data_buf[2] = 0x00;
        cmd_data_buf[3] = 'I';
        cmd_data_buf[4] = 'T';
        cmd_data_buf[5] = '7';
        cmd_data_buf[6] = '2';
        cmd_data_buf[7] = '6';
        cmd_data_buf[8] = '0';
        cmd_data_buf[9] = 0x55;
        cmd_data_buf[10] = 0xAA;

        printk("before 1 tpd_i2c_write_2 %s, %d\n", __func__, __LINE__);

        cmd_data_buf[0] = 0x20;
        if(!ite7258_i2c_Write(client, cmd_data_buf, 11)){
                printk("XXX %s, %d\n", __func__, __LINE__);
                return false;
        }

        printk("after 1 tpd_i2c_write_2 %s, %d\n", __func__, __LINE__);
        ite7258_wait_command_done(client);
        cmd_data_buf[0] = 0xA0;
        if(!ite7258_i2c_Read(client, cmd_data_buf, 1, cmd_response, 2) ){
                printk("XXX %s, %d\n", __func__, __LINE__);
                return false;
        }

        if(cmd_response[0] | cmd_response[1] ){
                printk("XXX %s, %d\n", __func__, __LINE__);
                return false;
        }
        printk("OOO %s, %d\n", __func__, __LINE__);
        return true;
}

bool ite7258_exit_update_mode(struct i2c_client *client)
{
        char cmd_data_buf[MAX_BUFFER_SIZE];
        char cmd_response[2] = {0xFF, 0xFF};

        ite7258_wait_command_done(client);

        cmd_data_buf[0] = 0x20;
        cmd_data_buf[1] = 0x60;
        cmd_data_buf[2] = 0x80;
        cmd_data_buf[3] = 'I';
        cmd_data_buf[4] = 'T';
        cmd_data_buf[5] = '7';
        cmd_data_buf[6] = '2';
        cmd_data_buf[7] = '6';
        cmd_data_buf[8] = '0';
        cmd_data_buf[9] = 0xAA;
        cmd_data_buf[10] = 0x55;

        if(!ite7258_i2c_Write(client, cmd_data_buf, 11)){
                printk("XXX %s, %d\n", __func__, __LINE__);
                return false;
        }

        ite7258_wait_command_done(client);

        cmd_data_buf[0] = 0xA0;
        if(!ite7258_i2c_Read(client, cmd_data_buf, 1, cmd_response, 2)){
                printk("XXX %s, %d\n", __func__, __LINE__);
                return false;
        }

        if(cmd_response[0] | cmd_response[1]){
                printk("XXX %s, %d\n", __func__, __LINE__);
                return false;
        }
        printk("OOO %s, %d\n", __func__, __LINE__);
        return true;
}

static bool ite7258_firmware_reinit(struct i2c_client *client)
{
        u8 cmd_data_buf[2];
        ite7258_wait_command_done(client);

        cmd_data_buf[0] = 0x20;
        cmd_data_buf[0] = 0x6F;
        if(!ite7258_i2c_Write(client, cmd_data_buf, 2) ){
                printk("XXX %s, %d\n", __func__, __LINE__);
                return false;
        }
        printk("OOO %s, %d\n", __func__, __LINE__);
        return true;
}

static bool ite7258_setupdate_offset(struct i2c_client *client,
                unsigned short offset)
{
        u8 command_buf[MAX_BUFFER_SIZE];
        char command_respon_buf[2] = {0xFF, 0xFF};

        ite7258_wait_command_done(client);

        command_buf[0] = 0x20;
        command_buf[1] = 0x61;
        command_buf[2] = 0;
        command_buf[3] = (offset & 0x00FF);
        command_buf[4] = ((offset & 0xFF00) >> 8);

        if(!ite7258_i2c_Write(client, command_buf, 5)){
                printk("XXX %s, %d\n", __func__, __LINE__);
                return false;
        }

        ite7258_wait_command_done(client);

        command_buf[0] = 0xA0;
        if(ite7258_i2c_Read(client, command_buf, 1, command_respon_buf, 2)){
                printk("XXX %s, %d\n", __func__, __LINE__);
                return false;
        }

        if(command_respon_buf[0] | command_respon_buf[1]){
                printk("XXX %s, %d\n", __func__, __LINE__);
                return false;
        }

        return true;
}

static bool ite7258_really_update(struct i2c_client *client,
                unsigned int length, char *date, unsigned short offset)
{
        unsigned int index = 0;
        unsigned char buffer[130] = {0};
        unsigned char buf_write[130] = {0};
        unsigned char buf_read[130] = {0};
        unsigned char read_length;
        int retry_count;
        int i;

        read_length = 128;
        while(index < length){
                retry_count = 0;
                do{
                        ite7258_setupdate_offset(client, offset + index);
                        buffer[0] = 0x20;
                        buffer[1] = 0x62;
                        buffer[2] = 128;
                        for (i = 0; i < 129; i++) {
                                buf_write[i] = buffer[3 + i] = date[index + i];
                        }
                        ite7258_i2c_Write(client, buffer, 132);

                        // Read from Flash
                        buffer[0] = 0x20;
                        buffer[1] = 0x63;
                        buffer[2] = read_length;

                        ite7258_setupdate_offset(client, offset + index);
                        ite7258_i2c_Write(client, buffer, 3);
                        ite7258_wait_command_done(client);

                        buffer[0] = 0xA0;
                        ite7258_i2c_Read(client, buffer, 1, buf_read, read_length);
                        // Compare
                        for (i = 0; i < 128; i++) {
                                if (buf_read[i] != buf_write[i]) {
                                        break;
                                }
                        }
                        if (i == 128) break;
                }while (retry_count++ < 4);

                if (retry_count == 4 && i != 128){
                        printk("XXX %s, %d\n", __func__, __LINE__);
                        return false;
                }
                index += 128;
        }
        return true;
}

static bool ite7258_firmware_down(void)
{
        if((update->fw_length == 0 || update->fw_buf == NULL) && \
                        (update->conf_length == 0 || update->conf_buf == NULL)){
                printk("XXX %s, %d\n", __func__, __LINE__);
                return false;
        }
        printk("ite7258_firmware_down %s, %d\n", __func__, __LINE__);

        if(!ite7258_enter_update_mode(update->client)){
                printk("XXX %s, %d\n", __func__, __LINE__);
                return false;
        }
        printk("ite7258_enter_update_mode %s, %d\n", __func__, __LINE__);

        if(update->fw_length != 0 && update->fw_buf != NULL){
                // Download firmware
                if(!ite7258_really_update(update->client, update->fw_length,
                                        update->fw_buf, 0)){
                        printk("XXX %s, %d\n", __func__, __LINE__);
                        return false;
                }
        }
        printk("write_and_compare_flash Fireware %s, %d\n", __func__, __LINE__);

        if(update->conf_length != 0 && update->conf_buf != NULL){
                // Download configuration
                unsigned short wFlashSize = 0x8000;
                if(!ite7258_really_update(update->client, update->conf_length, \
                                update->conf_buf, wFlashSize - \
                                (unsigned short)update->conf_length)){
                        printk("XXX %s, %d\n", __func__, __LINE__);
                        return false;
                }
        }
        printk("write_and_compare_flash Config %s, %d\n", __func__, __LINE__);

        if(!ite7258_exit_update_mode(update->client)){
                printk("XXX %s, %d\n", __func__, __LINE__);
                return false;
        }
        printk("ite7258_exit_update_mode %s, %d\n", __func__, __LINE__);

        if(!ite7258_firmware_reinit(update->client)){
                printk("XXX %s, %d\n", __func__, __LINE__);
                return false;
        }
        printk("OOO %s, %d\n", __func__, __LINE__);
        return true;
}

static int ite7258_upgrade_fw(void)
{
        unsigned int fw_size = 0;
        unsigned int config_size = 0;
        struct file *fw_fd = NULL;
        struct file *config_fd = NULL;

        mm_segment_t fs;
        u8 *fw_buf = kzalloc(0x8000, GFP_KERNEL);
        u8 *config_buf = kzalloc(0x500, GFP_KERNEL);
        if(fw_buf  == NULL || config_buf == NULL){
                printk("kzalloc failed\n");
        }

        fs = get_fs();
        set_fs(get_ds());

        fw_fd = filp_open("/data/upgrade/it7260_FW", O_RDONLY, 0);
        if (fw_fd < 0){
                printk("open /data/upgrade/it7260_FW failed \n");
                fw_fd = filp_open("/sdcard/upgrade/it7260_FW", O_RDONLY, 0);
                if (fw_fd < 0){
                        printk("open /sdcard/upgrade/it7260_FW failed \n");
                        filp_close(fw_fd,NULL);
                        return 1;
                }
        }

        fw_size = fw_fd->f_op->read(fw_fd, fw_buf, 0x8000, &fw_fd->f_pos);
        printk("fw_ver : %d,%d,%d,%d\n",fw_buf[8], fw_buf[9], fw_buf[10], fw_buf[11]);
        printk("--------------------- fw_size = %x\n", fw_size);

        config_fd = filp_open("/data/upgrade/it7260_Config", O_RDONLY, 0);
        if(config_fd < 0){
                printk("open /data/upgrade/it7260_Config failed \n");
                config_fd = filp_open("/sdcard/upgrade/it7260_Config", O_RDONLY, 0);
                if (config_fd < 0){
                        printk("open /sdcard/upgrade/it7260_Config failed \n");
                        filp_close(config_fd,NULL);
                        return 1;
                }
        }
        update->fw_length = fw_size;
        update->fw_buf = fw_buf;

        config_size = config_fd->f_op->read(config_fd, \
                        config_buf, 0x500, &config_fd->f_pos);
        printk("cfg_ver : %d,%d,%d,%d\n",config_buf[config_size-8],
                                        config_buf[config_size-7],
                                        config_buf[config_size-6],
                                        config_buf[config_size-5]);
        printk("--------------------- config_size = %x\n", config_size);
        update->conf_length = config_size - 8;
        update->conf_buf    = &config_buf[config_size-7];

        set_fs(fs);
        filp_close(fw_fd,NULL);
        filp_close(config_fd,NULL);
        //tpd_halt = 1;
        //mt65xx_eint_mask(CUST_EINT_TOUCH_PANEL_NUM);
        if (ite7258_firmware_down() == false){
                //fail
                //tpd_halt = 0;
                //mt65xx_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);
                return 1;
        }else{
                //succese;
                //tpd_halt = 0;
                //mt65xx_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);
                return 0;
        }

}

static int IT7258_upgrade_store_temp(void)
{
        if(!ite7258_upgrade_fw()) {
                printk("IT7258_upgrade_OK\n\n");
                return 0;
        } else {
                printk("IT7258_upgrade_failed\n");
                return -1;
        }
}

static ssize_t IT7258_upgrade_store(struct device *dev,
                struct device_attribute *attr, const char *buf, size_t count)
{
        printk("%s():\n", __func__);
        IT7258_upgrade_store_temp();
        return count;
}

static void ite7258_report_value(struct ite7258_ts_data *data)
{
#ifdef  CONFIG_ITE7258_MULTITOUCH
	input_report_abs(data->input_dev, ABS_MT_POSITION_X, data->x_pos);
	input_report_abs(data->input_dev, ABS_MT_POSITION_Y, data->y_pos);
	input_report_abs(data->input_dev, ABS_MT_TOUCH_MAJOR, 128);
	input_report_abs(data->input_dev, ABS_MT_WIDTH_MAJOR, 128);
#else
	input_report_abs(data->input_dev, ABS_X, data->x_pos);
	input_report_abs(data->input_dev, ABS_Y, data->y_pos);
	//input_report_abs(data->input_dev, ABS_PRESSURE, 0xF);
#endif
}

static int ite7258_read_Touchdata(struct ite7258_ts_data *data)
{
        int ret = -1;
	int flag = 1;
	int xraw, yraw;

	unsigned char pucPoint[14];
	while(flag){
		pucPoint[0] = QUERY_BUF_ADDR; //reg addr
		ret = ite7258_i2c_Read(data->client, pucPoint, 1, pucPoint, 1); //from addr 0x80
		if(!( pucPoint[0] & 0x80 || pucPoint[0] & 0x01 )){
			msleep(10);
			if(data->is_suspend)
			{
				return 0;
			}
			//                printk("-------------------test\n");
			//return 0;
			continue;
		}
		flag = 0;
		pucPoint[0] = POINT_INFO_BUF_ADDR;
		ret = ite7258_i2c_Read(data->client, pucPoint, 1, pucPoint, 14); //from addr 0xE0

#ifdef  CONFIG_ITE7258_MULTITOUCH
		if(pucPoint[0] & 0x01){
			xraw = ((pucPoint[3] & 0x0F) << 8) + pucPoint[2];
			yraw = ((pucPoint[3] & 0xF0) << 4) + pucPoint[4];
			            //  printk("[mtk-tpd] input Read_Point1 x=%d y=%d\n",xraw,yraw);
			data->x_pos = xraw;
			data->y_pos = yraw;
			ite7258_report_value(data);
			//		input_mt_sync(data->input_dev);
			//      	input_sync(data->input_dev);
		}
#if 1
		if(pucPoint[0] & 0x02){
			xraw = ((pucPoint[7] & 0x0F) << 8) + pucPoint[6];
			yraw = ((pucPoint[7] & 0xF0) << 4) + pucPoint[8];
			//printk("[mtk-tpd] input Read_Point2 x=%d y=%d\n",xraw,yraw);
			data->x_pos = xraw;
			data->y_pos = yraw;
			ite7258_report_value(data);
			//		input_mt_sync(data->input_dev);
			//      	input_sync(data->input_dev);
		}
		input_mt_sync(data->input_dev);
		input_sync(data->input_dev);
#endif
#else
		if(pucPoint[0] & 0x01){
			xraw = ((pucPoint[3] & 0x0F) << 8) + pucPoint[2];
			yraw = ((pucPoint[3] & 0xF0) << 4) + pucPoint[4];
			//printk("[mtk-tpd] input Read_Point1 x=%d y=%d\n",xraw,yraw);
			data->x_pos = xraw;
			data->y_pos = yraw;
			input_report_key(data->input_dev, BTN_TOUCH, 1);
			ite7258_report_value(data);
			input_sync(data->input_dev);
		}
#endif
	}
	return 0;

}

static void ite7258_work_handler(struct work_struct *work)
{
        struct ite7258_ts_data *ite7258_ts = \
                container_of(work, struct ite7258_ts_data, work);
        int ret = 0;

        ret = ite7258_read_Touchdata(ite7258_ts);

        enable_irq(ite7258_ts->irq);
}

/*The ite7258 device will signal the host about TRIGGER_FALLING.
 *Processed when the interrupt is asserted.
 */
static irqreturn_t ite7258_ts_interrupt(int irq, void *dev_id)
{
        struct ite7258_ts_data *ite7258_ts = dev_id;
        disable_irq_nosync(ite7258_ts->irq);

#if 1
        if (ite7258_ts->is_suspend)
                return IRQ_HANDLED;
#endif

        if (!work_pending(&ite7258_ts->work)) {
                queue_work(ite7258_ts->workqueue, &ite7258_ts->work);
        } else {
                enable_irq(ite7258_ts->irq);
        }

        return IRQ_HANDLED;
}

static void ite7258_idle_mode(struct i2c_client *client)
{
        unsigned char wbuf[8] = {0xFF};
        int ret = 0;

        wbuf[0] = CMD_BUF_ADDR;
        wbuf[1] = 0x12;
        wbuf[2] = 0x00;
        wbuf[3] = 0xB8;
        wbuf[4] = 0x1B;
        wbuf[5] = 0x00;
        wbuf[6] = 0x00;
        ite7258_i2c_Write(client, wbuf, 7);
        ite7258_wait_command_done(client);

        wbuf[0] = CMD_BUF_ADDR;
        wbuf[1] = 0x11;
        wbuf[2] = 0x00;
        wbuf[3] = 0x01;
        ite7258_i2c_Write(client, wbuf, 4);
        ite7258_wait_command_done(client);

        ite7258_wait_command_done(client);
        wbuf[0] = CMD_BUF_ADDR;
        wbuf[1] = 0x04;
        wbuf[2] = 0x00;
        wbuf[3] = 0x01;
        ret = ite7258_i2c_Write(client, wbuf, 4);
}

#define	ITE7258_DEBUG
static int ite7258_print_version(struct i2c_client *client)
{
        unsigned char wbuffer[9];
        unsigned char rbuffer[9];

        int ret = -1;

#ifdef  ITE7258_DEBUG

        ite7258_wait_command_done(client);
        /* Firmware Information */
        wbuffer[0] = CMD_BUF_ADDR;
        wbuffer[1] = 0x01;
        wbuffer[2] = 0x00;
        ite7258_i2c_Write(client, wbuffer, 3);
        msleep(10);
        ite7258_wait_command_done(client);

        wbuffer[0] = CMD_RESPONSE_BUF_ADDR;
        memset(rbuffer, 0xFF,  8);
        ret = ite7258_i2c_Read(client, wbuffer, 1, rbuffer, 9);
        printk("ITE7258 Touch Panel FW Version:%d.%d.%d.%d\tExtension ROM Version:%d.%d.%d.%d\n",\
                               rbuffer[1], rbuffer[2], rbuffer[3], rbuffer[4], \
                               rbuffer[5], rbuffer[6], rbuffer[7], rbuffer[8]);

        ite7258_wait_command_done(client);
        /* Configuration Version */
        wbuffer[0] = CMD_BUF_ADDR;
        wbuffer[1] = 0x01;
        wbuffer[2] = 0x06;
        ite7258_i2c_Write(client, wbuffer, 3);
        msleep(10);
        ite7258_wait_command_done(client);
        memset(rbuffer, 0xFF,  8);
        wbuffer[0] = CMD_RESPONSE_BUF_ADDR;
        ret = ite7258_i2c_Read(client, wbuffer, 1, rbuffer, 7);
        printk("ITE7258 Touch Panel Configuration Version:%x.%x.%x.%x\n",
                        rbuffer[1], rbuffer[2], rbuffer[3], rbuffer[4]);

        /* IRQ status */
        ite7258_wait_command_done(client);
        wbuffer[0] = CMD_BUF_ADDR;
        wbuffer[1] = 0x01;
        wbuffer[2] = 0x04;
        ite7258_i2c_Write(client, wbuffer, 3);
        msleep(10);
        ite7258_wait_command_done(client);

        memset(rbuffer, 0xFF,  8);
        wbuffer[0] = CMD_RESPONSE_BUF_ADDR;
        ret = ite7258_i2c_Read(client, wbuffer, 1, rbuffer, 2);
        printk("[mtk-tpd] ITE7258 Touch irq %x, %x \n",rbuffer[0], rbuffer[1]);
#endif

        /* vendor ID && Device ID */
        ite7258_wait_command_done(client);
        wbuffer[0] = CMD_BUF_ADDR;
        wbuffer[1] = 0x0;
        wbuffer[2] = 0x0;
        ite7258_i2c_Write(client, wbuffer, 2);
        msleep(10);
        ite7258_wait_command_done(client);
        wbuffer[0] = CMD_RESPONSE_BUF_ADDR;
        memset(rbuffer, 0xFF,  8);
        ret = ite7258_i2c_Read(client, wbuffer, 1, rbuffer, 8);

/*	if(strncmp(rbuffer, "ITE7260", 7)){
		return -1;
	}
*/
	if(rbuffer[1] != 0x49 && rbuffer[2] != 0x54 && rbuffer[3] != 0x45){
		return -1;
	}

        printk("ITE7258 Touch Panel Firmware Version %c%c%c%c%c%c%c\n",
                        rbuffer[1], rbuffer[2], rbuffer[3], rbuffer[4],
                        rbuffer[5], rbuffer[6], rbuffer[7]);

	return 0;

}

#if defined(CONFIG_PM) && defined(CONFIG_HAS_EARLYSUSPEND)
static void ite7258_ts_suspend(struct early_suspend *handler)
{
       struct ite7258_ts_data *ts = container_of(handler, struct ite7258_ts_data,
                        early_suspend);

	//flush_work(&ts->work);
	printk("---------------TP suspend\n");
	mutex_lock(&ts->lock);
	flush_scheduled_work();
        disable_irq(ts->irq);
	ts->is_suspend = 1;
	regulator_disable(ts->vcc_reg);
	mutex_unlock(&ts->lock);
        dev_dbg(&ts->client->dev, "[FTS]ite7258 suspend\n");

}

static void ite7258_ts_resume(struct early_suspend *handler)
{
	printk("ite7258 resume ---------\n");
        struct ite7258_ts_data *ts = container_of(handler, struct ite7258_ts_data,
                        early_suspend);
	//mutex_lock(&ts->lock);
	flush_scheduled_work();
        gpio_direction_output(ts->rst, 1);
	msleep(2);
        gpio_direction_output(ts->rst, 0);
	gpio_direction_output(ts->client->irq, 0);
        msleep(10);
        regulator_enable(ts->vcc_reg);
        dev_dbg(&ts->client->dev, "[FTS]ite7258 resume.\n");
	msleep(2);
	gpio_direction_input(ts->client->irq);
	//ite7258_idle_mode(ts->client);
	ts->is_suspend = 0;

	//msleep(10);
        enable_irq(ts->irq);
	printk("ite7258 resume ---------\n");
	//mutex_unlock(&ts->lock);
	//ite7258_print_version(ts->client);
}
#endif

 #ifdef CONFIG_HAS_EARLYSUSPEND
static void ite7258_early_suspend(struct early_suspend *h)
{
	ite7258_ts_suspend(h);
}

static void ite7258_ts_late_resume(struct early_suspend *h)
{
//	printk("ite7258 late resume ----------\n");
	ite7258_ts_resume(h);
}
#endif

static void ite7258_interrupt_mode(struct i2c_client *client)
{
        unsigned char buf[12];
        unsigned char tmp[2];

        do{
                tmp[0] = QUERY_BUF_ADDR;
                buf[0] = 0xFF;
                ite7258_i2c_Read(client, tmp, 1, buf, 1);
        }while( buf[0] & 0x01 );

        buf[0] = CMD_BUF_ADDR;
        buf[1] = 0x02;
        buf[2] = 0x04;
        buf[3] = 0x01; //enable interrupt
        buf[4] = 0x11; //Falling edge trigger
        ite7258_i2c_Write(client, buf, 5);
        do{
                tmp[0] = QUERY_BUF_ADDR;
                buf[0] = 0xFF;
                ite7258_i2c_Read(client, tmp, 1, buf, 1);
        }while( buf[0] & 0x01 );
        buf[0] = CMD_RESPONSE_BUF_ADDR;
        ite7258_i2c_Read(client, buf, 1, buf, 2);
        printk("DDD_____ 0xA0 : %X, %X\n", buf[0], buf[1]);
}

static void ite7258_hw_init(struct i2c_client *client)
{
        ite7258_interrupt_mode(client);
}




static int ite7258_gpio_request(struct ite7258_ts_data *ts)
{
        int err = 0;

        err = gpio_request(ts->rst, "ite7258 reset");
        if (err < 0) {
                printk("touch: %s failed to set gpio reset.\n",
                                __func__);
                return err;
        }

        err = gpio_request(ts->irq,"ite7258 irq");
        if (err < 0) {
                printk("touch: %s failed to set gpio irq.\n",
                                __func__);
                gpio_free(ts->rst);
                return err;
        }
        gpio_direction_input(ts->irq);

        gpio_direction_output(ts->rst, 1);
        gpio_direction_output(ts->rst, 0);

        return 0;
}

static int ite7258_regulator_get(struct ite7258_ts_data *ite7258_ts)
{
        int err = 0;

        ite7258_ts->vcc_reg = regulator_get(NULL, ite7258_ts->vcc_name);
        if (IS_ERR(ite7258_ts->vcc_reg)) {
                printk("failed to get VCC regulator.");
                err = PTR_ERR(ite7258_ts->vcc_reg);
                return -EINVAL;
        }

        regulator_enable(ite7258_ts->vcc_reg);

        return 0;
}

static void ite7258_input_set(struct input_dev *input_dev, struct ite7258_ts_data *ts)
{
#ifdef CONFIG_ITE7258_MULTITOUCH
        set_bit(ABS_MT_TOUCH_MAJOR, input_dev->absbit);
        set_bit(ABS_MT_POSITION_X, input_dev->absbit);
        set_bit(ABS_MT_POSITION_Y, input_dev->absbit);
        set_bit(ABS_MT_WIDTH_MAJOR, input_dev->absbit);

        input_set_abs_params(input_dev, ABS_MT_POSITION_X, 0, ts->x_max, 0, 0);
        input_set_abs_params(input_dev, ABS_MT_POSITION_Y, 0, ts->y_max, 0, 0);
        input_set_abs_params(input_dev, ABS_MT_WIDTH_MAJOR, 0, PRESS_MAX, 0, 0);
        input_set_abs_params(input_dev, ABS_MT_TOUCH_MAJOR, 0, PRESS_MAX, 0, 0);
#else
        set_bit(ABS_X, input_dev->absbit);
        set_bit(ABS_Y, input_dev->absbit);

	//set_bit(ABS_PRESSURE, input_dev->absbit);
        set_bit(EV_SYN, input_dev->evbit);
        set_bit(BTN_TOUCH, input_dev->keybit);

        input_set_abs_params(input_dev, ABS_X, 0, ts->x_max, 0, 0);
        input_set_abs_params(input_dev, ABS_Y, 0, ts->y_max, 0, 0);
	input_set_abs_params(input_dev, ABS_PRESSURE, 0, 0xFF, 0, 0);
#endif
        set_bit(EV_KEY, input_dev->evbit);
        set_bit(EV_ABS, input_dev->evbit);
	set_bit(EV_SYN, input_dev->evbit);
	set_bit(INPUT_PROP_DIRECT, input_dev->propbit);

	input_dev->name = ITE7258_NAME;
	input_dev->id.bustype = BUS_I2C;
	input_dev->id.vendor = 0xDEAD;
	input_dev->id.product = 0xBEEF;
	input_dev->id.version = 10427;

}



static DEVICE_ATTR(upgrade, 0666, NULL, IT7258_upgrade_store);
static struct attribute *it7258_attributes[] = {
        &dev_attr_upgrade.attr,
        NULL
};

static const struct attribute_group it7258_attr_group = {
        .attrs = it7258_attributes,
};

static int ite7258_ts_probe(struct i2c_client *client,
                const struct i2c_device_id *id)
{
        struct jztsc_platform_data *pdata =
                (struct jztsc_platform_data *)client->dev.platform_data;
        struct ite7258_ts_data *ite7258_ts;
        struct input_dev *input_dev;
        int err = 0;

        if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
                err = -ENODEV;
                goto exit_check_functionality_failed;
        }

        ite7258_ts = kzalloc(sizeof(struct ite7258_ts_data), GFP_KERNEL);
        if (!ite7258_ts) {
                dev_err(&client->dev, "failed to allocate input driver data\n");
                err = -ENOMEM;
                goto exit_alloc_data_failed;
        }

        update = kzalloc(sizeof(struct ite7258_update_data), GFP_KERNEL);
        if(!update){
                dev_err(&client->dev, "failed to allocate driver data\n");
                err = -ENOMEM;
                goto exit_alloc_update_failed;
        }
        i2c_set_clientdata(client, ite7258_ts);
        ite7258_ts->irq = pdata->gpio[0].num;
        ite7258_ts->rst = pdata->gpio[1].num;
        client->irq = ite7258_ts->irq;
        ite7258_ts->client = client;
        ite7258_ts->pdata = pdata;
        ite7258_ts->x_max = pdata->x_max - 1;
        ite7258_ts->y_max = pdata->y_max - 1;
        ite7258_ts->vcc_name = pdata->vcc_name;

        err = ite7258_gpio_request(ite7258_ts);
        if(err){
                dev_err(&client->dev, "failed to request gpio\n");
                goto exit_gpio_failed;
        }

        ite7258_ts->irq = gpio_to_irq(pdata->gpio[0].num);
        err = ite7258_regulator_get(ite7258_ts);
        if(err){
                dev_err(&client->dev, "failed to get regulator\n");
                goto exit_regulator_failed;
        }

	if(ite7258_print_version(client)){
		printk("cmp OK!!!!!!!!!!!!!!--------------------------\n");
		err = -ENOMEM;
		goto exit_get_version;
	}

	input_dev = input_allocate_device();
        if (!input_dev) {
                err = -ENOMEM;
                dev_err(&client->dev, "failed to allocate input device\n");
                goto exit_input_dev_alloc_failed;
        }

        ite7258_ts->input_dev = input_dev;
        ite7258_input_set(input_dev, ite7258_ts);

        err = input_register_device(input_dev);
        if (err) {
                dev_err(&client->dev,
                                "ite7258_ts_probe: failed to register input device: %s\n",
                                dev_name(&client->dev));
                goto exit_input_register_device_failed;
        }

#ifdef CONFIG_HAS_EARLYSUSPEND
        ite7258_ts->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
        ite7258_ts->early_suspend.suspend = ite7258_early_suspend;
        ite7258_ts->early_suspend.resume  = ite7258_ts_late_resume;
        register_early_suspend(&ite7258_ts->early_suspend);
#endif

#ifdef SYSFS_DEBUG
        ite7258_create_sysfs(client);
#endif

#ifdef FTS_CTL_IIC
        if (ft_rw_iic_drv_init(client) < 0)
                dev_err(&client->dev, "%s:[FTS] create fts control iic driver failed\n",
                                __func__);
#endif
        msleep(50);



		mutex_init(&ite7258_ts->lock);
        ite7258_hw_init(client);
	    ite7258_idle_mode(client);

        INIT_WORK(&ite7258_ts->work, ite7258_work_handler);
        ite7258_ts->workqueue = create_singlethread_workqueue("ite7258_ts");
	printk("%s: ts data: %p\n", __func__, ite7258_ts);

#if 0
        /* Update interface */
        err = sysfs_create_group(&(client->dev.kobj), &it7258_attr_group);
        if(err){
                dev_err(&client->dev, "failed to register sysfs\n");
        }else{
                printk("create it7260 sysfs attr_group sucontinues\n");
        }
#endif
        err = request_irq(ite7258_ts->irq, ite7258_ts_interrupt,
                        pdata->irqflags, client->dev.driver->name,
                        ite7258_ts);
        if (err < 0) {
                dev_err(&client->dev, "ite7258_probe: request irq failed\n");
                goto exit_irq_request_failed;
        }

        return 0;


exit_irq_request_failed:
        cancel_work_sync(&ite7258_ts->work);
	input_unregister_device(input_dev);

exit_input_register_device_failed:
        input_free_device(input_dev);

exit_input_dev_alloc_failed:
exit_get_version:
        regulator_disable(ite7258_ts->vcc_reg);
        regulator_put(ite7258_ts->vcc_reg);

        i2c_set_clientdata(client, NULL);

exit_regulator_failed:
        gpio_free(ite7258_ts->rst);
        gpio_free(ite7258_ts->irq);
        kfree(update);
exit_alloc_update_failed:
exit_gpio_failed:
        kfree(ite7258_ts);

exit_alloc_data_failed:
exit_check_functionality_failed:
        return err;
}

static int ite7258_ts_remove(struct i2c_client *client)
{
        struct ite7258_ts_data *ite7258_ts;
        ite7258_ts = i2c_get_clientdata(client);
        input_unregister_device(ite7258_ts->input_dev);
        input_free_device(ite7258_ts->input_dev);
        gpio_free(ite7258_ts->rst);
        gpio_free(ite7258_ts->irq);

#ifdef SYSFS_DEBUG
        ite7258_release_sysfs(client);
#endif
#ifdef FTS_CTL_IIC
        ft_rw_iic_drv_exit();
#endif
        free_irq(ite7258_ts->irq, ite7258_ts);
        if (!IS_ERR(ite7258_ts->vcc_reg)) {
                regulator_disable(ite7258_ts->vcc_reg);
                regulator_put(ite7258_ts->vcc_reg);
        }

        kfree(ite7258_ts);

        i2c_set_clientdata(client, NULL);
        return 0;
}

#if defined(CONFIG_PM) && defined(CONFIG_HAS_EARLYSUSPEND)
static const struct dev_pm_ops ite7258_ts_pm_ops = {
	.suspend        = ite7258_ts_suspend,
	.resume		= ite7258_ts_resume,
};
#endif

static const struct i2c_device_id ite7258_ts_id[] = {
        {ITE7258_NAME, 0},
        {}
};

MODULE_DEVICE_TABLE(i2c, ite7258_ts_id);

static struct i2c_driver ite7258_ts_driver = {
        .probe = ite7258_ts_probe,
        .remove = ite7258_ts_remove,
        .id_table = ite7258_ts_id,
        .driver = {
                .name = ITE7258_NAME,
                .owner = THIS_MODULE,
#if defined(CONFIG_PM) && defined(CONFIG_HAS_EARLYSUSPEND)
		.pm	= &ite7258_ts_pm_ops,
#endif
        },
};

static int __init ite7258_ts_init(void)
{
        int ret;
        ret = i2c_add_driver(&ite7258_ts_driver);
        return ret;
}

static void __exit ite7258_ts_exit(void)
{
        i2c_del_driver(&ite7258_ts_driver);
}

module_init(ite7258_ts_init);
module_exit(ite7258_ts_exit);

MODULE_AUTHOR("<Rejion>");
MODULE_DESCRIPTION("FocalTech ite7258 TouchScreen driver");
MODULE_LICENSE("GPL");
