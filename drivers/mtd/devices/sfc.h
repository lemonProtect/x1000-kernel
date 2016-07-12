#ifndef __LINUX_SFC_H
#define __LINUX_SFC_H

#include "jz_sfc.h"


#define THRESHOLD		32

void sfc_start(struct sfc *sfc);
void sfc_flush_fifo(struct sfc *sfc);
void sfc_mode(struct sfc *sfc, int channel, int value);
void sfc_set_addr_length(struct sfc *sfc, int channel, unsigned int value);
void sfc_cmd_enble(struct sfc *sfc, int channel, unsigned int value);
void sfc_write_cmd(struct sfc *sfc, int channel, unsigned int value);
void sfc_dev_addr_dummy_bytes(struct sfc *sfc, int channel, unsigned int	value);
void sfc_dev_pollen(struct sfc *sfc, int channel, unsigned int value);
void sfc_dev_sta_exp(struct sfc *sfc, unsigned int value);
void sfc_dev_sta_msk(struct sfc *sfc, unsigned int value);
void sfc_clear_all_intc(struct sfc *sfc);
void sfc_enable_all_intc(struct sfc *sfc);
void sfc_set_length(struct sfc *sfc, int value);


void dump_sfc_reg(struct sfc *sfc);

void sfc_message_init(struct sfc_message *m);
void sfc_message_add_tail(struct sfc_transfer *t, struct sfc_message *m);
void sfc_transfer_del(struct sfc_transfer *t);
int sfc_sync(struct sfc *sfc, struct sfc_message *message);
int sfc_sync1(struct sfc *sfc, struct sfc_message *message);
struct sfc *sfc_res_init(struct platform_device *pdev);
#endif
