#ifndef __LINUX_SFC_JZ_H
#define __LINUX_SFC_JZ_H

#include <mach/jzssi.h>


#define UNCACHE(addr)   ((typeof(addr))(((unsigned long)(addr)) | 0xa0000000))
/* Flash opcodes. */
#define SPINOR_OP_WREN		0x06	/* Write enable */
#define SPINOR_OP_RDSR		0x05	/* Read status register */
#define SPINOR_OP_RDSR_1	0x35	/* Read status1 register */
#define SPINOR_OP_RDSR_2	0x15	/* Read status2 register */
#define SPINOR_OP_WRSR		0x01	/* Write status register 1 byte */
#define SPINOR_OP_WRSR_1	0x31	/* Write status1 register 1 byte */
#define SPINOR_OP_WRSR_2	0x11	/* Write status2 register 1 byte */
#define SPINOR_OP_READ		0x03	/* Read data bytes (low frequency) */
#define SPINOR_OP_READ_FAST	0x0b	/* Read data bytes (high frequency) */
#define SPINOR_OP_READ_1_1_2	0x3b	/* Read data bytes (Dual SPI) */
#define SPINOR_OP_READ_1_1_4	0x6b	/* Read data bytes (Quad SPI) */
#define SPINOR_OP_PP		0x02	/* Page program (up to 256 bytes) */
#define SPINOR_OP_QPP		0x32	/* Page program (up to 256 bytes) */
#define SPINOR_OP_BE_4K		0x20	/* Erase 4KiB block */
#define SPINOR_OP_BE_4K_PMC	0xd7	/* Erase 4KiB block on PMC chips */
#define SPINOR_OP_BE_32K	0x52	/* Erase 32KiB block */
#define SPINOR_OP_CHIP_ERASE	0xc7	/* Erase whole flash chip */
#define SPINOR_OP_SE		0xd8	/* Sector erase (usually 64KiB) */
#define SPINOR_OP_RDID		0x9f	/* Read JEDEC ID */
#define SPINOR_OP_RDCR		0x35	/* Read configuration register */
#define SPINOR_OP_RDFSR		0x70	/* Read flag status register */

/* 4-byte address opcodes - used on Spansion and some Macronix flashes. */
#define SPINOR_OP_READ4		0x13	/* Read data bytes (low frequency) */
#define SPINOR_OP_READ4_FAST	0x0c	/* Read data bytes (high frequency) */
#define SPINOR_OP_READ4_1_1_2	0x3c	/* Read data bytes (Dual SPI) */
#define SPINOR_OP_READ4_1_1_4	0x6c	/* Read data bytes (Quad SPI) */
#define SPINOR_OP_PP_4B		0x12	/* Page program (up to 256 bytes) */
#define SPINOR_OP_SE_4B		0xdc	/* Sector erase (usually 64KiB) */

/* Used for SST flashes only. */
#define SPINOR_OP_BP		0x02	/* Byte program */
#define SPINOR_OP_WRDI		0x04	/* Write disable */
#define SPINOR_OP_AAI_WP	0xad	/* Auto address increment word program */

/* Used for Macronix and Winbond flashes. */
#define SPINOR_OP_EN4B		0xb7	/* Enter 4-byte mode */
#define SPINOR_OP_EX4B		0xe9	/* Exit 4-byte mode */

/* Used for Spansion flashes only. */
#define SPINOR_OP_BRWR		0x17	/* Bank register write */

/* Status Register bits. */
#define SR_WIP			1	/* Write in progress */
#define SR_WEL			2	/* Write enable latch */
#define SR_SQE			(1 << 1)	/* QUAD MODE enable */
/* meaning of other SR_* bits may differ between vendors */
#define SR_BP0			4	/* Block protect 0 */
#define SR_BP1			8	/* Block protect 1 */
#define SR_BP2			0x10	/* Block protect 2 */
#define SR_SRWD			0x80	/* SR write protect */

#define SR_QUAD_EN_MX		0x40	/* Macronix Quad I/O */

/* Flag Status Register bits */
#define FSR_READY		0x80

/* Configuration Register bits. */
#define CR_QUAD_EN_SPAN		0x2	/* Spansion Quad I/O */



#define BUFFER_SIZE PAGE_SIZE

/* SFC register */

#define	SFC_GLB				(0x0000)
#define	SFC_DEV_CONF			(0x0004)
#define	SFC_DEV_STA_EXP			(0x0008)
#define	SFC_DEV_STA_RT			(0x000c)
#define	SFC_DEV_STA_MSK			(0x0010)
#define	SFC_TRAN_CONF(n)		(0x0014 + (n * 4))
#define	SFC_TRAN_LEN			(0x002c)
#define	SFC_DEV_ADDR(n)			(0x0030 + (n * 4))
#define	SFC_DEV_ADDR_PLUS(n)		(0x0048 + (n * 4))
#define	SFC_MEM_ADDR			(0x0060)
#define	SFC_TRIG			(0x0064)
#define	SFC_SR				(0x0068)
#define	SFC_SCR				(0x006c)
#define	SFC_INTC			(0x0070)
#define	SFC_FSM				(0x0074)
#define	SFC_CGE				(0x0078)
#define	SFC_RM_DR			(0x1000)

/* For SFC_GLB */
#define	GLB_TRAN_DIR			(1 << 13)
#define GLB_TRAN_DIR_WRITE		(1)
#define GLB_TRAN_DIR_READ		(0)
#define	GLB_THRESHOLD_OFFSET		(7)
#define GLB_THRESHOLD_MSK		(0x3f << GLB_THRESHOLD_OFFSET)
#define GLB_OP_MODE			(1 << 6)
#define SLAVE_MODE			(0x0)
#define DMA_MODE			(0x1)
#define GLB_PHASE_NUM_OFFSET		(3)
#define GLB_PHASE_NUM_MSK		(0x7  << GLB_PHASE_NUM_OFFSET)
#define GLB_WP_EN			(1 << 2)
#define GLB_BURST_MD_OFFSET		(0)
#define GLB_BURST_MD_MSK		(0x3  << GLB_BURST_MD_OFFSET)

/* For SFC_DEV_CONF */
#define	DEV_CONF_ONE_AND_HALF_CYCLE_DELAY	(3)
#define	DEV_CONF_ONE_CYCLE_DELAY	(2)
#define	DEV_CONF_HALF_CYCLE_DELAY	(1)
#define	DEV_CONF_NO_DELAY	        (0)
#define	DEV_CONF_SMP_DELAY_OFFSET	(16)
#define	DEV_CONF_SMP_DELAY_MSK		(0x3 << DEV_CONF_SMP_DELAY_OFFSET)
#define DEV_CONF_CMD_TYPE		(0x1 << 15)
#define DEV_CONF_STA_TYPE_OFFSET	(13)
#define DEV_CONF_STA_TYPE_MSK		(0x1 << DEV_CONF_STA_TYPE_OFFSET)
#define DEV_CONF_THOLD_OFFSET		(11)
#define	DEV_CONF_THOLD_MSK		(0x3 << DEV_CONF_THOLD_OFFSET)
#define DEV_CONF_TSETUP_OFFSET		(9)
#define DEV_CONF_TSETUP_MSK		(0x3 << DEV_CONF_TSETUP_OFFSET)
#define DEV_CONF_TSH_OFFSET		(5)
#define DEV_CONF_TSH_MSK		(0xf << DEV_CONF_TSH_OFFSET)
#define DEV_CONF_CPHA			(0x1 << 4)
#define DEV_CONF_CPOL			(0x1 << 3)
#define DEV_CONF_CEDL			(0x1 << 2)
#define DEV_CONF_HOLDDL			(0x1 << 1)
#define DEV_CONF_WPDL			(0x1 << 0)

/* For SFC_TRAN_CONF */
#define	TRAN_CONF_TRAN_MODE_OFFSET	(29)
#define	TRAN_CONF_TRAN_MODE_MSK		(0x7)
#define	TRAN_CONF_ADDR_WIDTH_OFFSET	(26)
#define	TRAN_CONF_ADDR_WIDTH_MSK	(0x7 << ADDR_WIDTH_OFFSET)
#define TRAN_CONF_POLLEN		(1 << 25)
#define TRAN_CONF_CMDEN			(1 << 24)
#define TRAN_CONF_FMAT			(1 << 23)
#define TRAN_CONF_DMYBITS_OFFSET	(17)
#define TRAN_CONF_DMYBITS_MSK		(0x3f << DMYBITS_OFFSET)
#define TRAN_CONF_DATEEN		(1 << 16)
#define	TRAN_CONF_CMD_OFFSET		(0)
#define	TRAN_CONF_CMD_MSK		(0xffff << CMD_OFFSET)

/* For SFC_TRIG */
#define TRIG_FLUSH			(1 << 2)
#define TRIG_STOP			(1 << 1)
#define TRIG_START			(1 << 0)

/* For SFC_SCR */
#define	CLR_END			(1 << 4)
#define CLR_TREQ		(1 << 3)
#define CLR_RREQ		(1 << 2)
#define CLR_OVER		(1 << 1)
#define CLR_UNDER		(1 << 0)

/* For SFC_TRAN_CONFx */
#define	TRAN_MODE_OFFSET	(29)
#define	TRAN_MODE_MSK		(0x7 << TRAN_MODE_OFFSET)
#define TRAN_SPI_STANDARD   (0x0)
#define TRAN_SPI_DUAL   (0x1 )
#define TRAN_SPI_QUAD   (0x5 )
#define TRAN_SPI_IO_QUAD   (0x6 )


#define	ADDR_WIDTH_OFFSET	(26)
#define	ADDR_WIDTH_MSK		(0x7 << ADDR_WIDTH_OFFSET)
#define POLLEN			(1 << 25)
#define CMDEN			(1 << 24)
#define FMAT			(1 << 23)
#define DMYBITS_OFFSET		(17)
#define DMYBITS_MSK		(0x3f << DMYBITS_OFFSET)
#define DATEEN			(1 << 16)
#define	CMD_OFFSET		(0)
#define	CMD_MSK			(0xffff << CMD_OFFSET)

#define N_MAX				6
#define MAX_SEGS        128

#define CHANNEL_0		0
#define CHANNEL_1		1
#define CHANNEL_2		2
#define CHANNEL_3		3
#define CHANNEL_4		4
#define CHANNEL_5		5

#define ENABLE			1
#define DISABLE			0

#define COM_CMD			1	// common cmd
#define POLL_CMD		2	// the cmd will poll the status of flash,ext: read status

#define DMA_OPS			1
#define CPU_OPS			0

#define TM_STD_SPI		0
#define TM_DI_DO_SPI	1
#define TM_DIO_SPI		2
#define TM_FULL_DIO_SPI	3
#define TM_QI_QO_SPI	5
#define TM_QIO_SPI		6
#define	TM_FULL_QIO_SPI	7

#define DEFAULT_ADDRSIZE	3

struct cmd_info{
	int cmd_type; //1:common cmd, 2: poll cmd
	int cmd;
	int addr_len;
	unsigned int addr_plus;
	int dummy_byte;
	int dataen;
	int sta_exp;
	int sta_msk;
};

struct sfc_transfer {
	int direction;
	unsigned int addr;

	const unsigned char *data;
	unsigned int len;
	unsigned int tmp_len;

	int sfc_mode;
	int ops_mode;
	struct cmd_info *cmd_info;
	struct list_head transfer_list;
};

struct sfc_message {
	struct list_head    transfers;
	unsigned        actual_length;
	int         status;

};

struct sfc{

	void __iomem		*iomem;
	struct resource     *ioarea;
	int			irq;
	struct clk		*clk;
	struct clk		*clk_gate;
	unsigned long src_clk;
	struct completion	done;
	int			threshold;
	irqreturn_t (*irq_callback)(struct sfc *);
	unsigned long		phys;

	struct sfc_transfer *transfer;
};


struct sfc_flash {
	struct mtd_info     mtd;
	struct device		*dev;
	struct resource		*resource;
	struct sfc			*sfc;
	struct jz_sfc_info *pdata;
	struct spi_nor_platform_data *flash_info;
	unsigned int flash_info_num;

	struct mutex        lock;

	int			status;
	spinlock_t		lock_status;
};

#endif
