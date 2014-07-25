#ifndef _DDR_H_
#define _DDR_H_
#define DDRP_PIR_INIT		(1 << 0)
#define DDRP_PIR_DLLSRST	(1 << 1)
#define DDRP_PIR_DLLLOCK	(1 << 2)
#define DDRP_PIR_ZCAL   	(1 << 3)
#define DDRP_PIR_ITMSRST   	(1 << 4)
#define DDRP_PIR_DRAMRST   	(1 << 5)
#define DDRP_PIR_DRAMINT   	(1 << 6)
#define DDRP_PIR_QSTRN   	(1 << 7)
#define DDRP_PIR_EYETRN   	(1 << 8)
#define DDRP_PIR_DLLBYP   	(1 << 17)
#define DDRP_PIR_LOCKBYP   	(1 << 29)
#define DDRP_PGSR_IDONE		(1 << 0)
#define DDRP_PGSR_DLDONE	(1 << 1)
#define DDRP_PGSR_ZCDONE	(1 << 2)
#define DDRP_PGSR_DIDONE	(1 << 3)
#define DDRP_PGSR_DTDONE	(1 << 4)
#define DDRP_PGSR_DTERR 	(1 << 5)
#define DDRP_PGSR_DTIERR 	(1 << 6)
#define DDRP_PGSR_DFTEERR 	(1 << 7)


#define DDR_PHY_OFFSET	(-0x4e0000 + 0x1000)

#define DDRP_PIR	(DDR_PHY_OFFSET + 0x4) /* PHY Initialization Register */
#define DDRP_PGSR	(DDR_PHY_OFFSET + 0xc) /* PHY General Status Register*/
#define DDRP_DX0GSR     (DDR_PHY_OFFSET + 0x71 * 4)
#define DDRC_DLP			0xbc
#define DDRP_DSGCR	(DDR_PHY_OFFSET + 0x2c) /* DDR System General Configuration Register */

#define DDRC_STATUS			0x0
#define DDRC_CFG			0x4
#define DDRC_CTRL			0x8
#define DDRC_LMR			0xc
#define DDRC_REFCNT			0x18
#define DDRC_MMAP0			0x24
#define DDRC_MMAP1			0x28
#define DDRC_DLP			0xbc
#define DDRC_STRB			0x34
#define DDRC_AUTOSR_CNT                 0x308
#define DDRC_AUTOSR_EN		        0x304


#define DDRP_DXnDQSTR(n)     (DDR_PHY_OFFSET + (0x10 * n + 0x75) * 4)
#define DDRP_DXnDQTR(n)      (DDR_PHY_OFFSET + (0x10 * n + 0x74) * 4)

#ifndef REG32
#define REG32(x) *(volatile unsigned int *)(x)
#endif

#define ddr_writel(value, reg)	REG32(DDRC_BASE + reg) = (value)
#define ddr_readl(reg)		REG32(DDRC_BASE + reg)



#endif /* _DDR_H_ */
