/*
 * linux/arch/mips/xburst/soc-m200/common/pm_p0.c
 *
 *  M200 Power Management Routines
 *  Copyright (C) 2006 - 2012 Ingenic Semiconductor Inc.
 *
 *  This program is free software; you can distribute it and/or modify it
 *  under the terms of the GNU General Public License (Version 2) as
 *  published by the Free Software Foundation.
 *
 *  This program is distributed in the hope it will be useful, but WITHOUT
 *  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 *  FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
 *  for more details.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with this program; if not, write to the Free Software Foundation, Inc.,
 *  59 Temple Place - Suite 330, Boston MA 02111-1307, USA.
 *
 */

#include <linux/init.h>
#include <linux/pm.h>
#include <linux/sched.h>
#include <linux/interrupt.h>
#include <linux/suspend.h>
#include <linux/proc_fs.h>
#include <linux/syscalls.h>
#include <linux/fs.h>
#include <linux/sysctl.h>
#include <linux/delay.h>
#include <asm/fpu.h>
#include <linux/syscore_ops.h>
#include <linux/regulator/consumer.h>
#include <linux/clk.h>

#include <asm/cacheops.h>
#include <soc/cache.h>
#include <asm/r4kcache.h>
#include <soc/base.h>
#include <soc/cpm.h>
#include <soc/ddr.h>
#include <tcsm.h>
#include <smp_cp0.h>

extern long long save_goto(unsigned int);
extern int restore_goto(void);

#define get_cp0_ebase()	__read_32bit_c0_register($15, 1)

#define OFF_TDR         (0x00)
#define OFF_LCR         (0x0C)
#define OFF_LSR         (0x14)

#define LSR_TDRQ        (1 << 5)
#define LSR_TEMT        (1 << 6)

#define U1_IOBASE (UART1_IOBASE + 0xa0000000)
#define TCSM_PCHAR(x)							\
	*((volatile unsigned int*)(U1_IOBASE+OFF_TDR)) = x;		\
	while ((*((volatile unsigned int*)(U1_IOBASE + OFF_LSR)) & (LSR_TDRQ | LSR_TEMT)) != (LSR_TDRQ | LSR_TEMT))

#define TCSM_DELAY(x)					\
	i = x;						\
	while(i--)					\
		__asm__ volatile(".set mips32\n\t"	\
				 "nop\n\t"		\
				 ".set mips32")


static inline void serial_put_hex(unsigned int x) {
	int i;
	unsigned int d;
	for(i = 7;i >= 0;i--) {
		d = (x  >> (i * 4)) & 0xf;
		if(d < 10) d += '0';
		else d += 'A' - 10;
		TCSM_PCHAR(d);
	}
}
#ifdef DDR_MEM_TEST
#define MEM_TEST_SIZE   0x100000
static unsigned int test_mem_space[MEM_TEST_SIZE / 4];

static inline void test_ddr_data_init(void) {
	int i;
	unsigned int *test_mem;
	test_mem = (unsigned int *)((unsigned int)test_mem_space | 0xa0000000);
	dma_cache_wback_inv((unsigned int)test_mem_space,0x100000);
	for(i = 0;i < MEM_TEST_SIZE / 4;i++) {
		test_mem[i] = (unsigned int)&test_mem[i];
	}
}
static inline void check_ddr_data(void) {
	int i;
	unsigned int *test_mem;
	test_mem = (unsigned int *)((unsigned int)test_mem_space | 0xa0000000);
	for(i = 0;i < MEM_TEST_SIZE / 4;i++) {
		unsigned int dd;
		dd = test_mem[i];
		if(dd != (unsigned int)&test_mem[i]) {
			serial_put_hex(dd);
			TCSM_PCHAR(' ');
			serial_put_hex(i);
			TCSM_PCHAR(' ');
			serial_put_hex((unsigned int)&test_mem[i]);
			TCSM_PCHAR('\r');
			TCSM_PCHAR('\n');
		}
	}
}
#endif
static inline void dump_ddr_param(void) {
	int i;
	for(i = 0;i < 4;i++) {
		TCSM_PCHAR('<');
		serial_put_hex(i);
		TCSM_PCHAR('>');
		TCSM_PCHAR('<');
		serial_put_hex(ddr_readl(DDRP_DXnDQSTR(i)));
		TCSM_PCHAR('>');
		serial_put_hex(ddr_readl(DDRP_DXnDQTR(i)));
		TCSM_PCHAR('\r');
		TCSM_PCHAR('\n');
	}
	TCSM_PCHAR(':');
	serial_put_hex(ddr_readl(DDRP_PGSR));
	TCSM_PCHAR('\r');
	TCSM_PCHAR('\n');

}
extern void dump_clk(void);
struct save_reg
{
	unsigned int addr;
	unsigned int value;
};
#define SUSPEND_SAVE_REG_SIZE 10
static struct save_reg m_save_reg[SUSPEND_SAVE_REG_SIZE];
static int m_save_reg_count = 0;
static unsigned int read_save_reg_add(unsigned int addr)
{
	unsigned int value = REG32(CKSEG1ADDR(addr));
	if(m_save_reg_count < SUSPEND_SAVE_REG_SIZE) {
		m_save_reg[m_save_reg_count].addr = addr;
		m_save_reg[m_save_reg_count].value = value;
		m_save_reg_count++;
	}else
		printk("suspend_reg buffer size too small!\n");
	return value;
}
static void restore_all_reg(void)
{
	int i;
	for(i = 0;i < m_save_reg_count;i++) {
		REG32(CKSEG1ADDR(m_save_reg[i].addr)) = m_save_reg[i].value;
	}
	m_save_reg_count = 0;
}
static inline void config_powerdown_core(unsigned int *resume_pc) {
	unsigned int cpu_no,opcr;
	/* set SLBC and SLPC */
	cpm_outl(1,CPM_SLBC);
	/* Clear previous reset status */
	cpm_outl(0,CPM_RSR);
	/* OPCR.PD and OPCR.L2C_PD */
	cpu_no = get_cp0_ebase() & 1;
	opcr = cpm_inl(CPM_OPCR);
	//p 0 or 1 powerdown
	opcr &= ~(3<<25);
	opcr |= (cpu_no + 1) << 25;
	cpm_outl(opcr,CPM_OPCR);

	printk("opcr = %x\n",cpm_inl(CPM_OPCR));
	printk("lcr = %x\n",cpm_inl(CPM_LCR));

	// set resume pc
	cpm_outl((unsigned int)resume_pc,CPM_SLPC);
	blast_dcache32();
	blast_icache32();
	blast_scache32();
}
/* void set_gpio_func(int port,int pin,int type) { */
/* 	int i; */
/* 	int addr = 0xb0010010 + port * 0x100; */
/* 	for(i = 0;i < 4;i++){ */
/* 		REG32(addr + 0x10 * i) &= ~(1 << pin); */
/* 		REG32(addr + 0x10 * i) |= (((type >> (3 - i)) & 1) << pin); */
/* 	} */
/* } */

int get_gpio_func(int port,int pin) {
	int i;
	int ret = 0;
	int addr = 0xb0010010 + port * 0x100;
	for(i = 0;i < 4;i++){
		ret |= ((REG32(addr + 0x10 * i) >> pin) & 1)  << (3 - i);
	}
	return ret;
}

#define SLEEP_TSCM_SPACE    0xb3423000
#define SLEEP_TSCM_DATA_LEN 0x20
#define SLEEP_TSCM_TEXT     (SLEEP_TSCM_SPACE+SLEEP_TSCM_DATA_LEN)
#define SLEEP_TSCM_DATA     (SLEEP_TSCM_SPACE)
#define SLEEP_TSCM_TEXT_LEN (2048 - SLEEP_TSCM_DATA_LEN)
static noinline void cpu_sleep(void)
{
	register unsigned int val;
	config_powerdown_core((unsigned int *)SLEEP_TSCM_TEXT);
	__asm__ volatile(".set mips32\n\t"
			 "sync\n\t"
			 "lw $0,0(%0)\n\t"
			 "nop\n\t"
			 "nop\n\t"
			 "nop\n\t"
			 ".set mips32 \n\t"
			 :
			 : "r" (0xa0000000) );
	/* printk("sleep!\n"); */
	/* printk("int mask:0x%08x\n",REG32(0xb0001004)); */
	/* printk("gate:0x%08x\n",cpm_inl(CPM_CLKGR)); */
	/* printk("CPM_DDRCDR:0x%08x\n",cpm_inl(CPM_DDRCDR)); */
	/* printk("DDRC_AUTOSR_EN: %x\n",ddr_readl(DDRC_AUTOSR_EN)); */
	/* printk("DDRC_DLP: %x\n",ddr_readl(DDRC_DLP)); */

	cache_prefetch(LABLE1,200);
LABLE1:
	val = ddr_readl(DDRC_AUTOSR_EN);
	REG32(SLEEP_TSCM_DATA + 0) = val;

	ddr_writel(0,DDRC_AUTOSR_EN);             // exit auto sel-refresh
	val = ddr_readl(DDRC_DLP);
	REG32(SLEEP_TSCM_DATA + 4) = val;
	if(!(ddr_readl(DDRP_PIR) & DDRP_PIR_DLLBYP) && !val)
	{
		ddr_writel(0xf003 , DDRC_DLP);
		val = ddr_readl(DDRP_DSGCR);
		val |= (1 << 4);
		ddr_writel(val,DDRP_DSGCR);
	}

	val = ddr_readl(DDRC_CTRL);
	val |= (1 << 17);   // enter to hold ddr state
	ddr_writel(val,DDRC_CTRL);

	__asm__ volatile(".set mips32\n\t"
			 "wait\n\t"
			 "nop\n\t"
			 "nop\n\t"
			 "nop\n\t"
			 ".set mips32 \n\t");

	/* { */
	/* 	void (*f)(void); */
	/* 	f = (void (*)(void))cpm_inl(CPM_SLPC); */
	/* 	f(); */
	/* } */
	while(1)
		TCSM_PCHAR('n');

}
static noinline void cpu_resume(void)
{
	register int val = 0;
	register int bypassmode = 0;
	TCSM_PCHAR('o');
	bypassmode = ddr_readl(DDRP_PIR) & DDRP_PIR_DLLBYP;
	if(!bypassmode) {
		val = DDRP_PIR_INIT | DDRP_PIR_DLLSRST | DDRP_PIR_DLLLOCK | DDRP_PIR_ITMSRST;
	RETRY_LABLE:
		ddr_writel(val, DDRP_PIR);
		while((ddr_readl(DDRP_DX0GSR) & 0x3) != 3);
		while (ddr_readl(DDRP_PGSR) != (DDRP_PGSR_IDONE | DDRP_PGSR_DLDONE | DDRP_PGSR_ZCDONE
						| DDRP_PGSR_DIDONE | DDRP_PGSR_DTDONE)) {

			if(ddr_readl(DDRP_PGSR) & (DDRP_PGSR_DTERR | DDRP_PGSR_DTIERR)) {

				ddr_writel(1 << 28,DDRP_PIR);
				while((ddr_readl(DDRP_DX0GSR) & 0x3) != 0)
					TCSM_PCHAR('1');
				val++;
				serial_put_hex(val);
				TCSM_PCHAR('\r');
				TCSM_PCHAR('\n');
				goto RETRY_LABLE;
			}
		}
	}

	val = ddr_readl(DDRC_CTRL);
	val &= ~(1<< 17);    // exit to hold ddr state
	ddr_writel(val,DDRC_CTRL);
	serial_put_hex(REG32(SLEEP_TSCM_DATA + 4));
	if(!REG32(SLEEP_TSCM_DATA + 4) && !bypassmode)
	{
		ddr_writel(0x0 , DDRC_DLP);
		{
			val = ddr_readl(DDRP_DSGCR);
			val &= ~(1 << 4);
			ddr_writel(val,DDRP_DSGCR);
		}
	}
	if(REG32(SLEEP_TSCM_DATA + 0))
		ddr_writel(1,DDRC_AUTOSR_EN);   // enter auto sel-refresh


	dump_ddr_param();
#ifdef DDR_MEM_TEST
	check_ddr_data();
#endif
	write_c0_ecc(0x0);
	__jz_cache_init();
	TCSM_PCHAR('r');
	__asm__ volatile(".set mips32\n\t"
			 "jr %0\n\t"
			 "nop\n\t"
			 ".set mips32 \n\t" :: "r" (restore_goto));

}
static void load_func_to_tcsm(unsigned int *tcsm_addr,unsigned int *f_addr,unsigned int size)
{
	unsigned int instr;
	int offset;
	int i;
	for(i = 0;i < size / 4;i++) {
		instr = f_addr[i];
		if((instr >> 26) == 2){
			offset = instr & 0x3ffffff;
			offset = (offset << 2) - ((unsigned int)f_addr & 0xfffffff);
			if(offset > 0) {
				offset = ((unsigned int)tcsm_addr & 0xfffffff) + offset;
				instr = (2 << 26) | (offset >> 2);
			}
		}
		tcsm_addr[i] = instr;
	}
}
static int m200_pm_enter(suspend_state_t state)
{

	unsigned int  lcr_tmp;
	unsigned int  opcr_tmp;
	unsigned int gate,spcr0;
	unsigned int core_ctrl;
	unsigned int i;

	disable_fpu();
#ifdef DDR_MEM_TEST
	test_ddr_data_init();
#endif
	for(i = 0;i < SLEEP_TSCM_DATA_LEN;i += 4)
		REG32(SLEEP_TSCM_DATA + i) = 0;
	load_func_to_tcsm((unsigned int *)SLEEP_TSCM_TEXT,(unsigned int *)cpu_resume,SLEEP_TSCM_TEXT_LEN);

	lcr_tmp = read_save_reg_add(CPM_IOBASE + CPM_LCR);
	lcr_tmp &= ~3;
	lcr_tmp |= LCR_LPM_SLEEP;
	cpm_outl(lcr_tmp,CPM_LCR);
	/* OPCR.MASK_INT bit30*/
	/* set Oscillator Stabilize Time bit8*/
	/* disable externel clock Oscillator in sleep mode bit4*/
	/* select 32K crystal as RTC clock in sleep mode bit2*/
        opcr_tmp = read_save_reg_add(CPM_IOBASE + CPM_OPCR);
	opcr_tmp &= ~((1 << 7) | (1 << 6) | (1 << 4));
	opcr_tmp |= (0xff << 8) | (1<<30) | (1 << 2) | (1 << 27) | (1 << 23);
        cpm_outl(opcr_tmp,CPM_OPCR);
	/*
	 * set sram pdma_ds & open nfi
	 */
	spcr0 = read_save_reg_add(CPM_IOBASE + CPM_SPCR0);
	spcr0 |= (1 << 31);
	spcr0 &= ~((1 << 27) | (1 << 2) | (1 << 15) | (1 << 31));
	cpm_outl(spcr0,CPM_SPCR0);

	/*
	 * set clk gate nfi nemc enable pdma
	 */
	gate = read_save_reg_add(CPM_IOBASE + CPM_CLKGR);
	gate &= ~(3  | (1 << 21));
	cpm_outl(gate,CPM_CLKGR);
	core_ctrl = get_smp_ctrl();
	set_smp_ctrl(core_ctrl & ~(3 << 8));

	//read_save_reg_add(0x134f0304);
	//REG32(0xb34f0304) = 0;       // exit auto sel-refresh
	//__fast_iob();
	mb();
	save_goto((unsigned int)cpu_sleep);
	mb();
	restore_all_reg();

	set_smp_ctrl(core_ctrl);
	return 0;
}
static struct m200_early_sleep_t {
	struct regulator*  core_vcc;
	struct clk *cpu_clk;
	unsigned int rate_hz;
	unsigned int vol_uv;

}m200_early_sleep;
const unsigned int sleep_rate_hz = 24*1000*1000;
const unsigned int sleep_vol_uv = 1025 * 1000;
static int m200_prepare(void)
{
	m200_early_sleep.core_vcc = regulator_get(NULL,"cpu_core");
	m200_early_sleep.rate_hz = clk_get_rate(m200_early_sleep.cpu_clk);
	clk_set_rate(m200_early_sleep.cpu_clk,sleep_rate_hz);
	if(!IS_ERR(m200_early_sleep.core_vcc))
	{
		m200_early_sleep.vol_uv = regulator_get_voltage(m200_early_sleep.core_vcc);
		printk("save vol_uv = %d\n",m200_early_sleep.vol_uv);
		regulator_set_voltage(m200_early_sleep.core_vcc,sleep_vol_uv,sleep_vol_uv);
	}
	return 0;
}
static void m200_finish(void)
{
	if(!IS_ERR(m200_early_sleep.core_vcc))
	{
		regulator_set_voltage(m200_early_sleep.core_vcc,m200_early_sleep.vol_uv,m200_early_sleep.vol_uv);
		regulator_put(m200_early_sleep.core_vcc);
	}
	clk_set_rate(m200_early_sleep.cpu_clk,m200_early_sleep.rate_hz);
}
/*
 * Initialize power interface
 */
struct platform_suspend_ops pm_ops = {
	.valid = suspend_valid_only_mem,
	.enter = m200_pm_enter,
	.prepare = m200_prepare,
	.finish = m200_finish,
};
//extern void ddr_retention_exit(void);
//extern void ddr_retention_entry(void);

int __init m200_pm_init(void)
{
        volatile unsigned int lcr,opcr;//,i;

	suspend_set_ops(&pm_ops);

        /* init opcr and lcr for idle */
        lcr = cpm_inl(CPM_LCR);
        lcr &= ~(0x7);		/* LCR.SLEEP.DS=1'b0,LCR.LPM=2'b00*/
        lcr |= 0xff << 8;	/* power stable time */
        cpm_outl(lcr,CPM_LCR);

        opcr = cpm_inl(CPM_OPCR);
        opcr &= ~(2 << 25);	/* OPCR.PD=2'b00 */
        opcr |= 0xff << 8;	/* EXCLK stable time */
        cpm_outl(opcr,CPM_OPCR);
	m200_early_sleep.cpu_clk = clk_get(NULL, "cclk");
	if (IS_ERR(m200_early_sleep.cpu_clk)) {
		printk("ERROR:cclk request fail!");
		suspend_set_ops(NULL);
		return -1;
	}
        /* sysfs */
	return 0;
}

arch_initcall(m200_pm_init);
