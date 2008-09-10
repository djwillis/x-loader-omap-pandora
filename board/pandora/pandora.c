/*
 * (C) Copyright 2006
 * Texas Instruments, <www.ti.com>
 * Jian Zhang <jzhang@ti.com>
 * Richard Woodruff <r-woodruff2@ti.com>
 *
 * Modified for the Pandora
 * John Willis <source@distant-earth.com>
 *
 * See file CREDITS for list of people who contributed to this
 * project.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

#include <common.h>
#include <command.h>
#include <part.h>
#include <fat.h>
#include <asm/arch/cpu.h>
#include <asm/arch/bits.h>
#include <asm/arch/mux.h>
#include <asm/arch/sys_proto.h>
#include <asm/arch/sys_info.h>
#include <asm/arch/clocks.h>
#include <asm/arch/mem.h>
//#include <i2c.h>
//#include <mmc.h>

//#define	CLK_MULT	3

/* Used to index into DPLL parameter tables */
struct dpll_param {
	unsigned int m;
	unsigned int n;
	unsigned int fsel;
	unsigned int m2;
};

typedef struct dpll_param dpll_param;

/* Following functions are exported from lowlevel_init.S */
extern dpll_param *get_mpu_dpll_param(void);
extern dpll_param *get_iva_dpll_param(void);
extern dpll_param *get_core_dpll_param(void);
extern dpll_param *get_per_dpll_param(void);

#define __raw_readl(a)    (*(volatile unsigned int *)(a))
#define __raw_writel(v,a) (*(volatile unsigned int *)(a) = (v))
#define __raw_readw(a)    (*(volatile unsigned short *)(a))
#define __raw_writew(v,a) (*(volatile unsigned short *)(a) = (v))
//extern int fat_register_device(block_dev_desc_t * dev_desc, int part_no);
//extern block_dev_desc_t *mmc_get_dev(int dev);

/*******************************************************
 * Routine: delay
 * Description: spinning delay to use before udelay works
 ******************************************************/
static inline void delay(unsigned long loops)
{
	__asm__ volatile ("1:\n" "subs %0, %1, #1\n"
			  "bne 1b":"=r" (loops):"0"(loops));
}

void udelay (unsigned long usecs) {
	delay(usecs);
}

/*****************************************
 * Routine: board_init
 * Description: Early hardware init.
 *****************************************/
int board_init(void)
{
	return 0;
}

/*************************************************************
 *  get_device_type(): tell if GP/HS/EMU/TST
 *************************************************************/
u32 get_device_type(void)
{
	int mode;
	mode = __raw_readl(CONTROL_STATUS) & (DEVICE_MASK);
	return (mode >>= 8);
}

/************************************************
 * get_sysboot_value(void) - return SYS_BOOT[4:0]
 ************************************************/
u32 get_sysboot_value(void)
{
	int mode;
	mode = __raw_readl(CONTROL_STATUS) & (SYSBOOT_MASK);
	return mode;
}

/*************************************************************
 * Routine: get_mem_type(void) - returns the kind of memory connected
 * to GPMC that we are trying to boot form. Uses SYS BOOT settings.
 *************************************************************/
u32 get_mem_type(void)
{
	u32   mem_type = get_sysboot_value();
	switch (mem_type) {
	case 0:
	case 2:
	case 4:
	case 16:
	case 22:
		return GPMC_ONENAND;

	case 1:
	case 12:
	case 15:
	case 21:
	case 27:
		return GPMC_NAND;

	case 3:
	case 6:
		return MMC_ONENAND;

	case 8:
	case 11:
	case 14:
	case 20:
	case 26:
		return GPMC_MDOC;

	case 17:
	case 18:
	case 24:
		return MMC_NAND;

	case 7:
	case 10:
	case 13:
	case 19:
	case 25:
	default:
		return GPMC_NOR;
	}
}

/******************************************
 * get_cpu_rev(void) - extract version info
 ******************************************/
u32 get_cpu_rev(void)
{
	u32 cpuid = 0;
	/* On ES1.0 the IDCODE register is not exposed on L4
	 * so using CPU ID to differentiate
	 * between ES2.0 and ES1.0.
	 */
	__asm__ __volatile__("mrc p15, 0, %0, c0, c0, 0":"=r"(cpuid));
	if ((cpuid & 0xf) == 0x0)
		return CPU_3430_ES1;
	else
		return CPU_3430_ES2;

}

/******************************************
 * cpu_is_3410(void) - returns true for 3410
 ******************************************/
u32 cpu_is_3410(void)
{
	int status;
	if (get_cpu_rev() < CPU_3430_ES2) {
		return 0;
	} else {
		/* read scalability status and return 1 for 3410*/
		status = __raw_readl(CONTROL_SCALABLE_OMAP_STATUS);
		/* Check whether MPU frequency is set to 266 MHz which
		 * is nominal for 3410. If yes return true else false
		 */
		if (((status >> 8) & 0x3) == 0x2)
			return 1;
		else
			return 0;
	}
}

/*****************************************************************
 * sr32 - clear & set a value in a bit range for a 32 bit address
 *****************************************************************/
void sr32(u32 addr, u32 start_bit, u32 num_bits, u32 value)
{
	u32 tmp, msk = 0;
	msk = 1 << num_bits;
	--msk;
	tmp = __raw_readl(addr) & ~(msk << start_bit);
	tmp |= value << start_bit;
	__raw_writel(tmp, addr);
}

/*********************************************************************
 * wait_on_value() - common routine to allow waiting for changes in
 *   volatile regs.
 *********************************************************************/
u32 wait_on_value(u32 read_bit_mask, u32 match_value, u32 read_addr, u32 bound)
{
	u32 i = 0, val;
	do {
		++i;
		val = __raw_readl(read_addr) & read_bit_mask;
		if (val == match_value)
			return (1);
		if (i == bound)
			return (0);
	} while (1);
}

#ifdef CFG_3430SDRAM_DDR
/*********************************************************************
 * config_3430sdram_ddr() - Init DDR on 3430SDP dev board.
 *********************************************************************/
void config_3430sdram_ddr(void)
{
        /* reset sdrc controller */
        __raw_writel(SOFTRESET, SDRC_SYSCONFIG);
        wait_on_value(BIT0, BIT0, SDRC_STATUS, 12000000);
        __raw_writel(0, SDRC_SYSCONFIG);

        /* setup sdrc to ball mux */
        __raw_writel(SDP_SDRC_SHARING, SDRC_SHARING);

        /* SDRC put in weak */
//        (*(unsigned int*)0x6D00008C) = 0x00000020;

        /* SDRC_MCFG0 register */
        (*(unsigned int*)0x6D000080) = 0x02584099;//from Micron

        /* SDRC_ACTIM_CTRLA0 register */
//our value        (*(unsigned int*)0x6D00009c) = 0xa29db4c6;// for 166M
        (*(unsigned int*)0x6D00009c) = 0xaa9db4c6;// for 166M from rkw

        /* SDRC_ACTIM_CTRLB0 register */
//from micron   (*(unsigned int*)0x6D0000a0) = 0x12214;// for 166M

//        (*(unsigned int*)0x6D0000a0) = 0x00011417; our value
        (*(unsigned int*)0x6D0000a0) = 0x00011517;

        /* SDRC_RFR_CTRL0 register */
//from micron   (*(unsigned int*)0x6D0000a4) =0x54601; // for 166M

        (*(unsigned int*)0x6D0000a4) =0x0004DC01;

        /* Disble Power Down of CKE cuz of 1 CKE on combo part */
        (*(unsigned int*)0x6D000070) = 0x00000081;

        /* SDRC_Manual command register */
        (*(unsigned int*)0x6D0000a8) = 0x00000000; // NOP command
        delay(5000);
        (*(unsigned int*)0x6D0000a8) = 0x00000001; // Precharge command
        (*(unsigned int*)0x6D0000a8) = 0x00000002; // Auto-refresh command
        (*(unsigned int*)0x6D0000a8) = 0x00000002; // Auto-refresh command

        /* SDRC MR0 register */
        (*(int*)0x6D000084) = 0x00000032; // Burst length =4
        // CAS latency = 3
        // Write Burst = Read Burst
        // Serial Mode

        /* SDRC DLLA control register */
        (*(unsigned int*)0x6D000060) = 0x0000A;
        delay(0x20000); // some delay

}
#endif /* CFG_3430SDRAM_DDR */

/*************************************************************
 * get_sys_clk_speed - determine reference oscillator speed
 *  based on known 32kHz clock and gptimer.
 *************************************************************/
u32 get_osc_clk_speed(void)
{
	u32 start, cstart, cend, cdiff, val;

	val = __raw_readl(PRM_CLKSRC_CTRL);
	/* If SYS_CLK is being divided by 2, remove for now */
	val = (val & (~BIT7)) | BIT6;
	__raw_writel(val, PRM_CLKSRC_CTRL);

	/* enable timer2 */
	val = __raw_readl(CM_CLKSEL_WKUP) | BIT0;
	__raw_writel(val, CM_CLKSEL_WKUP);	/* select sys_clk for GPT1 */

	/* Enable I and F Clocks for GPT1 */
	val = __raw_readl(CM_ICLKEN_WKUP) | BIT0 | BIT2;
	__raw_writel(val, CM_ICLKEN_WKUP);
	val = __raw_readl(CM_FCLKEN_WKUP) | BIT0;
	__raw_writel(val, CM_FCLKEN_WKUP);

	__raw_writel(0, OMAP34XX_GPT1 + TLDR);	/* start counting at 0 */
	__raw_writel(GPT_EN, OMAP34XX_GPT1 + TCLR);	/* enable clock */
	/* enable 32kHz source */
	/* enabled out of reset */
	/* determine sys_clk via gauging */

	start = 20 + __raw_readl(S32K_CR);	/* start time in 20 cycles */
	while (__raw_readl(S32K_CR) < start) ;	/* dead loop till start time */
	cstart = __raw_readl(OMAP34XX_GPT1 + TCRR);	/* get start sys_clk count */
	while (__raw_readl(S32K_CR) < (start + 20)) ;	/* wait for 40 cycles */
	cend = __raw_readl(OMAP34XX_GPT1 + TCRR);	/* get end sys_clk count */
	cdiff = cend - cstart;	/* get elapsed ticks */

	/* based on number of ticks assign speed */
	if (cdiff > 19000)
		return (S38_4M);
	else if (cdiff > 15200)
		return (S26M);
	else if (cdiff > 13000)
		return (S24M);
	else if (cdiff > 9000)
		return (S19_2M);
	else if (cdiff > 7600)
		return (S13M);
	else
		return (S12M);
}

/******************************************************************************
 * get_sys_clkin_sel() - returns the sys_clkin_sel field value based on
 *   -- input oscillator clock frequency.
 *
 *****************************************************************************/
void get_sys_clkin_sel(u32 osc_clk, u32 * sys_clkin_sel)
{
	if (osc_clk == S38_4M)
		*sys_clkin_sel = 4;
	else if (osc_clk == S26M)
		*sys_clkin_sel = 3;
	else if (osc_clk == S19_2M)
		*sys_clkin_sel = 2;
	else if (osc_clk == S13M)
		*sys_clkin_sel = 1;
	else if (osc_clk == S12M)
		*sys_clkin_sel = 0;
}

/******************************************************************************
 * prcm_init() - inits clocks for PRCM as defined in clocks.h
 *   -- called from SRAM, or Flash (using temp SRAM stack).
 *****************************************************************************/
void prcm_init(void)
{
	u32 osc_clk = 0, sys_clkin_sel;
	dpll_param *dpll_param_p;
	u32 clk_index, sil_index;

	/* Gauge the input clock speed and find out the sys_clkin_sel
	 * value corresponding to the input clock.
	 */
	osc_clk = get_osc_clk_speed();
	get_sys_clkin_sel(osc_clk, &sys_clkin_sel);

	sr32(PRM_CLKSEL, 0, 3, sys_clkin_sel);	/* set input crystal speed */

	/* If the input clock is greater than 19.2M always divide/2 */
	if (sys_clkin_sel > 2) {
		sr32(PRM_CLKSRC_CTRL, 6, 2, 2);	/* input clock divider */
		clk_index = sys_clkin_sel / 2;
	} else {
		sr32(PRM_CLKSRC_CTRL, 6, 2, 1);	/* input clock divider */
		clk_index = sys_clkin_sel;
	}

	sr32(PRM_CLKSRC_CTRL, 0, 2, 0);/* Bypass mode: T2 inputs a square clock */

	/* The DPLL tables are defined according to sysclk value and
	 * silicon revision. The clk_index value will be used to get
	 * the values for that input sysclk from the DPLL param table
	 * and sil_index will get the values for that SysClk for the
	 * appropriate silicon rev.
	 */
	sil_index = get_cpu_rev() - 1;

	/* Unlock MPU DPLL (slows things down, and needed later) */
	sr32(CM_CLKEN_PLL_MPU, 0, 3, PLL_LOW_POWER_BYPASS);
	wait_on_value(BIT0, 0, CM_IDLEST_PLL_MPU, LDELAY);

	/* Getting the base address of Core DPLL param table */
	dpll_param_p = (dpll_param *) get_core_dpll_param();
	/* Moving it to the right sysclk and ES rev base */
	dpll_param_p = dpll_param_p + 2*clk_index + sil_index;
	/* CORE DPLL */
	/* sr32(CM_CLKSEL2_EMU) set override to work when asleep */
	sr32(CM_CLKEN_PLL, 0, 3, PLL_FAST_RELOCK_BYPASS);
	wait_on_value(BIT0, 0, CM_IDLEST_CKGEN, LDELAY);
	sr32(CM_CLKSEL1_EMU, 16, 5, CORE_M3X2);	/* m3x2 */
	sr32(CM_CLKSEL1_PLL, 27, 2, dpll_param_p->m2);	/* Set M2 */
	sr32(CM_CLKSEL1_PLL, 16, 11, dpll_param_p->m);	/* Set M */
	sr32(CM_CLKSEL1_PLL, 8, 7, dpll_param_p->n);	/* Set N */
	sr32(CM_CLKSEL1_PLL, 6, 1, 0);	/* 96M Src */
	sr32(CM_CLKSEL_CORE, 8, 4, CORE_SSI_DIV);	/* ssi */
	sr32(CM_CLKSEL_CORE, 4, 2, CORE_FUSB_DIV);	/* fsusb */
	sr32(CM_CLKSEL_CORE, 2, 2, CORE_L4_DIV);	/* l4 */
	sr32(CM_CLKSEL_CORE, 0, 2, CORE_L3_DIV);	/* l3 */
	sr32(CM_CLKSEL_GFX, 0, 3, GFX_DIV);	/* gfx */
	sr32(CM_CLKSEL_WKUP, 1, 2, WKUP_RSM);	/* reset mgr */
	sr32(CM_CLKEN_PLL, 4, 4, dpll_param_p->fsel);	/* FREQSEL */
	sr32(CM_CLKEN_PLL, 0, 3, PLL_LOCK);	/* lock mode */
	wait_on_value(BIT0, 1, CM_IDLEST_CKGEN, LDELAY);

	/* Getting the base address to PER  DPLL param table */
	dpll_param_p = (dpll_param *) get_per_dpll_param();
	/* Moving it to the right sysclk base */
	dpll_param_p = dpll_param_p + clk_index;
	/* PER DPLL */
	sr32(CM_CLKEN_PLL, 16, 3, PLL_STOP);
	wait_on_value(BIT1, 0, CM_IDLEST_CKGEN, LDELAY);
	sr32(CM_CLKSEL1_EMU, 24, 5, PER_M6X2);	/* set M6 */
	sr32(CM_CLKSEL_CAM, 0, 5, PER_M5X2);	/* set M5 */
	sr32(CM_CLKSEL_DSS, 0, 5, PER_M4X2);	/* set M4 */
	sr32(CM_CLKSEL_DSS, 8, 5, PER_M3X2);	/* set M3 */
	sr32(CM_CLKSEL3_PLL, 0, 5, dpll_param_p->m2);	/* set M2 */
	sr32(CM_CLKSEL2_PLL, 8, 11, dpll_param_p->m);	/* set m */
	sr32(CM_CLKSEL2_PLL, 0, 7, dpll_param_p->n);	/* set n */
	sr32(CM_CLKEN_PLL, 20, 4, dpll_param_p->fsel);	/* FREQSEL */
	sr32(CM_CLKEN_PLL, 16, 3, PLL_LOCK);	/* lock mode */
	wait_on_value(BIT1, 2, CM_IDLEST_CKGEN, LDELAY);

	/* Getting the base address to MPU DPLL param table */
	dpll_param_p = (dpll_param *) get_mpu_dpll_param();
	/* Moving it to the right sysclk and ES rev base */
	dpll_param_p = dpll_param_p + 2*clk_index + sil_index;
	/* MPU DPLL (unlocked already) */
	sr32(CM_CLKSEL2_PLL_MPU, 0, 5, dpll_param_p->m2);	/* Set M2 */
	sr32(CM_CLKSEL1_PLL_MPU, 8, 11, dpll_param_p->m);	/* Set M */
	sr32(CM_CLKSEL1_PLL_MPU, 0, 7, dpll_param_p->n);	/* Set N */
	sr32(CM_CLKEN_PLL_MPU, 4, 4, dpll_param_p->fsel);	/* FREQSEL */
	sr32(CM_CLKEN_PLL_MPU, 0, 3, PLL_LOCK);	/* lock mode */
	wait_on_value(BIT0, 1, CM_IDLEST_PLL_MPU, LDELAY);

	/* Getting the base address to IVA DPLL param table */
	dpll_param_p = (dpll_param *) get_iva_dpll_param();
	/* Moving it to the right sysclk and ES rev base */
	dpll_param_p = dpll_param_p + 2*clk_index + sil_index;
	/* IVA DPLL (set to 12*20=240MHz) */
	sr32(CM_CLKEN_PLL_IVA2, 0, 3, PLL_STOP);
	wait_on_value(BIT0, 0, CM_IDLEST_PLL_IVA2, LDELAY);
	sr32(CM_CLKSEL2_PLL_IVA2, 0, 5, dpll_param_p->m2);	/* set M2 */
	sr32(CM_CLKSEL1_PLL_IVA2, 8, 11, dpll_param_p->m);	/* set M */
	sr32(CM_CLKSEL1_PLL_IVA2, 0, 7, dpll_param_p->n);	/* set N */
	sr32(CM_CLKEN_PLL_IVA2, 4, 4, dpll_param_p->fsel);	/* FREQSEL */
	sr32(CM_CLKEN_PLL_IVA2, 0, 3, PLL_LOCK);	/* lock mode */
	wait_on_value(BIT0, 1, CM_IDLEST_PLL_IVA2, LDELAY);

	/* Set up GPTimers to sys_clk source only */
	sr32(CM_CLKSEL_PER, 0, 8, 0xff);
	sr32(CM_CLKSEL_WKUP, 0, 1, 1);

	delay(5000);
}

/*****************************************
 * Routine: secure_unlock
 * Description: Setup security registers for access
 * (GP Device only)
 *****************************************/
void secure_unlock(void)
{
	/* Permission values for registers -Full fledged permissions to all */
#define UNLOCK_1 0xFFFFFFFF
#define UNLOCK_2 0x00000000
#define UNLOCK_3 0x0000FFFF
	/* Protection Module Register Target APE (PM_RT) */
	__raw_writel(UNLOCK_1, RT_REQ_INFO_PERMISSION_1);
	__raw_writel(UNLOCK_1, RT_READ_PERMISSION_0);
	__raw_writel(UNLOCK_1, RT_WRITE_PERMISSION_0);
	__raw_writel(UNLOCK_2, RT_ADDR_MATCH_1);

	__raw_writel(UNLOCK_3, GPMC_REQ_INFO_PERMISSION_0);
	__raw_writel(UNLOCK_3, GPMC_READ_PERMISSION_0);
	__raw_writel(UNLOCK_3, GPMC_WRITE_PERMISSION_0);

	__raw_writel(UNLOCK_3, OCM_REQ_INFO_PERMISSION_0);
	__raw_writel(UNLOCK_3, OCM_READ_PERMISSION_0);
	__raw_writel(UNLOCK_3, OCM_WRITE_PERMISSION_0);
	__raw_writel(UNLOCK_2, OCM_ADDR_MATCH_2);

	/* IVA Changes */
	__raw_writel(UNLOCK_3, IVA2_REQ_INFO_PERMISSION_0);
	__raw_writel(UNLOCK_3, IVA2_READ_PERMISSION_0);
	__raw_writel(UNLOCK_3, IVA2_WRITE_PERMISSION_0);

	__raw_writel(UNLOCK_1, SMS_RG_ATT0);	/* SDRC region 0 public */
}

/**********************************************************
 * Routine: try_unlock_sram()
 * Description: If chip is GP type, unlock the SRAM for
 *  general use.
 ***********************************************************/
void try_unlock_memory(void)
{
	int mode;

	/* if GP device unlock device SRAM for general use */
	/* secure code breaks for Secure/Emulation device - HS/E/T */
	mode = get_device_type();
	if (mode == GP_DEVICE) {
		secure_unlock();
	}
	return;
}

/**********************************************************
 * Routine: s_init
 * Description: Does early system init of muxing and clocks.
 * - Called at time when only stack is available.
 **********************************************************/

void s_init(void)
{
	watchdog_init();
#ifdef CONFIG_3430_AS_3410
	/* setup the scalability control register for
	 * 3430 to work in 3410 mode
	 */
	__raw_writel(0x5ABF, CONTROL_SCALABLE_OMAP_OCP);
#endif
	try_unlock_memory();
	set_muxconf_regs();
	delay(100);
	prcm_init();
	per_clocks_enable();
	config_3430sdram_ddr();
}

/*******************************************************
 * Routine: misc_init_r
 * Description: Init
 ********************************************************/
int misc_init_r(void)
{
	unsigned char byte;


	/*VAUX4 = 2.8V    (TOUCH,NUBS,RS-232 TRANSCEIVER)*/
	byte = 0x20;
	i2c_write(0x4B, 0x7E, 1, &byte, 1);
	byte = 0x0A;
	i2c_write(0x4B, 0x81, 1, &byte, 1);

	#ifdef CFG_PRINTF
		printf("Pandora X-Loader - USB Test \n");
		printf("Serial Initialised \n");
	#endif

	return (0);
}

/******************************************************
 * Routine: wait_for_command_complete
 * Description: Wait for posting to finish on watchdog
 ******************************************************/
void wait_for_command_complete(unsigned int wd_base)
{
	int pending = 1;
	do {
		pending = __raw_readl(wd_base + WWPS);
	} while (pending);
}

/****************************************
 * Routine: watchdog_init
 * Description: Shut down watch dogs
 *****************************************/
void watchdog_init(void)
{
	/* There are 3 watch dogs WD1=Secure, WD2=MPU, WD3=IVA. WD1 is
	 * either taken care of by ROM (HS/EMU) or not accessible (GP).
	 * We need to take care of WD2-MPU or take a PRCM reset.  WD3
	 * should not be running and does not generate a PRCM reset.
	 */
	sr32(CM_FCLKEN_WKUP, 5, 1, 1);
	sr32(CM_ICLKEN_WKUP, 5, 1, 1);
	wait_on_value(BIT5, 0x20, CM_IDLEST_WKUP, 5);	/* some issue here */

	__raw_writel(WD_UNLOCK1, WD2_BASE + WSPR);
	wait_for_command_complete(WD2_BASE);
	__raw_writel(WD_UNLOCK2, WD2_BASE + WSPR);
}

/**********************************************
 * Routine: dram_init
 * Description: sets uboots idea of sdram size
 **********************************************/
int dram_init(void)
{
	return 0;
}

/*****************************************************************
 * Routine: peripheral_enable
 * Description: Enable the clks & power for perifs (GPT2, UART1,...)
 ******************************************************************/
void per_clocks_enable(void)
{
	/* Enable GP2 timer. */
	sr32(CM_CLKSEL_PER, 0, 1, 0x1);	/* GPT2 = sys clk */
	sr32(CM_ICLKEN_PER, 3, 1, 0x1);	/* ICKen GPT2 */
	sr32(CM_FCLKEN_PER, 3, 1, 0x1);	/* FCKen GPT2 */

#ifdef CFG_NS16550
	/* Enable UART1 clocks */
	sr32(CM_FCLKEN1_CORE, 13, 1, 0x1);
	sr32(CM_ICLKEN1_CORE, 13, 1, 0x1);

	/* UART 3 Clocks */
	sr32(CM_FCLKEN_PER, 11, 1, 0x1);
	sr32(CM_ICLKEN_PER, 11, 1, 0x1);

#endif

#ifdef CONFIG_DRIVER_OMAP34XX_I2C
	/* Turn on all 3 I2C clocks */
	sr32(CM_FCLKEN1_CORE, 15, 3, 0x7);
	sr32(CM_ICLKEN1_CORE, 15, 3, 0x7);	/* I2C1,2,3 = on */
#endif

	/* Enable the ICLK for 32K Sync Timer as its used in udelay */
	sr32(CM_ICLKEN_WKUP, 2, 1, 0x1);

	sr32(CM_FCLKEN_IVA2, 0, 32, FCK_IVA2_ON);
	sr32(CM_FCLKEN1_CORE, 0, 32, FCK_CORE1_ON);
	sr32(CM_ICLKEN1_CORE, 0, 32, ICK_CORE1_ON);
	sr32(CM_ICLKEN2_CORE, 0, 32, ICK_CORE2_ON);
	sr32(CM_FCLKEN_WKUP, 0, 32, FCK_WKUP_ON);
	sr32(CM_ICLKEN_WKUP, 0, 32, ICK_WKUP_ON);
	sr32(CM_FCLKEN_DSS, 0, 32, FCK_DSS_ON);
	sr32(CM_ICLKEN_DSS, 0, 32, ICK_DSS_ON);
	sr32(CM_FCLKEN_CAM, 0, 32, FCK_CAM_ON);
	sr32(CM_ICLKEN_CAM, 0, 32, ICK_CAM_ON);
	sr32(CM_FCLKEN_PER, 0, 32, FCK_PER_ON);
	sr32(CM_ICLKEN_PER, 0, 32, ICK_PER_ON);

	/* Enable GPIO5 clocks for blinky LEDs */
	sr32(CM_FCLKEN_PER, 16, 1, 0x1);	/* FCKen GPIO5 */
	sr32(CM_ICLKEN_PER, 16, 1, 0x1);	/* ICKen GPIO5 */

	delay(1000);
}

/* Set MUX for UART, GPMC, SDRC, GPIO */

#define 	MUX_VAL(OFFSET,VALUE)\
		__raw_writew((VALUE), OMAP34XX_CTRL_BASE + (OFFSET));

#define		CP(x)	(CONTROL_PADCONF_##x)
/*
 * IEN  - Input Enable
 * IDIS - Input Disable
 * PTD  - Pull type Down
 * PTU  - Pull type Up
 * DIS  - Pull type selection is inactive
 * EN   - Pull type selection is active
 * M0   - Mode 0
 * The commented string gives the final mux configuration for that pin
 */
#define MUX_DEFAULT()\
	/*SDRC*/\
	MUX_VAL(CP(SDRC_D0),        (IEN  | PTD | DIS | M0)) /*SDRC_D0*/\
	MUX_VAL(CP(SDRC_D1),        (IEN  | PTD | DIS | M0)) /*SDRC_D1*/\
	MUX_VAL(CP(SDRC_D2),        (IEN  | PTD | DIS | M0)) /*SDRC_D2*/\
	MUX_VAL(CP(SDRC_D3),        (IEN  | PTD | DIS | M0)) /*SDRC_D3*/\
	MUX_VAL(CP(SDRC_D4),        (IEN  | PTD | DIS | M0)) /*SDRC_D4*/\
	MUX_VAL(CP(SDRC_D5),        (IEN  | PTD | DIS | M0)) /*SDRC_D5*/\
	MUX_VAL(CP(SDRC_D6),        (IEN  | PTD | DIS | M0)) /*SDRC_D6*/\
	MUX_VAL(CP(SDRC_D7),        (IEN  | PTD | DIS | M0)) /*SDRC_D7*/\
	MUX_VAL(CP(SDRC_D8),        (IEN  | PTD | DIS | M0)) /*SDRC_D8*/\
	MUX_VAL(CP(SDRC_D9),        (IEN  | PTD | DIS | M0)) /*SDRC_D9*/\
	MUX_VAL(CP(SDRC_D10),       (IEN  | PTD | DIS | M0)) /*SDRC_D10*/\
	MUX_VAL(CP(SDRC_D11),       (IEN  | PTD | DIS | M0)) /*SDRC_D11*/\
	MUX_VAL(CP(SDRC_D12),       (IEN  | PTD | DIS | M0)) /*SDRC_D12*/\
	MUX_VAL(CP(SDRC_D13),       (IEN  | PTD | DIS | M0)) /*SDRC_D13*/\
	MUX_VAL(CP(SDRC_D14),       (IEN  | PTD | DIS | M0)) /*SDRC_D14*/\
	MUX_VAL(CP(SDRC_D15),       (IEN  | PTD | DIS | M0)) /*SDRC_D15*/\
	MUX_VAL(CP(SDRC_D16),       (IEN  | PTD | DIS | M0)) /*SDRC_D16*/\
	MUX_VAL(CP(SDRC_D17),       (IEN  | PTD | DIS | M0)) /*SDRC_D17*/\
	MUX_VAL(CP(SDRC_D18),       (IEN  | PTD | DIS | M0)) /*SDRC_D18*/\
	MUX_VAL(CP(SDRC_D19),       (IEN  | PTD | DIS | M0)) /*SDRC_D19*/\
	MUX_VAL(CP(SDRC_D20),       (IEN  | PTD | DIS | M0)) /*SDRC_D20*/\
	MUX_VAL(CP(SDRC_D21),       (IEN  | PTD | DIS | M0)) /*SDRC_D21*/\
	MUX_VAL(CP(SDRC_D22),       (IEN  | PTD | DIS | M0)) /*SDRC_D22*/\
	MUX_VAL(CP(SDRC_D23),       (IEN  | PTD | DIS | M0)) /*SDRC_D23*/\
	MUX_VAL(CP(SDRC_D24),       (IEN  | PTD | DIS | M0)) /*SDRC_D24*/\
	MUX_VAL(CP(SDRC_D25),       (IEN  | PTD | DIS | M0)) /*SDRC_D25*/\
	MUX_VAL(CP(SDRC_D26),       (IEN  | PTD | DIS | M0)) /*SDRC_D26*/\
	MUX_VAL(CP(SDRC_D27),       (IEN  | PTD | DIS | M0)) /*SDRC_D27*/\
	MUX_VAL(CP(SDRC_D28),       (IEN  | PTD | DIS | M0)) /*SDRC_D28*/\
	MUX_VAL(CP(SDRC_D29),       (IEN  | PTD | DIS | M0)) /*SDRC_D29*/\
	MUX_VAL(CP(SDRC_D30),       (IEN  | PTD | DIS | M0)) /*SDRC_D30*/\
	MUX_VAL(CP(SDRC_D31),       (IEN  | PTD | DIS | M0)) /*SDRC_D31*/\
	MUX_VAL(CP(SDRC_CLK),       (IEN  | PTD | DIS | M0)) /*SDRC_CLK*/\
	MUX_VAL(CP(SDRC_DQS0),      (IEN  | PTD | DIS | M0)) /*SDRC_DQS0*/\
	MUX_VAL(CP(SDRC_DQS1),      (IEN  | PTD | DIS | M0)) /*SDRC_DQS1*/\
	MUX_VAL(CP(SDRC_DQS2),      (IEN  | PTD | DIS | M0)) /*SDRC_DQS2*/\
	MUX_VAL(CP(SDRC_DQS3),      (IEN  | PTD | DIS | M0)) /*SDRC_DQS3*/\
	/*GPMC*/\
	MUX_VAL(CP(GPMC_A1),        (IDIS | PTD | DIS | M0)) /*GPMC_A1*/\
	MUX_VAL(CP(GPMC_A2),        (IDIS | PTD | DIS | M0)) /*GPMC_A2*/\
	MUX_VAL(CP(GPMC_A3),        (IDIS | PTD | DIS | M0)) /*GPMC_A3*/\
	MUX_VAL(CP(GPMC_A4),        (IDIS | PTD | DIS | M0)) /*GPMC_A4*/\
	MUX_VAL(CP(GPMC_A5),        (IDIS | PTD | DIS | M0)) /*GPMC_A5*/\
	MUX_VAL(CP(GPMC_A6),        (IDIS | PTD | DIS | M0)) /*GPMC_A6*/\
	MUX_VAL(CP(GPMC_A7),        (IDIS | PTD | DIS | M0)) /*GPMC_A7*/\
	MUX_VAL(CP(GPMC_A8),        (IDIS | PTD | DIS | M0)) /*GPMC_A8*/\
	MUX_VAL(CP(GPMC_A9),        (IDIS | PTD | DIS | M0)) /*GPMC_A9*/\
	MUX_VAL(CP(GPMC_A10),       (IDIS | PTD | DIS | M0)) /*GPMC_A10*/\
	MUX_VAL(CP(GPMC_D0),        (IEN  | PTD | DIS | M0)) /*GPMC_D0*/\
	MUX_VAL(CP(GPMC_D1),        (IEN  | PTD | DIS | M0)) /*GPMC_D1*/\
	MUX_VAL(CP(GPMC_D2),        (IEN  | PTD | DIS | M0)) /*GPMC_D2*/\
	MUX_VAL(CP(GPMC_D3),        (IEN  | PTD | DIS | M0)) /*GPMC_D3*/\
	MUX_VAL(CP(GPMC_D4),        (IEN  | PTD | DIS | M0)) /*GPMC_D4*/\
	MUX_VAL(CP(GPMC_D5),        (IEN  | PTD | DIS | M0)) /*GPMC_D5*/\
	MUX_VAL(CP(GPMC_D6),        (IEN  | PTD | DIS | M0)) /*GPMC_D6*/\
	MUX_VAL(CP(GPMC_D7),        (IEN  | PTD | DIS | M0)) /*GPMC_D7*/\
	MUX_VAL(CP(GPMC_D8),        (IEN  | PTD | DIS | M0)) /*GPMC_D8*/\
	MUX_VAL(CP(GPMC_D9),        (IEN  | PTD | DIS | M0)) /*GPMC_D9*/\
	MUX_VAL(CP(GPMC_D10),       (IEN  | PTD | DIS | M0)) /*GPMC_D10*/\
	MUX_VAL(CP(GPMC_D11),       (IEN  | PTD | DIS | M0)) /*GPMC_D11*/\
	MUX_VAL(CP(GPMC_D12),       (IEN  | PTD | DIS | M0)) /*GPMC_D12*/\
	MUX_VAL(CP(GPMC_D13),       (IEN  | PTD | DIS | M0)) /*GPMC_D13*/\
	MUX_VAL(CP(GPMC_D14),       (IEN  | PTD | DIS | M0)) /*GPMC_D14*/\
	MUX_VAL(CP(GPMC_D15),       (IEN  | PTD | DIS | M0)) /*GPMC_D15*/\
	MUX_VAL(CP(GPMC_nCS0),      (IDIS | PTU | EN  | M0)) /*GPMC_nCS0*/\
	MUX_VAL(CP(GPMC_nCS1),      (IDIS | PTU | EN  | M0)) /*GPMC_nCS1*/\
	MUX_VAL(CP(GPMC_nCS2),      (IDIS | PTU | EN  | M0)) /*GPMC_nCS2*/\
	MUX_VAL(CP(GPMC_nCS3),      (IDIS | PTU | EN  | M0)) /*GPMC_nCS3*/\
	MUX_VAL(CP(GPMC_nCS4),      (IDIS | PTU | EN  | M0))\
	MUX_VAL(CP(GPMC_nCS5),      (IDIS | PTD | DIS | M0))\
	MUX_VAL(CP(GPMC_nCS6),      (IEN  | PTD | DIS | M1))\
	MUX_VAL(CP(GPMC_nCS7),      (IEN  | PTU | EN  | M1))\
	MUX_VAL(CP(GPMC_nBE1),      (IEN  | PTD | DIS | M0))\
	MUX_VAL(CP(GPMC_WAIT2),     (IEN  | PTU | EN  | M0))\
	MUX_VAL(CP(GPMC_WAIT3),     (IEN  | PTU | EN  | M0))\
	MUX_VAL(CP(GPMC_CLK),       (IDIS | PTD | DIS | M0)) /*GPMC_CLK*/\
	MUX_VAL(CP(GPMC_nADV_ALE),  (IDIS | PTD | DIS | M0)) /*GPMC_nADV_ALE*/\
	MUX_VAL(CP(GPMC_nOE),       (IDIS | PTD | DIS | M0)) /*GPMC_nOE*/\
	MUX_VAL(CP(GPMC_nWE),       (IDIS | PTD | DIS | M0)) /*GPMC_nWE*/\
	MUX_VAL(CP(GPMC_nBE0_CLE),  (IDIS | PTD | DIS | M0)) /*GPMC_nBE0_CLE*/\
	MUX_VAL(CP(GPMC_nWP),       (IEN  | PTD | DIS | M0)) /*GPMC_nWP*/\
	MUX_VAL(CP(GPMC_WAIT0),     (IEN  | PTU | EN  | M0)) /*GPMC_WAIT0*/\
	MUX_VAL(CP(GPMC_WAIT1),     (IEN  | PTU | EN  | M0)) /*GPMC_WAIT1*/\
	/*LCD Interface*/\
	MUX_VAL(CP(DSS_PCLK),       (IDIS | PTD | DIS | M0)) /*DSS_PCLK*/\
	MUX_VAL(CP(DSS_HSYNC),      (IDIS | PTD | DIS | M0)) /*DSS_HSYNC*/\
	MUX_VAL(CP(DSS_VSYNC),      (IDIS | PTD | DIS | M0)) /*DSS_VSYNC*/\
	MUX_VAL(CP(DSS_ACBIAS),     (IDIS | PTD | DIS | M0)) /*DSS_ACBIAS*/\
	MUX_VAL(CP(DSS_DATA0),      (IDIS | PTD | DIS | M0)) /*DSS_DATA0*/\
	MUX_VAL(CP(DSS_DATA1),      (IDIS | PTD | DIS | M0)) /*DSS_DATA1*/\
	MUX_VAL(CP(DSS_DATA2),      (IDIS | PTD | DIS | M0)) /*DSS_DATA2*/\
	MUX_VAL(CP(DSS_DATA3),      (IDIS | PTD | DIS | M0)) /*DSS_DATA3*/\
	MUX_VAL(CP(DSS_DATA4),      (IDIS | PTD | DIS | M0)) /*DSS_DATA4*/\
	MUX_VAL(CP(DSS_DATA5),      (IDIS | PTD | DIS | M0)) /*DSS_DATA5*/\
	MUX_VAL(CP(DSS_DATA6),      (IDIS | PTD | DIS | M0)) /*DSS_DATA6*/\
	MUX_VAL(CP(DSS_DATA7),      (IDIS | PTD | DIS | M0)) /*DSS_DATA7*/\
	MUX_VAL(CP(DSS_DATA8),      (IDIS | PTD | DIS | M0)) /*DSS_DATA8*/\
	MUX_VAL(CP(DSS_DATA9),      (IDIS | PTD | DIS | M0)) /*DSS_DATA9*/\
	MUX_VAL(CP(DSS_DATA10),     (IDIS | PTD | DIS | M0)) /*DSS_DATA10*/\
	MUX_VAL(CP(DSS_DATA11),     (IDIS | PTD | DIS | M0)) /*DSS_DATA11*/\
	MUX_VAL(CP(DSS_DATA12),     (IDIS | PTD | DIS | M0)) /*DSS_DATA12*/\
	MUX_VAL(CP(DSS_DATA13),     (IDIS | PTD | DIS | M0)) /*DSS_DATA13*/\
	MUX_VAL(CP(DSS_DATA14),     (IDIS | PTD | DIS | M0)) /*DSS_DATA14*/\
	MUX_VAL(CP(DSS_DATA15),     (IDIS | PTD | DIS | M0)) /*DSS_DATA15*/\
	MUX_VAL(CP(DSS_DATA16),     (IDIS | PTD | DIS | M0)) /*DSS_DATA16*/\
	MUX_VAL(CP(DSS_DATA17),     (IDIS | PTD | DIS | M0)) /*DSS_DATA17*/\
	MUX_VAL(CP(DSS_DATA18),     (IDIS | PTD | DIS | M0)) /*DSS_DATA18*/\
	MUX_VAL(CP(DSS_DATA19),     (IDIS | PTD | DIS | M0)) /*DSS_DATA19*/\
	MUX_VAL(CP(DSS_DATA20),     (IDIS | PTD | DIS | M0)) /*DSS_DATA20*/\
	MUX_VAL(CP(DSS_DATA21),     (IDIS | PTD | DIS | M0)) /*DSS_DATA21*/\
	MUX_VAL(CP(DSS_DATA22),     (IDIS | PTD | DIS | M0)) /*DSS_DATA22*/\
	MUX_VAL(CP(DSS_DATA23),     (IDIS | PTD | DIS | M0)) /*DSS_DATA23*/\
	/*GPIO based game buttons*/\
	MUX_VAL(CP(CAM_HS ),        (IEN  | PTU | EN  | M4)) /*GPIO_94, SPARE_GPIO0*/\
	MUX_VAL(CP(CAM_VS ),        (IEN  | PTU | EN  | M4)) /*GPIO_95, SPARE_GPIO1*/\
	MUX_VAL(CP(CAM_XCLKA),      (IEN  | PTU | EN  | M4)) /*GPIO_96, GAME_DPAD_LEFT*/\
	MUX_VAL(CP(CAM_PCLK),       (IEN  | PTU | EN  | M4)) /*GPIO_97, GAME_L2_SHOULDER*/\
	MUX_VAL(CP(CAM_FLD),        (IEN  | PTU | EN  | M4)) /*GPIO_98, GAME_DPAD_RIGHT*/\
	MUX_VAL(CP(CAM_D0 ),        (IEN  | PTU | DIS | M4)) /*GPIO_99, GAME_MENU*/\
	MUX_VAL(CP(CAM_D1 ),        (IEN  | PTU | DIS | M4)) /*GPIO_100, GAME_START*/\
	MUX_VAL(CP(CAM_D2 ),        (IEN  | PTU | EN  | M4)) /*GPIO_101, GAME_BUTTON_Y*/\
	MUX_VAL(CP(CAM_D3 ),        (IEN  | PTU | EN  | M4)) /*GPIO_102, GAME_L1_SHOULDER*/\
	MUX_VAL(CP(CAM_D4 ),        (IEN  | PTU | EN  | M4)) /*GPIO_103, GAME_DPAD_DOWN*/\
	MUX_VAL(CP(CAM_D5 ),        (IEN  | PTU | EN  | M4)) /*GPIO_104, GAME_SELECT*/\
	MUX_VAL(CP(CAM_D6 ),        (IEN  | PTU | DIS | M4)) /*GPIO_105, GAME_R1_SHOULDER*/\
	MUX_VAL(CP(CAM_D7 ),        (IEN  | PTU | DIS | M4)) /*GPIO_106, GAME_B*/\
	MUX_VAL(CP(CAM_D8 ),        (IEN  | PTU | DIS | M4)) /*GPIO_107, GAME_R2_SHOULDER*/\
	MUX_VAL(CP(CAM_D9 ),        (IEN  | PTU | DIS | M4)) /*GPIO_108, LID_SWITCH*/\
	MUX_VAL(CP(CAM_D10),        (IEN  | PTU | EN  | M4)) /*GPIO_109, GAME_X*/\
	MUX_VAL(CP(CAM_D11),        (IEN  | PTU | EN  | M4)) /*GPIO_110, GAME_DPAD_UP*/\
	MUX_VAL(CP(CAM_XCLKB),      (IEN  | PTU | EN  | M4)) /*GPIO_111, GAME_A*/\
	MUX_VAL(CP(CAM_WEN),        (IEN  | PTU | EN  | M4)) /*GPIO_167, SPARE_GPIO3*/\
	MUX_VAL(CP(CAM_STROBE),     (IEN  | PTU | EN  | M4)) /*GPIO_126, SPARE_GPIO2*/\
	/*UNCONNECTED PINS*/\
	MUX_VAL(CP(CSI2_DX0),       (IEN  | PTD | DIS | M0)) /*CSI2_DX0, UNCONNECTED*/\
	MUX_VAL(CP(CSI2_DY0),       (IEN  | PTD | DIS | M0)) /*CSI2_DY0, UNCONNECTED*/\
	MUX_VAL(CP(CSI2_DX1),       (IEN  | PTD | DIS | M0)) /*CSI2_DX1, UNCONNECTED*/\
	MUX_VAL(CP(CSI2_DY1),       (IEN  | PTD | DIS | M0)) /*CSI2_DY1, UNCONNECTED*/\
	MUX_VAL(CP(HDQ_SIO),        (IEN  | PTU | EN  | M0)) /*HDQ_SIO, NOT CONNECTED*/\
	MUX_VAL(CP(I2C2_SCL),       (IEN  | PTU | EN  | M0)) /*I2C2_SCL, UNCONNECTED*/\
	MUX_VAL(CP(I2C2_SDA),       (IEN  | PTU | EN  | M0)) /*I2C2_SDA, UNCONNECTED*/\
	/*Audio Interface To Triton2*/\
	MUX_VAL(CP(McBSP2_FSX),     (IDIS | PTD | DIS | M0)) /*McBSP2_FSX*/\
	MUX_VAL(CP(McBSP2_CLKX),    (IDIS | PTD | DIS | M0)) /*McBSP2_CLKX*/\
	MUX_VAL(CP(McBSP2_DR),      (IEN  | PTD | DIS | M0)) /*McBSP2_DR*/\
	MUX_VAL(CP(McBSP2_DX),      (IDIS | PTD | DIS | M0)) /*McBSP2_DX*/\
	MUX_VAL(CP(McBSP_CLKS),     (IEN  | PTU | DIS | M0)) /*McBSP_CLKS  */\
	/*Expansion card 1*/\
	MUX_VAL(CP(MMC1_CLK),       (IDIS | PTU | EN  | M0)) /*MMC1_CLK*/\
	MUX_VAL(CP(MMC1_CMD),       (IEN  | PTU | EN  | M0)) /*MMC1_CMD*/\
	MUX_VAL(CP(MMC1_DAT0),      (IEN  | PTU | EN  | M0)) /*MMC1_DAT0*/\
	MUX_VAL(CP(MMC1_DAT1),      (IEN  | PTU | EN  | M0)) /*MMC1_DAT1*/\
	MUX_VAL(CP(MMC1_DAT2),      (IEN  | PTU | EN  | M0)) /*MMC1_DAT2*/\
	MUX_VAL(CP(MMC1_DAT3),      (IEN  | PTU | EN  | M0)) /*MMC1_DAT3*/\
	MUX_VAL(CP(MMC1_DAT4),      (IEN  | PTD | DIS | M4)) /*GPIO_126, MMC1_WP*/\
	MUX_VAL(CP(MMC1_DAT5),      (IEN  | PTD | DIS | M4)) /*GPIO_127, MMC2_WP*/\
	MUX_VAL(CP(MMC1_DAT6),      (IDIS | PTD | DIS | M4)) /*GPIO_128, MMC1_ACTIVITY*/\
	MUX_VAL(CP(MMC1_DAT7),      (IDIS | PTD | DIS | M4)) /*GPIO_129, MMC2_ACTIVITY*/\
	/*Expansion card 2*/\
	MUX_VAL(CP(MMC2_CLK),       (IDIS | PTD | DIS | M0)) /*MMC2_CLK*/\
	MUX_VAL(CP(MMC2_CMD),       (IEN  | PTU | EN  | M0)) /*MMC2_CMD*/\
	MUX_VAL(CP(MMC2_DAT0),      (IEN  | PTU | EN  | M0)) /*MMC2_DAT0*/\
	MUX_VAL(CP(MMC2_DAT1),      (IEN  | PTU | EN  | M0)) /*MMC2_DAT1*/\
	MUX_VAL(CP(MMC2_DAT2),      (IEN  | PTU | EN  | M0)) /*MMC2_DAT2*/\
	MUX_VAL(CP(MMC2_DAT3),      (IEN  | PTU | EN  | M0)) /*MMC2_DAT3*/\
	MUX_VAL(CP(MMC2_DAT4),      (IDIS | PTD | DIS | M1)) /*MMC2_DIR_DAT0*/\
	MUX_VAL(CP(MMC2_DAT5),      (IDIS | PTD | DIS | M1)) /*MMC2_DIR_DAT1*/\
	MUX_VAL(CP(MMC2_DAT6),      (IDIS | PTD | DIS | M1)) /*MMC2_DIR_CMD */\
	MUX_VAL(CP(MMC2_DAT7),      (IEN  | PTU | EN  | M1)) /*MMC2_CLKIN*/\
	/*Audio Interface To External DAC (Headphone, Speakers)*/\
	MUX_VAL(CP(McBSP3_DX),      (IDIS | PTD | DIS | M0)) /*McBSP3_DX*/\
	MUX_VAL(CP(McBSP3_CLKX),    (IDIS | PTD | DIS | M0)) /*McBSP3_CLKX  */\
	MUX_VAL(CP(McBSP3_FSX),     (IDIS | PTD | DIS | M0)) /*McBSP3_FSX*/\
	/*Future interface to Bluetooth or other usage (EXP CONN)*/\
	MUX_VAL(CP(UART1_TX),       (IDIS | PTD | DIS | M0)) /*UART1_TX*/\
	MUX_VAL(CP(UART1_RTS),      (IEN  | PTD | DIS | M0)) /*UART1_RTS*/\
	MUX_VAL(CP(UART1_CTS),      (IEN  | PTD | DIS | M0)) /*UART1_CTS*/\
	MUX_VAL(CP(UART1_RX),       (IEN  | PTD | DIS | M0)) /*UART1_RX*/\
	MUX_VAL(CP(McBSP4_CLKX),    (IDIS | PTD | DIS | M0)) /*McBSP4_CLKX*/\
	MUX_VAL(CP(McBSP4_DR),      (IEN  | PTD | DIS | M0)) /*McBSP4_DR*/\
	MUX_VAL(CP(McBSP4_DX),      (IDIS | PTD | DIS | M0)) /*McBSP4_DX*/\
	MUX_VAL(CP(McBSP4_FSX),     (IDIS | PTD | DIS | M0)) /*McBSP4_FSX*/\
	/*GPIO definitions for muxed pins*/\
	MUX_VAL(CP(McBSP3_DR),      (IDIS | PTD | DIS | M4)) /*GPIO_141, nPOWERDOWN_DAC*/\
	MUX_VAL(CP(UART2_CTS),      (IEN  | PTU | EN  | M4)) /*GPIO_144, nTOUCH_IRQ*/\
	MUX_VAL(CP(UART2_RTS),      (IDIS | PTU | DIS | M4)) /*GPIO_145, nNUB_RESET*/\
	MUX_VAL(CP(UART2_TX),       (IEN  | PTU | EN  | M4)) /*GPIO_146, nTOUCH_BUSY*/\
	MUX_VAL(CP(UART2_RX),       (IDIS | PTD | DIS | M4)) /*GPIO_147, CS_USB_HOST*/\
	MUX_VAL(CP(McBSP1_CLKX),    (IDIS | PTD | DIS | M4)) /*GPIO_162, START_ADC*/\
	MUX_VAL(CP(UART3_CTS_RCTX), (IEN  | PTD | DIS | M4)) /*GPIO_163, nOC_USB5*/\
	MUX_VAL(CP(UART3_RTS_SD),   (IDIS | PTD | DIS | M4)) /*GPIO_164, EN_USB_5V*/\
	MUX_VAL(CP(ETK_D0_ES2 ),    (IDIS | PTD | DIS | M4)) /*GPIO_14, nHEADPHONE_SHUTDOWN*/\
	MUX_VAL(CP(ETK_D1_ES2 ),    (IDIS | PTD | DIS | M4)) /*GPIO_15, WIFI_POWER_ENABLE*/\
	MUX_VAL(CP(ETK_D2_ES2 ),    (IEN  | PTD | DIS | M4)) /*GPIO_16, MOTION_NUB2*/\
	MUX_VAL(CP(ETK_D7_ES2 ),    (IEN  | PTD | DIS | M4)) /*GPIO_21, MOTION_NUB1*/\
	MUX_VAL(CP(ETK_D8_ES2 ),    (IEN  | PTD | DIS | M4)) /*GPIO_22, MSECURE*/\
	MUX_VAL(CP(ETK_D9_ES2 ),    (IDIS | PTD | DIS | M4)) /*GPIO_23, WIFI_RESET*/\
	MUX_VAL(CP(McBSP1_FSR),     (IDIS | PTU | EN  | M4)) /*GPIO_157, LCD_RESET*/\
	MUX_VAL(CP(McBSP1_DR),      (IDIS | PTD | DIS | M4)) /*GPIO_159, LCD_STANDBY*/\
	/*LCD Control Interface*/\
	MUX_VAL(CP(McBSP1_CLKR),    (IDIS | PTD | DIS | M1)) /*McSPI4_CLK, SPI_LCD_CLK*/\
	MUX_VAL(CP(McBSP1_DX),      (IDIS | PTD | DIS | M1)) /*McSPI4_SIMO, SPI_LCD_MOSI*/\
	MUX_VAL(CP(McBSP1_FSX),     (IDIS | PTD | DIS | M1)) /*McSPI4_CS0, SPI_LCD_CS0*/\
	/*Serial Interface (Peripheral boot, Linux console)*/\
	MUX_VAL(CP(UART3_RX_IRRX ), (IEN  | PTD | DIS | M0)) /*UART3_RX*/\
	MUX_VAL(CP(UART3_TX_IRTX ), (IDIS | PTD | DIS | M0)) /*UART3_TX*/\
	/*HS USB OTG Port (connects to HSUSB0)*/\
	MUX_VAL(CP(HSUSB0_CLK),     (IEN  | PTD | DIS | M0)) /*HSUSB0_CLK*/\
	MUX_VAL(CP(HSUSB0_STP),     (IDIS | PTU | EN  | M0)) /*HSUSB0_STP*/\
	MUX_VAL(CP(HSUSB0_DIR),     (IEN  | PTD | DIS | M0)) /*HSUSB0_DIR*/\
	MUX_VAL(CP(HSUSB0_NXT),     (IEN  | PTD | DIS | M0)) /*HSUSB0_NXT*/\
	MUX_VAL(CP(HSUSB0_DATA0),   (IEN  | PTD | DIS | M0)) /*HSUSB0_DATA0 */\
	MUX_VAL(CP(HSUSB0_DATA1),   (IEN  | PTD | DIS | M0)) /*HSUSB0_DATA1 */\
	MUX_VAL(CP(HSUSB0_DATA2),   (IEN  | PTD | DIS | M0)) /*HSUSB0_DATA2 */\
	MUX_VAL(CP(HSUSB0_DATA3),   (IEN  | PTD | DIS | M0)) /*HSUSB0_DATA3 */\
	MUX_VAL(CP(HSUSB0_DATA4),   (IEN  | PTD | DIS | M0)) /*HSUSB0_DATA4 */\
	MUX_VAL(CP(HSUSB0_DATA5),   (IEN  | PTD | DIS | M0)) /*HSUSB0_DATA5 */\
	MUX_VAL(CP(HSUSB0_DATA6),   (IEN  | PTD | DIS | M0)) /*HSUSB0_DATA6 */\
	MUX_VAL(CP(HSUSB0_DATA7),   (IEN  | PTD | DIS | M0)) /*HSUSB0_DATA7 */\
	/*I2C Ports*/\
	MUX_VAL(CP(I2C1_SCL),       (IEN  | PTU | EN  | M0)) /*I2C1_SCL*/\
	MUX_VAL(CP(I2C1_SDA),       (IEN  | PTU | EN  | M0)) /*I2C1_SDA*/\
	MUX_VAL(CP(I2C3_SCL),       (IEN  | PTU | EN  | M0)) /*I2C3_SCL*/\
	MUX_VAL(CP(I2C3_SDA),       (IEN  | PTU | EN  | M0)) /*I2C3_SDA*/\
	MUX_VAL(CP(I2C4_SCL),       (IEN  | PTU | EN  | M0)) /*I2C4_SCL*/\
	MUX_VAL(CP(I2C4_SDA),       (IEN  | PTU | EN  | M0)) /*I2C4_SDA*/\
	/*Serial Interface (Touch, Analog Nubs)*/\
	MUX_VAL(CP(McSPI1_CLK),     (IDIS | PTD | DIS | M0)) /*McSPI1_CLK*/\
	MUX_VAL(CP(McSPI1_SIMO),    (IDIS | PTD | DIS | M0)) /*McSPI1_SIMO*/\
	MUX_VAL(CP(McSPI1_SOMI),    (IEN  | PTD | DIS | M0)) /*McSPI1_SOMI*/\
	MUX_VAL(CP(McSPI1_CS0),     (IDIS | PTU | EN  | M0)) /*McSPI1_CS0, TOUCH*/\
	MUX_VAL(CP(McSPI1_CS1),     (IDIS | PTU | EN  | M0)) /*McSPI1_CS1, NUB_LEFT*/\
	MUX_VAL(CP(McSPI1_CS2),     (IDIS | PTU | EN  | M0)) /*McSPI1_CS2, NUB_RIGHT*/\
	/*HS USB HOST Port (connects to HSUSB2)*/\
	MUX_VAL(CP(ETK_D10_ES2),    (IDIS | PTD | DIS | M3)) /*USB_HOST_CLK*/\
	MUX_VAL(CP(ETK_D11_ES2),    (IDIS | PTU | EN  | M3)) /*USB_HOST_STP*/\
	MUX_VAL(CP(ETK_D12_ES2),    (IEN  | PTD | DIS | M3)) /*USB_HOST_DIR*/\
	MUX_VAL(CP(ETK_D13_ES2),    (IEN  | PTD | DIS | M3)) /*USB_HOST_NXT*/\
	MUX_VAL(CP(ETK_D14_ES2),    (IEN  | PTD | DIS | M3)) /*USB_HOST_D0*/\
	MUX_VAL(CP(ETK_D15_ES2),    (IEN  | PTD | DIS | M3)) /*USB_HOST_D1*/\
	MUX_VAL(CP(McSPI1_CS3),     (IEN  | PTD | DIS | M3)) /*USB_HOST_D2*/\
	MUX_VAL(CP(McSPI2_CS1),     (IEN  | PTD | DIS | M3)) /*USB_HOST_D3*/\
	MUX_VAL(CP(McSPI2_SIMO),    (IEN  | PTD | DIS | M3)) /*USB_HOST_D4*/\
	MUX_VAL(CP(McSPI2_SOMI),    (IEN  | PTD | DIS | M3)) /*USB_HOST_D5*/\
	MUX_VAL(CP(McSPI2_CS0),     (IEN  | PTD | DIS | M3)) /*USB_HOST_D6*/\
	MUX_VAL(CP(McSPI2_CLK),     (IEN  | PTD | DIS | M3)) /*USB_HOST_D7*/\
	/*Control and debug (Not for GP usage)*/\
	MUX_VAL(CP(SYS_32K),        (IEN  | PTD | DIS | M0)) /*SYS_32K*/\
	MUX_VAL(CP(SYS_CLKREQ),     (IEN  | PTD | DIS | M0)) /*SYS_CLKREQ*/\
	MUX_VAL(CP(SYS_nIRQ),       (IEN  | PTU | EN  | M0)) /*SYS_nIRQ*/\
	MUX_VAL(CP(SYS_BOOT0),      (IEN  | PTD | DIS | M4)) /*GPIO_2, CANNOT BE USED (LEAVE INPUT)*/\
	MUX_VAL(CP(SYS_BOOT1),      (IEN  | PTD | DIS | M4)) /*GPIO_3, CANNOT BE USED (LEAVE INPUT)*/\
	MUX_VAL(CP(SYS_BOOT2),      (IEN  | PTD | DIS | M4)) /*GPIO_4, CANNOT BE USED (LEAVE INPUT)*/\
	MUX_VAL(CP(SYS_BOOT3),      (IEN  | PTD | DIS | M4)) /*GPIO_5, CANNOT BE USED (LEAVE INPUT)*/\
	MUX_VAL(CP(SYS_BOOT4),      (IEN  | PTD | DIS | M4)) /*GPIO_6, CANNOT BE USED (LEAVE INPUT)*/\
	MUX_VAL(CP(SYS_BOOT5),      (IEN  | PTD | DIS | M4)) /*GPIO_7, CANNOT BE USED (LEAVE INPUT)*/\
	MUX_VAL(CP(SYS_BOOT6),      (IEN  | PTD | DIS | M4)) /*GPIO_8, CANNOT BE USED (LEAVE INPUT)*/\
	MUX_VAL(CP(SYS_OFF_MODE),   (IEN  | PTD | DIS | M0)) /*SYS_OFF_MODE */\
	MUX_VAL(CP(SYS_CLKOUT1),    (IEN  | PTD | DIS | M4)) /*SYS_CLKOUT1, NOT_CONNECTED*/\
	MUX_VAL(CP(SYS_CLKOUT2),    (IEN  | PTD | DIS | M4)) /*SYS_CLKOUT2, NOT_CONNECTED*/\
	/*SDIO Interface to WIFI Module*/\
	MUX_VAL(CP(ETK_CLK_ES2),    (IDIS | PTU | EN  | M2)) /*MMC3_CLK*/\
	MUX_VAL(CP(ETK_CTL_ES2),    (IEN  | PTD | DIS | M2)) /*MMC3_CMD*/\
	MUX_VAL(CP(ETK_D4_ES2 ),    (IEN  | PTD | DIS | M2)) /*MMC3_DAT0*/\
	MUX_VAL(CP(ETK_D5_ES2 ),    (IEN  | PTD | DIS | M2)) /*MMC3_DAT1*/\
	MUX_VAL(CP(ETK_D6_ES2 ),    (IEN  | PTD | DIS | M2)) /*MMC3_DAT2*/\
	MUX_VAL(CP(ETK_D3_ES2 ),    (IEN  | PTD | DIS | M2)) /*MMC3_DAT3*/\
	/*JTAG*/\
	MUX_VAL(CP(JTAG_nTRST),     (IEN  | PTD | DIS | M0)) /*JTAG_nTRST*/\
	MUX_VAL(CP(JTAG_TCK),       (IEN  | PTD | DIS | M0)) /*JTAG_TCK*/\
	MUX_VAL(CP(JTAG_TMS),       (IEN  | PTD | DIS | M0)) /*JTAG_TMS*/\
	MUX_VAL(CP(JTAG_TDI),       (IEN  | PTD | DIS | M0)) /*JTAG_TDI*/\
	MUX_VAL(CP(JTAG_EMU0),      (IEN  | PTD | DIS | M0)) /*JTAG_EMU0*/\
	MUX_VAL(CP(JTAG_EMU1),      (IEN  | PTD | DIS | M0)) /*JTAG_EMU1*/\
	/*Die to Die stuff*/\
	MUX_VAL(CP(d2d_mcad1),      (IEN  | PTD | EN  | M0)) /*d2d_mcad1*/\
	MUX_VAL(CP(d2d_mcad2),      (IEN  | PTD | EN  | M0)) /*d2d_mcad2*/\
	MUX_VAL(CP(d2d_mcad3),      (IEN  | PTD | EN  | M0)) /*d2d_mcad3*/\
	MUX_VAL(CP(d2d_mcad4),      (IEN  | PTD | EN  | M0)) /*d2d_mcad4*/\
	MUX_VAL(CP(d2d_mcad5),      (IEN  | PTD | EN  | M0)) /*d2d_mcad5*/\
	MUX_VAL(CP(d2d_mcad6),      (IEN  | PTD | EN  | M0)) /*d2d_mcad6*/\
	MUX_VAL(CP(d2d_mcad7),      (IEN  | PTD | EN  | M0)) /*d2d_mcad7*/\
	MUX_VAL(CP(d2d_mcad8),      (IEN  | PTD | EN  | M0)) /*d2d_mcad8*/\
	MUX_VAL(CP(d2d_mcad9),      (IEN  | PTD | EN  | M0)) /*d2d_mcad9*/\
	MUX_VAL(CP(d2d_mcad10),     (IEN  | PTD | EN  | M0)) /*d2d_mcad10*/\
	MUX_VAL(CP(d2d_mcad11),     (IEN  | PTD | EN  | M0)) /*d2d_mcad11*/\
	MUX_VAL(CP(d2d_mcad12),     (IEN  | PTD | EN  | M0)) /*d2d_mcad12*/\
	MUX_VAL(CP(d2d_mcad13),     (IEN  | PTD | EN  | M0)) /*d2d_mcad13*/\
	MUX_VAL(CP(d2d_mcad14),     (IEN  | PTD | EN  | M0)) /*d2d_mcad14*/\
	MUX_VAL(CP(d2d_mcad15),     (IEN  | PTD | EN  | M0)) /*d2d_mcad15*/\
	MUX_VAL(CP(d2d_mcad16),     (IEN  | PTD | EN  | M0)) /*d2d_mcad16*/\
	MUX_VAL(CP(d2d_mcad17),     (IEN  | PTD | EN  | M0)) /*d2d_mcad17*/\
	MUX_VAL(CP(d2d_mcad18),     (IEN  | PTD | EN  | M0)) /*d2d_mcad18*/\
	MUX_VAL(CP(d2d_mcad19),     (IEN  | PTD | EN  | M0)) /*d2d_mcad19*/\
	MUX_VAL(CP(d2d_mcad20),     (IEN  | PTD | EN  | M0)) /*d2d_mcad20*/\
	MUX_VAL(CP(d2d_mcad21),     (IEN  | PTD | EN  | M0)) /*d2d_mcad21*/\
	MUX_VAL(CP(d2d_mcad22),     (IEN  | PTD | EN  | M0)) /*d2d_mcad22*/\
	MUX_VAL(CP(d2d_mcad23),     (IEN  | PTD | EN  | M0)) /*d2d_mcad23*/\
	MUX_VAL(CP(d2d_mcad24),     (IEN  | PTD | EN  | M0)) /*d2d_mcad24*/\
	MUX_VAL(CP(d2d_mcad25),     (IEN  | PTD | EN  | M0)) /*d2d_mcad25*/\
	MUX_VAL(CP(d2d_mcad26),     (IEN  | PTD | EN  | M0)) /*d2d_mcad26*/\
	MUX_VAL(CP(d2d_mcad27),     (IEN  | PTD | EN  | M0)) /*d2d_mcad27*/\
	MUX_VAL(CP(d2d_mcad28),     (IEN  | PTD | EN  | M0)) /*d2d_mcad28*/\
	MUX_VAL(CP(d2d_mcad29),     (IEN  | PTD | EN  | M0)) /*d2d_mcad29*/\
	MUX_VAL(CP(d2d_mcad30),     (IEN  | PTD | EN  | M0)) /*d2d_mcad30*/\
	MUX_VAL(CP(d2d_mcad31),     (IEN  | PTD | EN  | M0)) /*d2d_mcad31*/\
	MUX_VAL(CP(d2d_mcad32),     (IEN  | PTD | EN  | M0)) /*d2d_mcad32*/\
	MUX_VAL(CP(d2d_mcad33),     (IEN  | PTD | EN  | M0)) /*d2d_mcad33*/\
	MUX_VAL(CP(d2d_mcad34),     (IEN  | PTD | EN  | M0)) /*d2d_mcad34*/\
	MUX_VAL(CP(d2d_mcad35),     (IEN  | PTD | EN  | M0)) /*d2d_mcad35*/\
	MUX_VAL(CP(d2d_mcad36),     (IEN  | PTD | EN  | M0)) /*d2d_mcad36*/\
	MUX_VAL(CP(d2d_clk26mi),    (IEN  | PTD | DIS | M0)) /*d2d_clk26mi  */\
	MUX_VAL(CP(d2d_nrespwron ), (IEN  | PTD | EN  | M0)) /*d2d_nrespwron*/\
	MUX_VAL(CP(d2d_nreswarm),   (IEN  | PTU | EN  | M0)) /*d2d_nreswarm */\
	MUX_VAL(CP(d2d_arm9nirq),   (IEN  | PTD | DIS | M0)) /*d2d_arm9nirq */\
	MUX_VAL(CP(d2d_uma2p6fiq ), (IEN  | PTD | DIS | M0)) /*d2d_uma2p6fiq*/\
	MUX_VAL(CP(d2d_spint),      (IEN  | PTD | EN  | M0)) /*d2d_spint*/\
	MUX_VAL(CP(d2d_frint),      (IEN  | PTD | EN  | M0)) /*d2d_frint*/\
	MUX_VAL(CP(d2d_dmareq0),    (IEN  | PTD | DIS | M0)) /*d2d_dmareq0  */\
	MUX_VAL(CP(d2d_dmareq1),    (IEN  | PTD | DIS | M0)) /*d2d_dmareq1  */\
	MUX_VAL(CP(d2d_dmareq2),    (IEN  | PTD | DIS | M0)) /*d2d_dmareq2  */\
	MUX_VAL(CP(d2d_dmareq3),    (IEN  | PTD | DIS | M0)) /*d2d_dmareq3  */\
	MUX_VAL(CP(d2d_n3gtrst),    (IEN  | PTD | DIS | M0)) /*d2d_n3gtrst  */\
	MUX_VAL(CP(d2d_n3gtdi),     (IEN  | PTD | DIS | M0)) /*d2d_n3gtdi*/\
	MUX_VAL(CP(d2d_n3gtdo),     (IEN  | PTD | DIS | M0)) /*d2d_n3gtdo*/\
	MUX_VAL(CP(d2d_n3gtms),     (IEN  | PTD | DIS | M0)) /*d2d_n3gtms*/\
	MUX_VAL(CP(d2d_n3gtck),     (IEN  | PTD | DIS | M0)) /*d2d_n3gtck*/\
	MUX_VAL(CP(d2d_n3grtck),    (IEN  | PTD | DIS | M0)) /*d2d_n3grtck  */\
	MUX_VAL(CP(d2d_mstdby),     (IEN  | PTU | EN  | M0)) /*d2d_mstdby*/\
	MUX_VAL(CP(d2d_swakeup),    (IEN  | PTD | EN  | M0)) /*d2d_swakeup  */\
	MUX_VAL(CP(d2d_idlereq),    (IEN  | PTD | DIS | M0)) /*d2d_idlereq  */\
	MUX_VAL(CP(d2d_idleack),    (IEN  | PTU | EN  | M0)) /*d2d_idleack  */\
	MUX_VAL(CP(d2d_mwrite),     (IEN  | PTD | DIS | M0)) /*d2d_mwrite*/\
	MUX_VAL(CP(d2d_swrite),     (IEN  | PTD | DIS | M0)) /*d2d_swrite*/\
	MUX_VAL(CP(d2d_mread),      (IEN  | PTD | DIS | M0)) /*d2d_mread*/\
	MUX_VAL(CP(d2d_sread),      (IEN  | PTD | DIS | M0)) /*d2d_sread*/\
	MUX_VAL(CP(d2d_mbusflag),   (IEN  | PTD | DIS | M0)) /*d2d_mbusflag */\
	MUX_VAL(CP(d2d_sbusflag),   (IEN  | PTD | DIS | M0)) /*d2d_sbusflag */\
	MUX_VAL(CP(sdrc_cke0),      (IDIS | PTU | EN  | M0)) /*sdrc_cke0 */\
	MUX_VAL(CP(sdrc_cke1),      (IDIS | PTD | DIS | M7)) /*sdrc_cke1 not used*/

/**********************************************************
 * Routine: set_muxconf_regs
 * Description: Setting up the configuration Mux registers
 *              specific to the hardware. Many pins need
 *              to be moved from protect to primary mode.
 *********************************************************/
void set_muxconf_regs(void)
{
	MUX_DEFAULT();
}

/**********************************************************
 * Routine: nand+_init
 * Description: Set up nand for nand and jffs2 commands
 *********************************************************/
int nand_init(void)
{
	/* global settings */
	__raw_writel(0x10, GPMC_SYSCONFIG);	/* smart idle */
	__raw_writel(0x0, GPMC_IRQENABLE);	/* isr's sources masked */
	__raw_writel(0, GPMC_TIMEOUT_CONTROL);	/* timeout disable */
#ifdef CFG_NAND
	__raw_writel(0x001, GPMC_CONFIG);	/* set nWP, disable limited addr */
#endif

	/* Set the GPMC Vals, NAND is mapped at CS0
	 *  We configure only GPMC CS0 with required values. Configuring other devices
	 *  at other CS is done in u-boot. So we don't have to bother doing it here.
	 */
	__raw_writel(0, GPMC_CONFIG7 + GPMC_CONFIG_CS0);
	delay(1000);

	if ((get_mem_type() == GPMC_NAND) || (get_mem_type() == MMC_NAND)) {
		__raw_writel(M_NAND_GPMC_CONFIG1, GPMC_CONFIG1 + GPMC_CONFIG_CS0);
		__raw_writel(M_NAND_GPMC_CONFIG2, GPMC_CONFIG2 + GPMC_CONFIG_CS0);
		__raw_writel(M_NAND_GPMC_CONFIG3, GPMC_CONFIG3 + GPMC_CONFIG_CS0);
		__raw_writel(M_NAND_GPMC_CONFIG4, GPMC_CONFIG4 + GPMC_CONFIG_CS0);
		__raw_writel(M_NAND_GPMC_CONFIG5, GPMC_CONFIG5 + GPMC_CONFIG_CS0);
		__raw_writel(M_NAND_GPMC_CONFIG6, GPMC_CONFIG6 + GPMC_CONFIG_CS0);

		/* Enable the GPMC Mapping */
		__raw_writel((((OMAP34XX_GPMC_CS0_SIZE & 0xF)<<8) |
			     ((NAND_BASE_ADR>>24) & 0x3F) |
			     (1<<6)),  (GPMC_CONFIG7 + GPMC_CONFIG_CS0));
		delay(2000);

		if (nand_chip()) {
#ifdef CFG_PRINTF
			printf("Unsupported Chip!\n");
#endif
			return 1;
		}

	}

	if ((get_mem_type() == GPMC_ONENAND) || (get_mem_type() == MMC_ONENAND)) {
		__raw_writel(ONENAND_GPMC_CONFIG1, GPMC_CONFIG1 + GPMC_CONFIG_CS0);
		__raw_writel(ONENAND_GPMC_CONFIG2, GPMC_CONFIG2 + GPMC_CONFIG_CS0);
		__raw_writel(ONENAND_GPMC_CONFIG3, GPMC_CONFIG3 + GPMC_CONFIG_CS0);
		__raw_writel(ONENAND_GPMC_CONFIG4, GPMC_CONFIG4 + GPMC_CONFIG_CS0);
		__raw_writel(ONENAND_GPMC_CONFIG5, GPMC_CONFIG5 + GPMC_CONFIG_CS0);
		__raw_writel(ONENAND_GPMC_CONFIG6, GPMC_CONFIG6 + GPMC_CONFIG_CS0);

		/* Enable the GPMC Mapping */
		__raw_writel((((OMAP34XX_GPMC_CS0_SIZE & 0xF)<<8) |
			     ((ONENAND_BASE>>24) & 0x3F) |
			     (1<<6)),  (GPMC_CONFIG7 + GPMC_CONFIG_CS0));
		delay(2000);

		if (onenand_chip()) {
#ifdef CFG_PRINTF
			printf("OneNAND Unsupported !\n");
#endif
			return 1;
		}
	}
	return 0;
}

#define DEBUG_SD1_LED			128	/* gpio - SD Slot 1 */
#define DEBUG_SD2_LED			129	/* gpio - SD Slot 2 */

void blinkLEDs()
{
	void *p;

	/* Alternately turn the LEDs on and off */
	p = (unsigned long *)OMAP34XX_GPIO5_BASE;
	while (1) {
		/* turn LED1 on and LED2 off */
		*(unsigned long *)(p + 0x94) = 1 << (DEBUG_SD1_LED % 32);
		*(unsigned long *)(p + 0x90) = 1 << (DEBUG_SD2_LED % 32);

		/* delay for a while */
		delay(1000);

		/* turn LED1 off and LED2 on */
		*(unsigned long *)(p + 0x90) = 1 << (DEBUG_SD1_LED % 32);
		*(unsigned long *)(p + 0x94) = 1 << (DEBUG_SD2_LED % 32);

		/* delay for a while */
		delay(1000);
	}
}

/* optionally do something like blinking LED */
void board_hang(void)
{
	while (1)
		blinkLEDs();
}

/******************************************************************************
 * Dummy function to handle errors for EABI incompatibility
 *****************************************************************************/
void raise(void)
{
}

/******************************************************************************
 * Dummy function to handle errors for EABI incompatibility
 *****************************************************************************/
void abort(void)
{
}
