/*
 * (C) Copyright 2004-2006
 * Texas Instruments, <www.ti.com>
 * Richard Woodruff <r-woodruff2@ti.com>
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
#include <asm/arch/cpu.h>
#include <asm/io.h>
#include <asm/arch/bits.h>
#include <asm/arch/mux.h>
#include <asm/arch/sys_proto.h>
#include <asm/arch/sys_info.h>
#include <asm/arch/clocks.h>
#include <asm/arch/mem.h>
#include <i2c.h>
#include <asm/mach-types.h>
#if (CONFIG_COMMANDS & CFG_CMD_NAND) && defined(CFG_NAND_LEGACY)
#include <linux/mtd/nand_legacy.h>
extern struct nand_chip nand_dev_desc[CFG_MAX_NAND_DEVICE];
#endif

/*******************************************************
 * Routine: delay
 * Description: spinning delay to use before udelay works
 ******************************************************/
static inline void delay(unsigned long loops)
{
	__asm__ volatile ("1:\n" "subs %0, %1, #1\n"
			  "bne 1b":"=r" (loops):"0"(loops));
}

/*****************************************
 * Routine: board_init
 * Description: Early hardware init.
 *****************************************/
int board_init(void)
{
	DECLARE_GLOBAL_DATA_PTR;

	gpmc_init();		/* in SRAM or SDRAM, finish GPMC */
	gd->bd->bi_arch_number = MACH_TYPE_OMAP3_PANDORA; /* board id for Linux */
	gd->bd->bi_boot_params = (OMAP34XX_SDRC_CS0 + 0x100); /* boot param addr */

	return 0;
}

/*****************************************
 * Routine: secure_unlock
 * Description: Setup security registers for access 
 * (GP Device only)
 *****************************************/
void secure_unlock_mem(void)
{
	/* Permission values for registers -Full fledged permissions to all */
	#define UNLOCK_1 0xFFFFFFFF
	#define UNLOCK_2 0x00000000
	#define UNLOCK_3 0x0000FFFF
	/* Protection Module Register Target APE (PM_RT)*/
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

	__raw_writel(UNLOCK_1, SMS_RG_ATT0); /* SDRC region 0 public */
}


/**********************************************************
 * Routine: secureworld_exit()
 * Description: If chip is EMU and boot type is external
 *		configure secure registers and exit secure world
 *  general use.
 ***********************************************************/
void secureworld_exit()
{
	unsigned long i;

	/* configrue non-secure access control register */
	__asm__ __volatile__("mrc p15, 0, %0, c1, c1, 2":"=r" (i));
	/* enabling co-processor CP10 and CP11 accesses in NS world */
	__asm__ __volatile__("orr %0, %0, #0xC00":"=r"(i));
	/* allow allocation of locked TLBs and L2 lines in NS world */
	/* allow use of PLE registers in NS world also */
	__asm__ __volatile__("orr %0, %0, #0x70000":"=r"(i));
	__asm__ __volatile__("mcr p15, 0, %0, c1, c1, 2":"=r" (i));

	/* Enable ASA in ACR register */
	__asm__ __volatile__("mrc p15, 0, %0, c1, c0, 1":"=r" (i));
	__asm__ __volatile__("orr %0, %0, #0x10":"=r"(i));
	__asm__ __volatile__("mcr p15, 0, %0, c1, c0, 1":"=r" (i));

	/* Exiting secure world */
	__asm__ __volatile__("mrc p15, 0, %0, c1, c1, 0":"=r" (i));
	__asm__ __volatile__("orr %0, %0, #0x31":"=r"(i));
	__asm__ __volatile__("mcr p15, 0, %0, c1, c1, 0":"=r" (i));
}

/**********************************************************
 * Routine: setup_auxcr()
 * Description: Write to AuxCR desired value using SMI.
 *  general use.
 ***********************************************************/
void setup_auxcr()
{
	unsigned long i;
	volatile unsigned int j;
	/* Save r0, r12 and restore them after usage */
	__asm__ __volatile__("mov %0, r12":"=r" (j));
	__asm__ __volatile__("mov %0, r0":"=r" (i));

	/* GP Device ROM code API usage here */
	/* r12 = AUXCR Write function and r0 value */
	__asm__ __volatile__("mov r12, #0x3");
	__asm__ __volatile__("mrc p15, 0, r0, c1, c0, 1");
	/* Enabling ASA */
	__asm__ __volatile__("orr r0, r0, #0x10");
	/* SMI instruction to call ROM Code API */
	__asm__ __volatile__(".word 0xE1600070");
	__asm__ __volatile__("mov r0, %0":"=r" (i));
	__asm__ __volatile__("mov r12, %0":"=r" (j));
}

/**********************************************************
 * Routine: try_unlock_sram()
 * Description: If chip is GP/EMU(special) type, unlock the SRAM for
 *  general use.
 ***********************************************************/
void try_unlock_memory()
{
	int mode;
	int in_sdram = running_in_sdram();

	/* if GP device unlock device SRAM for general use */
	/* secure code breaks for Secure/Emulation device - HS/E/T*/
	mode = get_device_type();
	if (mode == GP_DEVICE) {
		secure_unlock_mem();
	}
	/* If device is EMU and boot is XIP external booting 
	 * Unlock firewalls and disable L2 and put chip 
	 * out of secure world
	 */
	/* Assuming memories are unlocked by the demon who put us in SDRAM */
	if((mode <= EMU_DEVICE) && (get_boot_type() == 0x1F)
		&& (!in_sdram)) {
		secure_unlock_mem();
		secureworld_exit();
	}
	
	return;
}

/**********************************************************
 * Routine: s_init
 * Description: Does early system init of muxing and clocks.
 * - Called path is with SRAM stack.
 **********************************************************/
void s_init(void)
{
	int in_sdram = running_in_sdram();

#ifdef CONFIG_3430VIRTIO
	in_sdram = 0;  /* allow setup from memory for Virtio */
#endif
	watchdog_init();

	try_unlock_memory();
	
	/* Right now flushing at low MPU speed. Need to move after clock init */
	v7_flush_dcache_all(get_device_type());
#ifndef CONFIG_ICACHE_OFF
	icache_enable();
#endif

#ifdef CONFIG_L2_OFF
	l2cache_disable();
#else
	l2cache_enable();
#endif
	/* Writing to AuxCR in U-boot using SMI for GP DEV */
	/* Currently SMI in Kernel on ES2 devices seems to have an isse
	 * Once that is resolved, we can postpone this config to kernel
	 */
	if(get_device_type() == GP_DEVICE)
		setup_auxcr();

	set_muxconf_regs();
	delay(100);

	prcm_init();

	per_clocks_enable();

	if (!in_sdram)
		sdrc_init();
}

ushort tone[] = {
	0x0ce4, 0x0ce4, 0x1985, 0x1985, 0x25A1, 0x25A1, 0x30FD, 0x30FE,
	0x3B56, 0x3B55, 0x447A, 0x447A, 0x4C3B, 0x4C3C, 0x526D, 0x526C,
	0x56F1, 0x56F1, 0x59B1, 0x59B1, 0x5A9E, 0x5A9D, 0x59B1, 0x59B2,
	0x56F3, 0x56F2, 0x526D, 0x526D, 0x4C3B, 0x4C3B, 0x447C, 0x447C,
	0x3B5A, 0x3B59, 0x30FE, 0x30FE, 0x25A5, 0x25A6, 0x1989, 0x198A,
	0x0CE5, 0x0CE3, 0x0000, 0x0000, 0xF31C, 0xF31C, 0xE677, 0xE676,
	0xDA5B, 0xDA5B, 0xCF03, 0xCF03, 0xC4AA, 0xC4AA, 0xBB83, 0xBB83,
	0xB3C5, 0xB3C5, 0xAD94, 0xAD94, 0xA90D, 0xA90E, 0xA64F, 0xA64E,
	0xA562, 0xA563, 0xA64F, 0xA64F, 0xA910, 0xA90F, 0xAD93, 0xAD94,
	0xB3C4, 0xB3C4, 0xBB87, 0xBB86, 0xC4AB, 0xC4AB, 0xCF03, 0xCF03,
	0xDA5B, 0xDA5A, 0xE67B, 0xE67B, 0xF31B, 0xF3AC, 0x0000, 0x0000,
	0x0CE4, 0x0CE4, 0x1985, 0x1985, 0x25A1, 0x25A1, 0x30FD, 0x30FE,
	0x3B56, 0x3B55, 0x447A, 0x447A, 0x4C3B, 0x4C3C, 0x526D, 0x526C,
	0x56F1, 0x56F1, 0x59B1, 0x59B1, 0x5A9E, 0x5A9D, 0x59B1, 0x59B2,
	0x56F3, 0x56F2, 0x526D, 0x526D, 0x4C3B, 0x4C3B, 0x447C, 0x447C,
	0x3B5A, 0x3B59, 0x30FE, 0x30FE, 0x25A5, 0x25A6, 0x1989, 0x198A,
	0x0CE5, 0x0CE3, 0x0000, 0x0000, 0xF31C, 0xF31C, 0xE677, 0xE676,
	0xDA5B, 0xDA5B, 0xCF03, 0xCF03, 0xC4AA, 0xC4AA, 0xBB83, 0xBB83,
	0xB3C5, 0xB3C5, 0xAD94, 0xAD94, 0xA90D, 0xA90E, 0xA64F, 0xA64E,
	0xA562, 0xA563, 0xA64F, 0xA64F, 0xA910, 0xA90F, 0xAD93, 0xAD94,
	0xB3C4, 0xB3C4, 0xBB87, 0xBB86, 0xC4AB, 0xC4AB, 0xCF03, 0xCF03,
	0xDA5B, 0xDA5A, 0xE67B, 0xE67B, 0xF31B, 0xF3AC, 0x0000, 0x0000,
	0x0CE4, 0x0CE4, 0x1985, 0x1985, 0x25A1, 0x25A1, 0x30FD, 0x30FE,
	0x3B56, 0x3B55, 0x447A, 0x447A, 0x4C3B, 0x4C3C, 0x526D, 0x526C,
	0x56F1, 0x56F1, 0x59B1, 0x59B1, 0x5A9E, 0x5A9D, 0x59B1, 0x59B2,
	0x56F3, 0x56F2, 0x526D, 0x526D, 0x4C3B, 0x4C3B, 0x447C, 0x447C,
	0x3B5A, 0x3B59, 0x30FE, 0x30FE, 0x25A5, 0x25A6, 0x1989, 0x198A,
	0x0CE5, 0x0CE3, 0x0000, 0x0000, 0xF31C, 0xF31C, 0xE677, 0xE676,
	0xDA5B, 0xDA5B, 0xCF03, 0xCF03, 0xC4AA, 0xC4AA, 0xBB83, 0xBB83,
	0xB3C5, 0xB3C5, 0xAD94, 0xAD94, 0xA90D, 0xA90E, 0xA64F, 0xA64E,
	0xA562, 0xA563, 0xA64F, 0xA64F, 0xA910, 0xA90F, 0xAD93, 0xAD94,
	0xB3C4, 0xB3C4, 0xBB87, 0xBB86, 0xC4AB, 0xC4AB, 0xCF03, 0xCF03,
	0xDA5B, 0xDA5A, 0xE67B, 0xE67B, 0xF31B, 0xF3AC, 0x0000, 0x0000
};

int audio_init()
{

	unsigned char byte;

	printf("Audio Tone on Speakers");
	byte = 0x20;
	//i2c_write(0x4B, 0x7A, 1, &byte, 1); //VAUX3_DEV_GRP
	byte = 0x03;
	//i2c_write(0x4B, 0x7D, 1, &byte, 1); //VAUX3_DEDICATED
	byte = 0xE0;
	i2c_write(0x4B, 0x8E, 1, &byte, 1); //VPLL2_DEV_GRP
	byte = 0x05;
	i2c_write(0x4B, 0x91, 1, &byte, 1); //VPLL2_DEDICATED
	byte = 0x03;
	i2c_write(0x49, 0x01, 1, &byte, 1);
	byte = 0xc0;
	i2c_write(0x49, 0x02, 1, &byte, 1);
	byte = 0x00;
	i2c_write(0x49, 0x03, 1, &byte, 1);
	byte = 0x00;
	i2c_write(0x49, 0x04, 1, &byte, 1);
	byte = 0x00;
	i2c_write(0x49, 0x05, 1, &byte, 1);
	byte = 0x00;
	i2c_write(0x49, 0x06, 1, &byte, 1);
	byte = 0x00;
	i2c_write(0x49, 0x07, 1, &byte, 1);
	byte = 0x00;
	i2c_write(0x49, 0x08, 1, &byte, 1);
	byte = 0x00;
	i2c_write(0x49, 0x09, 1, &byte, 1);
	byte = 0x00;
	i2c_write(0x49, 0x0a, 1, &byte, 1);
	byte = 0x00;
	i2c_write(0x49, 0x0b, 1, &byte, 1);
	byte = 0x00;
	i2c_write(0x49, 0x0c, 1, &byte, 1);
	byte = 0x00;
	i2c_write(0x49, 0x0d, 1, &byte, 1);
	byte = 0x01;
	i2c_write(0x49, 0x0e, 1, &byte, 1);
	byte = 0x00;
	i2c_write(0x49, 0x0f, 1, &byte, 1);
	byte = 0x00;
	i2c_write(0x49, 0x10, 1, &byte, 1);
	byte = 0x00;
	i2c_write(0x49, 0x11, 1, &byte, 1);
	byte = 0x6c;
	i2c_write(0x49, 0x12, 1, &byte, 1);
	byte = 0x6c;
	i2c_write(0x49, 0x13, 1, &byte, 1);
	byte = 0x00;
	i2c_write(0x49, 0x14, 1, &byte, 1);
	byte = 0x00;
	i2c_write(0x49, 0x15, 1, &byte, 1);
	byte = 0x00;
	i2c_write(0x49, 0x16, 1, &byte, 1);
	byte = 0x0c;
	i2c_write(0x49, 0x17, 1, &byte, 1);
	byte = 0x00;
	i2c_write(0x49, 0x18, 1, &byte, 1);
	byte = 0x00;
	i2c_write(0x49, 0x19, 1, &byte, 1);
	byte = 0x00;
	i2c_write(0x49, 0x1a, 1, &byte, 1);
	byte = 0x2b;
	i2c_write(0x49, 0x1b, 1, &byte, 1);
	byte = 0x2b;
	i2c_write(0x49, 0x1c, 1, &byte, 1);
	byte = 0x00;
	i2c_write(0x49, 0x1d, 1, &byte, 1);
	byte = 0x00;
	i2c_write(0x49, 0x1e, 1, &byte, 1);
	byte = 0x00;
	i2c_write(0x49, 0x1f, 1, &byte, 1);
	byte = 0x00;
	i2c_write(0x49, 0x20, 1, &byte, 1);
	byte = 0x00;
	i2c_write(0x49, 0x21, 1, &byte, 1);
	byte = 0x24;
	i2c_write(0x49, 0x22, 1, &byte, 1);
	byte = 0x0a;
	i2c_write(0x49, 0x23, 1, &byte, 1);
	byte = 0x42;
	i2c_write(0x49, 0x24, 1, &byte, 1);
	byte = 0x00;
	i2c_write(0x49, 0x25, 1, &byte, 1);
	byte = 0x00;
	i2c_write(0x49, 0x26, 1, &byte, 1);
	byte = 0x00;
	i2c_write(0x49, 0x27, 1, &byte, 1);
	byte = 0x00;
	i2c_write(0x49, 0x28, 1, &byte, 1);
	byte = 0x00;
	i2c_write(0x49, 0x29, 1, &byte, 1);
	byte = 0x00;
	i2c_write(0x49, 0x2a, 1, &byte, 1);
	byte = 0x00;
	i2c_write(0x49, 0x2b, 1, &byte, 1);
	byte = 0x00;
	i2c_write(0x49, 0x2c, 1, &byte, 1);
	byte = 0x00;
	i2c_write(0x49, 0x2d, 1, &byte, 1);
	byte = 0x00;
	i2c_write(0x49, 0x2e, 1, &byte, 1);
	byte = 0x00;
	i2c_write(0x49, 0x2f, 1, &byte, 1);
	byte = 0x00;
	i2c_write(0x49, 0x30, 1, &byte, 1);
	byte = 0x00;
	i2c_write(0x49, 0x31, 1, &byte, 1);
	byte = 0x00;
	i2c_write(0x49, 0x32, 1, &byte, 1);
	byte = 0x00;
	i2c_write(0x49, 0x33, 1, &byte, 1);
	byte = 0x00;
	i2c_write(0x49, 0x34, 1, &byte, 1);
	byte = 0x00;
	i2c_write(0x49, 0x35, 1, &byte, 1);
	byte = 0x00;
	i2c_write(0x49, 0x36, 1, &byte, 1);
	byte = 0x00;
	i2c_write(0x49, 0x37, 1, &byte, 1);
	byte = 0x00;
	i2c_write(0x49, 0x38, 1, &byte, 1);
	byte = 0x00;
	i2c_write(0x49, 0x39, 1, &byte, 1);
	byte = 0x15;
	i2c_write(0x49, 0x3a, 1, &byte, 1);
	byte = 0x00;
	i2c_write(0x49, 0x3b, 1, &byte, 1);
	byte = 0x00;
	i2c_write(0x49, 0x3c, 1, &byte, 1);
	byte = 0x00;
	i2c_write(0x49, 0x3d, 1, &byte, 1);
	byte = 0x00;
	i2c_write(0x49, 0x3e, 1, &byte, 1);
	byte = 0x00;
	i2c_write(0x49, 0x3f, 1, &byte, 1);
	byte = 0x00;
	i2c_write(0x49, 0x40, 1, &byte, 1);
	byte = 0x00;
	i2c_write(0x49, 0x41, 1, &byte, 1);
	byte = 0x00;
	i2c_write(0x49, 0x42, 1, &byte, 1);
	byte = 0x00;
	i2c_write(0x49, 0x43, 1, &byte, 1);
	byte = 0x00;
	i2c_write(0x49, 0x44, 1, &byte, 1);
	byte = 0x00;
	i2c_write(0x49, 0x45, 1, &byte, 1);
	byte = 0x00;
	i2c_write(0x49, 0x46, 1, &byte, 1);
	byte = 0x00;
	i2c_write(0x49, 0x47, 1, &byte, 1);
	byte = 0x00;
	i2c_write(0x49, 0x48, 1, &byte, 1);
	byte = 0x00;
	i2c_write(0x49, 0x49, 1, &byte, 1);

	//McBSP2
	*((uint *) 0x4902208c) = 0x00000208;
	*((uint *) 0x49022090) = 0x00000000;
	*((uint *) 0x49022094) = 0x00000000;
	*((uint *) 0x490220ac) = 0x00001008;
	*((uint *) 0x490220b0) = 0x00000808;
	*((uint *) 0x49022018) = 0x00000000;
	*((uint *) 0x4902201c) = 0x00000000;
	*((uint *) 0x49022020) = 0x00000040;
	*((uint *) 0x49022024) = 0x00000040;
	*((uint *) 0x49022028) = 0x00000000;
	*((uint *) 0x4902202c) = 0x00000000;
	*((uint *) 0x49022048) = 0x00000083;
	*((uint *) 0x49022010) = 0x00000200;
	*((uint *) 0x49022014) = 0x00000000;
	*((uint *) 0x4902207c) = 0x00000023;
	*((uint *) 0x49022010) = 0x00000201;
	*((uint *) 0x49022008) = 0x000056f3;
	//McBSP3
	*((uint *) 0x4902408c) = 0x00000208;
	*((uint *) 0x49024090) = 0x00000000;
	*((uint *) 0x49024094) = 0x00000000;
	*((uint *) 0x490240ac) = 0x00001008;
	*((uint *) 0x490240b0) = 0x00000808;
	*((uint *) 0x49024018) = 0x00000000;
	*((uint *) 0x4902401c) = 0x00000000;
	*((uint *) 0x49024020) = 0x00000080;
	*((uint *) 0x49024024) = 0x00000080;
	*((uint *) 0x49024028) = 0x00000000;
	*((uint *) 0x4902402c) = 0x00000000;
	*((uint *) 0x49024048) = 0x00000083;
	*((uint *) 0x49024010) = 0x00000200;
	*((uint *) 0x49024014) = 0x00000000;
	*((uint *) 0x4902407c) = 0x00000023;
	*((uint *) 0x49024010) = 0x00000201;
	*((uint *) 0x49024008) = 0x000056f3;
	printf("  ... complete\n");

	int count = 0;

	for (count = 0; count < 50; count++) {
		int bytes;
		for (bytes = 0; bytes < sizeof(tone) / 2; bytes++) {
			*((uint *) 0x49022008) = tone[bytes];
			*((uint *) 0x49024008) = tone[bytes];
			udelay(100);
		}
	}
}

dss_init()
{
	unsigned int i;
	for (i = 0; i < (0x70800 * 4);) {
		*((unsigned int *)(0x80500000 + i)) = 0xF100F100;
		i = i + 4;
	}
	*((uint *) 0x48310034) = 0xfffffFFf; //GPIO1_OE
	*((uint *) 0x48310094) = 0x00000120; //GPIO1_SETDATAOUT
//	*((uint *) 0x48310090) = 0x00000120; //GPIO1_CLEARDATAOUT
/*
GPIO1 - 0x48310000
GPIO2 - 0x49050000
GPIO3 - 0x49052000
GPIO4 - 0x49054000
GPIO5 - 0x49056000
GPIO6 - 0x49058000
*/
	*((uint *) 0x48004D44) = 0x0001b00c;
	*((uint *) 0x48004E40) = 0x00001006;
	*((uint *) 0x48004D00) = 0x00370037;
	*((uint *) 0x48050C00) = 0x00000002; //TV ENCODER
	*((uint *) 0x48050C04) = 0x0000001B;
	*((uint *) 0x48050C08) = 0x00000040;
	*((uint *) 0x48050C0C) = 0x00000000;
	*((uint *) 0x48050C10) = 0x00000000;
	*((uint *) 0x48050C14) = 0x00008000;
	*((uint *) 0x48050C18) = 0x00000000;
	*((uint *) 0x48050C1C) = 0x00008359;
	*((uint *) 0x48050C20) = 0x0000020C;
	*((uint *) 0x48050C24) = 0x00000000;
	*((uint *) 0x48050C28) = 0x043F2631;
	*((uint *) 0x48050C2C) = 0x00000024;
	*((uint *) 0x48050C30) = 0x00000130;
	*((uint *) 0x48050C34) = 0x00000198;
	*((uint *) 0x48050C38) = 0x000001C0;
	*((uint *) 0x48050C3C) = 0x0000006A;
	*((uint *) 0x48050C40) = 0x0000005C;
	*((uint *) 0x48050C44) = 0x00000000;
	*((uint *) 0x48050C48) = 0x00000001;
	*((uint *) 0x48050C4C) = 0x0000003F;
	*((uint *) 0x48050C50) = 0x21F07C1F;
	*((uint *) 0x48050C54) = 0x00000000;
	*((uint *) 0x48050C58) = 0x00000015;
	*((uint *) 0x48050C5C) = 0x00001400;
	*((uint *) 0x48050C60) = 0x00000000;
	*((uint *) 0x48050C64) = 0x069300F4;
	*((uint *) 0x48050C68) = 0x0016020C;
	*((uint *) 0x48050C6C) = 0x00060107;
	*((uint *) 0x48050C70) = 0x008D034E;
	*((uint *) 0x48050C74) = 0x000F0359;
	*((uint *) 0x48050C78) = 0x01A00000;
	*((uint *) 0x48050C7C) = 0x020501A0;
	*((uint *) 0x48050C80) = 0x01AC0024;
	*((uint *) 0x48050C84) = 0x020D01AC;
	*((uint *) 0x48050C88) = 0x00000006;
	*((uint *) 0x48050C8C) = 0x00000000;
	*((uint *) 0x48050C90) = 0x03480079;
	*((uint *) 0x48050C94) = 0x02040024;
	*((uint *) 0x48050C98) = 0x00000000;
	*((uint *) 0x48050C9C) = 0x00000000;
	*((uint *) 0x48050CA0) = 0x0001008A;
	*((uint *) 0x48050CA4) = 0x01AC0106;
	*((uint *) 0x48050CA8) = 0x01060006;
	*((uint *) 0x48050CAC) = 0x00000000;
	*((uint *) 0x48050CB0) = 0x00140001;
	*((uint *) 0x48050CB4) = 0x00010001;
	*((uint *) 0x48050CB8) = 0x00FF0000;
	*((uint *) 0x48050CBC) = 0x00000000;
	*((uint *) 0x48050CC0) = 0x00000000;
	*((uint *) 0x48050CC4) = 0x0000000D;
	*((uint *) 0x48050CC8) = 0x00000000;
	*((uint *) 0x48050010) = 0x00000001; //DSS CONFIG
	*((uint *) 0x48050040) = 0x00000078;
	*((uint *) 0x48050044) = 0x00000000;
	*((uint *) 0x48050048) = 0x00000000;
	*((uint *) 0x48050050) = 0x00000000;
	*((uint *) 0x48050058) = 0x00000000;
	*((uint *) 0x48050410) = 0x00002015; //LCD
	*((uint *) 0x48050414) = 0x00000001;
	*((uint *) 0x48050444) = 0x00000004;
	*((uint *) 0x4805044c) = 0xFFFFFFFF;
	*((uint *) 0x48050450) = 0x00000000;
	*((uint *) 0x48050454) = 0x00000000;
	*((uint *) 0x48050458) = 0x00000000;
	*((uint *) 0x48050464) = 0x0D702700; //HORIZONTAL TIMING
	*((uint *) 0x48050468) = 0x02300A00; //VERTICAL TIMING
	*((uint *) 0x4805046c) = 0x00027028; //POLARITIES
	*((uint *) 0x48050470) = 0x00010002; //CLOCK DIVIDERS(???)
	*((uint *) 0x48050478) = 0x00ef027f;
	*((uint *) 0x4805047c) = 0x01DF031F; //DISPLAY SIZE
	*((uint *) 0x48050480) = 0x80500000; //FB0
	*((uint *) 0x48050484) = 0x80500000; //FB1
	*((uint *) 0x48050488) = 0x00000000;
	*((uint *) 0x4805048c) = 0x01DF031F; //WINDOW SIZE
	*((uint *) 0x480504a0) = 0x0000008d;
	*((uint *) 0x480504a4) = 0x03fc03bc;
	*((uint *) 0x480504a8) = 0x00000400;
	*((uint *) 0x480504ac) = 0x00000001;
	*((uint *) 0x480504b0) = 0x00000001;
	*((uint *) 0x480504b4) = 0x00000000;
	*((uint *) 0x480504b8) = 0x807ff000;
	udelay(1000);
	*((uint *) 0x48050440) = 0x0000836b;
	udelay(1000);
	*((uint *) 0x48050440) = 0x0000836b;
	udelay(1000);
	*((uint *) 0x48050440) = 0x0000836b;
	udelay(1000);

}

/*******************************************************
 * Routine: misc_init_r
 * Description: Init ethernet (done here so udelay works)
 ********************************************************/
int misc_init_r(void)
{
	unsigned char byte;
	int i;
#ifdef CONFIG_DRIVER_OMAP34XX_I2C
	i2c_init(CFG_I2C_SPEED, CFG_I2C_SLAVE);
#endif
	byte = 0x20;
	i2c_write(0x4B, 0x7A, 1, &byte, 1);
	byte = 0x03;
	i2c_write(0x4B, 0x7D, 1, &byte, 1);
	byte = 0xE0;
	i2c_write(0x4B, 0x8E, 1, &byte, 1);
	byte = 0x05;
	i2c_write(0x4B, 0x91, 1, &byte, 1);
	byte = 0x20;
	i2c_write(0x4B, 0x96, 1, &byte, 1);
	byte = 0x03;
	i2c_write(0x4B, 0x99, 1, &byte, 1);

        *((uint *) 0x49056034) = 0x00000000;
        *((uint *) 0x49056094) = 0xFFFFFFFF;
        *((uint *) 0x4905603C) = 0xFFFFFFFF;

	dss_init();
	audio_init();
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
	wait_on_value(BIT5, 0x20, CM_IDLEST_WKUP, 5); /* some issue here */

	__raw_writel(WD_UNLOCK1, WD2_BASE + WSPR);
	wait_for_command_complete(WD2_BASE);
	__raw_writel(WD_UNLOCK2, WD2_BASE + WSPR);
}

/*******************************************************************
 * Routine:ether_init
 * Description: take the Ethernet controller out of reset and wait
 *  		   for the EEPROM load to complete.
 ******************************************************************/
void ether_init(void)
{
#ifdef CONFIG_DRIVER_LAN91C96
	int cnt = 20;

	__raw_writew(0x0, LAN_RESET_REGISTER);
	do {
		__raw_writew(0x1, LAN_RESET_REGISTER);
		udelay(100);
		if (cnt == 0)
			goto h4reset_err_out;
		--cnt;
	} while (__raw_readw(LAN_RESET_REGISTER) != 0x1);

	cnt = 20;

	do {
		__raw_writew(0x0, LAN_RESET_REGISTER);
		udelay(100);
		if (cnt == 0)
			goto h4reset_err_out;
		--cnt;
	} while (__raw_readw(LAN_RESET_REGISTER) != 0x0000);
	udelay(1000);

	*((volatile unsigned char *)ETH_CONTROL_REG) &= ~0x01;
	udelay(1000);

      h4reset_err_out:
	return;
#endif
}

/**********************************************
 * Routine: dram_init
 * Description: sets uboots idea of sdram size
 **********************************************/
int dram_init(void)
{
    #define NOT_EARLY 0
    DECLARE_GLOBAL_DATA_PTR;
	unsigned int size0 = 0, size1 = 0;
	u32 mtype, btype;

	btype = get_board_type();
	mtype = get_mem_type();
#ifndef CONFIG_3430ZEBU
	/* fixme... dont know why this func is crashing in ZeBu */
	display_board_info(btype);
#endif
    /* If a second bank of DDR is attached to CS1 this is 
     * where it can be started.  Early init code will init
     * memory on CS0.
     */ 
	if ((mtype == DDR_COMBO) || (mtype == DDR_STACKED)) {
		do_sdrc_init(SDRC_CS1_OSET, NOT_EARLY);
	}
	size0 = get_sdr_cs_size(SDRC_CS0_OSET);
	size1 = get_sdr_cs_size(SDRC_CS1_OSET);

	gd->bd->bi_dram[0].start = PHYS_SDRAM_1;
	gd->bd->bi_dram[0].size = size0;
	gd->bd->bi_dram[1].start = PHYS_SDRAM_1+size0;
	gd->bd->bi_dram[1].size = size1;

	return 0;
}

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
#define MUX_DEFAULT_ES2()\
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
	/*DSS*/\
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
	MUX_VAL(CP(CAM_D0 ),        (IEN  | PTU | EN  | M4)) /*GPIO_99, GAME_MENU*/\
	MUX_VAL(CP(CAM_D1 ),        (IEN  | PTU | EN  | M4)) /*GPIO_100, GAME_START*/\
	MUX_VAL(CP(CAM_D2 ),        (IEN  | PTU | EN  | M4)) /*GPIO_101, GAME_BUTTON_Y*/\
	MUX_VAL(CP(CAM_D3 ),        (IEN  | PTU | EN  | M4)) /*GPIO_102, GAME_L1_SHOULDER*/\
	MUX_VAL(CP(CAM_D4 ),        (IEN  | PTU | EN  | M4)) /*GPIO_103, GAME_DPAD_DOWN*/\
	MUX_VAL(CP(CAM_D5 ),        (IEN  | PTU | EN  | M4)) /*GPIO_104, GAME_SELECT*/\
	MUX_VAL(CP(CAM_D6 ),        (IEN  | PTU | EN  | M4)) /*GPIO_105, GAME_R1_SHOULDER*/\
	MUX_VAL(CP(CAM_D7 ),        (IEN  | PTU | EN  | M4)) /*GPIO_106, GAME_B*/\
	MUX_VAL(CP(CAM_D8 ),        (IEN  | PTU | EN  | M4)) /*GPIO_107, GAME_R2_SHOULDER*/\
	MUX_VAL(CP(CAM_D9 ),        (IEN  | PTU | EN  | M4)) /*GPIO_108, LID_SWITCH*/\
	MUX_VAL(CP(CAM_D10),        (IEN  | PTU | EN  | M4)) /*GPIO_109, GAME_X*/\
	MUX_VAL(CP(CAM_D11),        (IEN  | PTU | EN  | M4)) /*GPIO_110, GAME_DPAD_UP*/\
	MUX_VAL(CP(CAM_XCLKB),      (IEN  | PTU | EN  | M4)) /*GPIO_111, GAME_A*/\
	MUX_VAL(CP(CAM_WEN),        (IEN  | PTU | EN  | M4)) /*GPIO_167, SPARE_GPIO3*/\
	MUX_VAL(CP(CAM_STROBE),     (IEN  | PTU | EN  | M4)) /*GPIO_126, SPARE_GPIO2*/\
	MUX_VAL(CP(CSI2_DX0),       (IEN  | PTD | DIS | M0)) /*CSI2_DX0, UNCONNECTED*/\
	MUX_VAL(CP(CSI2_DY0),       (IEN  | PTD | DIS | M0)) /*CSI2_DY0, UNCONNECTED*/\
	MUX_VAL(CP(CSI2_DX1),       (IEN  | PTD | DIS | M0)) /*CSI2_DX1, UNCONNECTED*/\
	MUX_VAL(CP(CSI2_DY1),       (IEN  | PTD | DIS | M0)) /*CSI2_DY1, UNCONNECTED*/\
	/*Audio Interface To Triton2*/\
	MUX_VAL(CP(McBSP2_FSX),     (IEN  | PTD | DIS | M0)) /*McBSP2_FSX*/\
	MUX_VAL(CP(McBSP2_CLKX),    (IEN  | PTD | DIS | M0)) /*McBSP2_CLKX*/\
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
	MUX_VAL(CP(MMC2_CLK),       (IEN  | PTD | DIS | M0)) /*MMC2_CLK*/\
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
	MUX_VAL(CP(McBSP3_DR),      (IDIS | PTD | DIS | M4)) /*GPIO_141, nPOWERDOWN_DAC*/\
	MUX_VAL(CP(McBSP3_CLKX),    (IEN  | PTD | DIS | M0)) /*McBSP3_CLKX  */\
	MUX_VAL(CP(McBSP3_FSX),     (IEN  | PTD | DIS | M0)) /*McBSP3_FSX*/\
	/*Future interface to Bluetooth or other usage (EXP CONN)*/\
	MUX_VAL(CP(UART1_TX),       (IDIS | PTD | DIS | M0)) /*UART1_TX*/\
      MUX_VAL(CP(UART1_RTS),      (IDIS | PTD | DIS | M0)) /*UART1_RTS*/\
      MUX_VAL(CP(UART1_CTS),      (IDIS | PTD | DIS | M0)) /*UART1_CTS*/\
	MUX_VAL(CP(UART1_RX),       (IEN  | PTD | DIS | M0)) /*UART1_RX*/\
	MUX_VAL(CP(McBSP4_CLKX),    (IEN  | PTD | DIS | M0)) /*McBSP4_CLKX*/\
	MUX_VAL(CP(McBSP4_DR),      (IEN  | PTD | DIS | M0)) /*McBSP4_DR*/\
	MUX_VAL(CP(McBSP4_DX),      (IDIS | PTD | DIS | M0)) /*McBSP4_DX*/\
	MUX_VAL(CP(McBSP4_FSX),     (IEN  | PTD | DIS | M0)) /*McBSP4_FSX*/\
	/*GPIO definitions for muxed pins*/\
	MUX_VAL(CP(UART2_CTS),      (IEN  | PTU | EN  | M4)) /*GPIO_144, nTOUCH_IRQ*/\
	MUX_VAL(CP(UART2_RTS),      (IEN  | PTU | DIS | M4)) /*GPIO_145, nNUB_RESET*/\
	MUX_VAL(CP(UART2_TX),       (IEN  | PTU | DIS | M4)) /*GPIO_146, nTOUCH_BUSY*/\
	MUX_VAL(CP(UART2_RX),       (IEN  | PTD | DIS | M4)) /*GPIO_147, CS_USB_HOST*/\
	MUX_VAL(CP(McBSP1_CLKX),    (IDIS | PTD | DIS | M4)) /*GPIO_162, START_ADC*/\
	MUX_VAL(CP(UART3_CTS_RCTX), (IEN  | PTD | DIS | M4)) /*GPIO_163, nOC_USB5*/\
	MUX_VAL(CP(UART3_RTS_SD),   (IEN  | PTD | DIS | M4)) /*GPIO_164, EN_USB_5V*/\
	MUX_VAL(CP(HDQ_SIO),        (IEN  | PTU | EN  | M0)) /*HDQ_SIO, NOT CONNECTED*/\
	MUX_VAL(CP(ETK_D0_ES2 ),    (IEN  | PTD | DIS | M4)) /*GPIO_14, nHEADPHONE_SHUTDOWN*/\
	MUX_VAL(CP(ETK_D1_ES2 ),    (IDIS | PTD | DIS | M4)) /*GPIO_15, WIFI_POWER_ENABLE*/\
	MUX_VAL(CP(ETK_D2_ES2 ),    (IEN  | PTD | DIS | M4)) /*GPIO_16, MOTION_NUB2*/\
	MUX_VAL(CP(ETK_D7_ES2 ),    (IEN  | PTD | DIS | M4)) /*GPIO_21, MOTION_NUB1*/\
	MUX_VAL(CP(ETK_D8_ES2 ),    (IDIS | PTD | DIS | M4)) /*GPIO_22, MSECURE*/\
	MUX_VAL(CP(ETK_D9_ES2 ),    (IDIS | PTD | DIS | M4)) /*GPIO_23, WIFI_RESET*/\
	/*LCD Control Interface*/\
	MUX_VAL(CP(McBSP1_CLKR),    (IEN  | PTD | DIS | M1)) /*McSPI4_CLK, SPI_LCD_CLK*/\
	MUX_VAL(CP(McBSP1_FSR),     (IDIS | PTU | EN  | M4)) /*GPIO_157, LCD_RESET*/\
	MUX_VAL(CP(McBSP1_DX),      (IDIS | PTD | DIS | M1)) /*McSPI4_SIMO, SPI_LCD_MOSI*/\
	MUX_VAL(CP(McBSP1_DR),      (IDIS | PTD | DIS | M4)) /*GPIO_159, LCD_STANDBY*/\
	MUX_VAL(CP(McBSP1_FSX),     (IEN  | PTD | DIS | M1)) /*McBSP1_FSX*/\
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
	MUX_VAL(CP(I2C2_SCL),       (IEN  | PTU | EN  | M0)) /*I2C2_SCL*/\
	MUX_VAL(CP(I2C2_SDA),       (IEN  | PTU | EN  | M0)) /*I2C2_SDA*/\
	MUX_VAL(CP(I2C3_SCL),       (IEN  | PTU | EN  | M0)) /*I2C3_SCL*/\
	MUX_VAL(CP(I2C3_SDA),       (IEN  | PTU | EN  | M0)) /*I2C3_SDA*/\
	MUX_VAL(CP(I2C4_SCL),       (IEN  | PTU | EN  | M0)) /*I2C4_SCL*/\
	MUX_VAL(CP(I2C4_SDA),       (IEN  | PTU | EN  | M0)) /*I2C4_SDA*/\
	/*Serial Interface (Touch, Analog Nubs)*/\
	MUX_VAL(CP(McSPI1_CLK),     (IEN  | PTD | DIS | M0)) /*McSPI1_CLK*/\
	MUX_VAL(CP(McSPI1_SIMO),    (IEN  | PTD | DIS | M0)) /*McSPI1_SIMO*/\
	MUX_VAL(CP(McSPI1_SOMI),    (IEN  | PTD | DIS | M0)) /*McSPI1_SOMI*/\
	MUX_VAL(CP(McSPI1_CS0),     (IEN  | PTD | EN  | M0)) /*McSPI1_CS0, TOUCH*/\
	MUX_VAL(CP(McSPI1_CS1),     (IEN  | PTD | EN  | M0)) /*McSPI1_CS1, NUB_LEFT*/\
	MUX_VAL(CP(McSPI1_CS2),     (IEN  | PTD | EN  | M0)) /*McSPI1_CS2, NUB_RIGHT*/\
	/*HS USB HOST Port (connects to HSUSB2)*/\
	MUX_VAL(CP(ETK_D10_ES2),    (IDIS | PTU | EN  | M3)) /*USB_HOST_CLK*/\
	MUX_VAL(CP(ETK_D11_ES2),    (IDIS | PTU | EN  | M3)) /*USB_HOST_STP*/\
	MUX_VAL(CP(ETK_D12_ES2),    (IDIS | PTU | EN  | M3)) /*USB_HOST_DIR*/\
	MUX_VAL(CP(ETK_D13_ES2),    (IDIS | PTU | EN  | M3)) /*USB_HOST_NXT*/\
	MUX_VAL(CP(ETK_D14_ES2),    (IDIS | PTU | EN  | M3)) /*USB_HOST_D0*/\
	MUX_VAL(CP(ETK_D15_ES2),    (IDIS | PTU | EN  | M3)) /*USB_HOST_D1*/\
	MUX_VAL(CP(McSPI1_CS3),     (IEN  | PTD | EN  | M3)) /*USB_HOST_D2*/\
	MUX_VAL(CP(McSPI2_CS1),     (IEN  | PTD | EN  | M3)) /*USB_HOST_D3*/\
	MUX_VAL(CP(McSPI2_SIMO),    (IEN  | PTD | DIS | M3)) /*USB_HOST_D4*/\
	MUX_VAL(CP(McSPI2_SOMI),    (IEN  | PTD | DIS | M3)) /*USB_HOST_D5*/\
	MUX_VAL(CP(McSPI2_CS0),     (IEN  | PTD | EN  | M3)) /*USB_HOST_D6*/\
	MUX_VAL(CP(McSPI2_CLK),     (IEN  | PTD | DIS | M3)) /*USB_HOST_D7*/\
	/*Control and debug */\
	MUX_VAL(CP(SYS_32K),        (IEN  | PTD | DIS | M0)) /*SYS_32K*/\
	MUX_VAL(CP(SYS_CLKREQ),     (IEN  | PTD | DIS | M0)) /*SYS_CLKREQ*/\
	MUX_VAL(CP(SYS_nIRQ),       (IEN  | PTU | EN  | M0)) /*SYS_nIRQ*/\
	MUX_VAL(CP(SYS_BOOT0),      (IEN  | PTD | DIS | M4)) /*GPIO_2, CANNOT BE USED (LEAVE INPUT)*/\
	MUX_VAL(CP(SYS_BOOT1),      (IEN  | PTD | DIS | M4)) /*GPIO_3, CANNOT BE USED (LEAVE INPUT)*/\*/\
	MUX_VAL(CP(SYS_BOOT2),      (IEN  | PTD | DIS | M4)) /*GPIO_4, CANNOT BE USED (LEAVE INPUT)*/\*/\
	MUX_VAL(CP(SYS_BOOT3),      (IEN  | PTD | DIS | M4)) /*GPIO_5, CANNOT BE USED (LEAVE INPUT)*/\*/\
	MUX_VAL(CP(SYS_BOOT4),      (IEN  | PTD | DIS | M4)) /*GPIO_6, CANNOT BE USED (LEAVE INPUT)*/\*/\
	MUX_VAL(CP(SYS_BOOT5),      (IEN  | PTD | DIS | M4)) /*GPIO_7, CANNOT BE USED (LEAVE INPUT)*/\*/\
	MUX_VAL(CP(SYS_BOOT6),      (IEN  | PTD | DIS | M4)) /*GPIO_8, CANNOT BE USED (LEAVE INPUT)*/\*/\
	MUX_VAL(CP(SYS_OFF_MODE),   (IEN  | PTD | DIS | M0)) /*SYS_OFF_MODE */\
	MUX_VAL(CP(SYS_CLKOUT1),    (IEN  | PTD | DIS | M4)) /*SYS_CLKOUT1, NOT_CONNECTED*/\
	MUX_VAL(CP(SYS_CLKOUT2),    (IEN  | PTD | DIS | M4)) /*SYS_CLKOUT2, NOT_CONNECTED*/\
	/*SDIO Interface to WIFI Module*/\
	MUX_VAL(CP(ETK_CLK_ES2),    (IDIS | PTU | EN  | M2)) /*MMC3_CLK*/\
	MUX_VAL(CP(ETK_CTL_ES2),    (IDIS | PTD | DIS | M2)) /*MMC3_CMD*/\
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
	MUX_DEFAULT_ES2();
}

/******************************************************************************
 * Routine: update_mux()
 * Description:Update balls which are different between boards.  All should be
 *             updated to match functionality.  However, I'm only updating ones
 *             which I'll be using for now.  When power comes into play they
 *             all need updating.
 *****************************************************************************/
void update_mux(u32 btype, u32 mtype)
{
	/* NOTHING as of now... */
}

#if (CONFIG_COMMANDS & CFG_CMD_NAND) && defined(CFG_NAND_LEGACY)
/**********************************************************
 * Routine: nand+_init
 * Description: Set up nand for nand and jffs2 commands
 *********************************************************/
void nand_init(void)
{
	extern flash_info_t flash_info[];

	nand_probe(CFG_NAND_ADDR);
	if (nand_dev_desc[0].ChipID != NAND_ChipID_UNKNOWN) {
		print_size(nand_dev_desc[0].totlen, "\n");
	}
#ifdef CFG_JFFS2_MEM_NAND
	flash_info[CFG_JFFS2_FIRST_BANK].flash_id = nand_dev_desc[0].id;
	/* only read kernel single meg partition */
	flash_info[CFG_JFFS2_FIRST_BANK].size = 1024 * 1024 * 2;
	/* 1024 blocks in 16meg chip (use less for raw/copied partition) */
	flash_info[CFG_JFFS2_FIRST_BANK].sector_count = 1024;
	/* ?, ram for now, open question, copy to RAM or adapt for NAND */
	flash_info[CFG_JFFS2_FIRST_BANK].start[0] = 0x80200000;
#endif
}
#endif

