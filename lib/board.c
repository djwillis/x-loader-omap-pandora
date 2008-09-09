/*
 * Copyright (C) 2005 Texas Instruments.
 *
 * (C) Copyright 2004
 * Jian Zhang, Texas Instruments, jzhang@ti.com.
 *
 * (C) Copyright 2002
 * Wolfgang Denk, DENX Software Engineering, wd@denx.de.
 *
 * (C) Copyright 2002
 * Sysgo Real-Time Solutions, GmbH <www.elinos.com>
 * Marius Groeger <mgroeger@sysgo.de>
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
#include <part.h>
#include <fat.h>
#include <asm/arch/mem.h>

const char version_string[] =
	"Texas Instruments X-Loader 1.4.2 (" __DATE__ " - " __TIME__ ")";

#ifdef CFG_PRINTF
int print_info(void)
{
        printf("\n\n%s\n", version_string);
	return 0;
}
#endif

static int init_func_i2c (void)
{
#ifdef CONFIG_MMC
	i2c_init (CFG_I2C_SPEED, CFG_I2C_SLAVE);
#endif
	return 0;
}

typedef int (init_fnc_t) (void);

init_fnc_t *init_sequence[] = {
	cpu_init,		/* basic cpu dependent setup */
	board_init,		/* basic board dependent setup */
#ifdef CFG_PRINTF
 	serial_init,		/* serial communications setup */
	print_info,
#endif
  	nand_init,		/* board specific nand init */
#ifdef CONFIG_MMC
	init_func_i2c,
#endif
  	NULL,
};

void start_armboot (void)
{
  	init_fnc_t **init_fnc_ptr;
 	int i, size;
	uchar *buf;
	block_dev_desc_t *dev_desc = NULL;

   	for (init_fnc_ptr = init_sequence; *init_fnc_ptr; ++init_fnc_ptr) {
		if ((*init_fnc_ptr)() != 0) {
			hang ();
		}
	}

	misc_init_r();

	buf =  (uchar*) CFG_LOADADDR;

#ifdef CONFIG_MMC
	/* first try mmc */
	if (mmc_init(1)) {
		size = file_fat_read("u-boot.bin", buf, 0);
		if (size > 0) {
#ifdef CFG_PRINTF
			printf("Loading u-boot.bin from MMC\n");
			printf("\n%ld Bytes Read from MMC \n", size);
#endif
			buf += size;
		}
	}
#endif

	if (buf == (uchar *)CFG_LOADADDR) {
		/* if no u-boot on mmc, try onenand or nand, depending upon sysboot */
		if (get_mem_type() == GPMC_ONENAND){
#ifdef CFG_PRINTF
       			printf("Loading u-boot.bin from onenand\n");
#endif
        		for (i = ONENAND_START_BLOCK; i < ONENAND_END_BLOCK; i++){
        			if (!onenand_read_block(buf, i))
        				buf += ONENAND_BLOCK_SIZE;
        		}
		} else if (get_mem_type() == GPMC_NAND){
#ifdef CFG_PRINTF
       			printf("Loading u-boot.bin from NAND\n");
#endif
        		for (i = NAND_UBOOT_START; i < NAND_UBOOT_END; i+= NAND_BLOCK_SIZE){
        			if (!nand_read_block(buf, i))
        				buf += NAND_BLOCK_SIZE; /* advance buf ptr */
        		}
		}
	}


	if (buf == (uchar *)CFG_LOADADDR)
		hang();

#ifdef CFG_PRINTF
	printf("Starting Stage 2 Bootloader...\n");
#endif

	/* go run U-Boot and never return */
 	((init_fnc_t *)CFG_LOADADDR)();

	/* should never come here */
}

void hang (void)
{
	/* call board specific hang function */
	board_hang();

	/* if board_hang() returns, hange here */
#ifdef CFG_PRINTF
	printf("X-Loader hangs\n");
#endif
	for (;;);
}
