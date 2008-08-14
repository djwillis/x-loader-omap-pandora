/*
 * Basic I2C functions
 *
 * Copyright (c) 2004 Texas Instruments
 *
 * This package is free software;  you can redistribute it and/or
 * modify it under the terms of the license found in the file
 * named COPYING that should have accompanied this file.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 * Author: Jian Zhang jzhang@ti.com, Texas Instruments
 *
 * Copyright (c) 2003 Wolfgang Denk, wd@denx.de
 * Rewritten to fit into the current U-Boot framework
 *
 * Adapted for OMAP2420 I2C, r-woodruff2@ti.com
 *
 */

#include <common.h>

#include <asm/arch/i2c.h>
#include <asm/io.h>
#include <i2c.h>

static const u32 i2c_base = I2C_DEFAULT_BASE;

//#define DEBUG

#if DEBUG

#define DBG(ARGS...) {printf ("[%d]",__LINE__);printf(ARGS);}
#define inb(a) ({u8 v=__raw_readb(i2c_base + (a));printf("%d:Rb[%x<=%x]\n",__LINE__,a,v);v;})
#define outb(v,a) {printf("%d:Wb[%x<=%x]\n",__LINE__,a,v);__raw_writeb((v), (i2c_base + (a)));}
#define inw(a) ({u16 v=__raw_readb(i2c_base + (a));printf("%d:Rw[%x<=%x]\n",__LINE__,a,v);v;})
#define outw(v,a) {printf("%d:Ww[%x<=%x]\n",__LINE__,a,v);__raw_writew((v), (i2c_base + (a)));}

#else
#define DBG(ARGS...)
#define inb(a) __raw_readb(i2c_base + (a))
#define outb(v,a) __raw_writeb((v), (i2c_base + (a)))
#define inw(a) __raw_readw(i2c_base +(a))
#define outw(v,a) __raw_writew((v), (i2c_base + (a)))
#endif

static void wait_for_bb(void);
static u16 wait_for_pin(void);
static void flush_fifo(void);

/*******************************************************
 * Routine: udelay
 * Description: fake, wrong udelay
 ******************************************************/
void udelay(unsigned long loops)
{
	loops *= 600/2;
	__asm__ volatile ("1:\n" "subs %0, %1, #1\n"
			  "bne 1b":"=r" (loops):"0"(loops));
}


void i2c_init(int speed, int slaveaddr)
{
	int scl_lh = 0;
	int psc = 0;
	int iclk = 0;

	/* assume clock settings done */
	/* write to clock regs to enable if and fun clks for board */

	outw(0x2, I2C_SYSC);	/* for ES2 after soft reset */
	udelay(1000);
	outw(0x0, I2C_SYSC);	/* will probably self clear but */

	if (inw(I2C_CON) & I2C_CON_EN) {
		outw(0, I2C_CON);
		udelay(50000);
	}

	/* notaz: hardcoded detected parameters from u-boot */
	iclk = 6000;
	psc = I2C_PSC_MAX;
	scl_lh = 23;

	/* Initialize the I2C clock timers to generate an I2C bus clock
	 * frequency of i2c_clock kilohertz (default is 100 KHz).
	 */

	DBG(" speed= %d SysClk=%d, iclk=%d,psc=0x%x[%d],scl_lh=0x%x[%d]\n",
	       speed, I2C_IP_CLK, iclk, psc, psc, scl_lh, scl_lh);

	outw(psc, I2C_PSC);
	outw(scl_lh, I2C_SCLL);
	outw(scl_lh, I2C_SCLH);
	/* own address */
	outw(CFG_I2C_SLAVE, I2C_OA);
	outw(I2C_CON_EN, I2C_CON);

	/* have to enable intrrupts or OMAP i2c module doesn't work */
	outw(I2C_IE_XRDY_IE | I2C_IE_RRDY_IE | I2C_IE_ARDY_IE |
	     I2C_IE_NACK_IE | I2C_IE_AL_IE, I2C_IE);
	udelay(1000);
	flush_fifo();
	outw(0xFFFF, I2C_STAT);
	outw(0, I2C_CNT);
}


static int i2c_write_byte(u8 devaddr, u8 regoffset, u8 value)
{
	int i2c_error = 0;
	u16 status, stat;

	/* wait until bus not busy */
	wait_for_bb();

	/* two bytes */
	outw(2, I2C_CNT);
	/* set slave address */
	outw(devaddr, I2C_SA);
	/* stop bit needed here */
	outw(I2C_CON_EN | I2C_CON_MST | I2C_CON_STT | I2C_CON_TRX | I2C_CON_STP, I2C_CON);

	/* wait until state change */
	status = wait_for_pin();

	if (status & I2C_STAT_XRDY) {
		/* send out 1 byte */
		outb(regoffset, I2C_DATA);
		outw(I2C_STAT_XRDY, I2C_STAT);
		status = wait_for_pin();
		if ((status & I2C_STAT_XRDY)) {
			/* send out next 1 byte */
			outb(value, I2C_DATA);
			outw(I2C_STAT_XRDY, I2C_STAT);
		} else {
			i2c_error = 1;
		}
		/* must have enough delay to allow BB bit to go low */
		udelay(50000);
		if (inw(I2C_STAT) & I2C_STAT_NACK) {
			i2c_error = 1;
		}
	} else {
		i2c_error = 1;
	}
	if (!i2c_error) {
		int eout = 200;

		outw(I2C_CON_EN, I2C_CON);
		while ((stat = inw(I2C_STAT)) || (inw(I2C_CON) & I2C_CON_MST)) {
			udelay(1000);
			/* have to read to clear intrrupt */
			outw(0xFFFF, I2C_STAT);
			if (--eout == 0)	/* better leave with error than hang */
				break;
		}
	}
	flush_fifo();
	outw(0xFFFF, I2C_STAT);
	outw(0, I2C_CNT);
	return i2c_error;
}

static void flush_fifo(void)
{
	u16 stat;

	/* note: if you try and read data when its not there or ready
	 * you get a bus error
	 */
	while (1) {
		stat = inw(I2C_STAT);
		if (stat == I2C_STAT_RRDY) {
			inb(I2C_DATA);
			outw(I2C_STAT_RRDY, I2C_STAT);
			udelay(1000);
		} else
			break;
	}
}

int i2c_write(uchar chip, uint addr, int alen, uchar * buffer, int len)
{
	int i;

	if (alen > 1) {
		printf("I2C read: addr len %d not supported\n", alen);
		return 1;
	}

	if (addr + len > 256) {
		printf("I2C read: address out of range\n");
		return 1;
	}

	for (i = 0; i < len; i++) {
		if (i2c_write_byte(chip, addr + i, buffer[i])) {
			printf("I2C read: I/O error\n");
			i2c_init(CFG_I2C_SPEED, CFG_I2C_SLAVE);
			return 1;
		}
	}

	return 0;
}

static void wait_for_bb(void)
{
	int timeout = 10;
	u16 stat;

	outw(0xFFFF, I2C_STAT);	/* clear current interruts... */
	while ((stat = inw(I2C_STAT) & I2C_STAT_BB) && timeout--) {
		outw(stat, I2C_STAT);
		udelay(50000);
	}

	if (timeout <= 0) {
		printf("timed out in wait_for_bb: I2C_STAT=%x\n",
		       inw(I2C_STAT));
	}
	outw(0xFFFF, I2C_STAT);	/* clear delayed stuff */
}

static u16 wait_for_pin(void)
{
	u16 status;
	int timeout = 10;

	do {
		udelay(1000);
		status = inw(I2C_STAT);
	} while (!(status &
		   (I2C_STAT_ROVR | I2C_STAT_XUDF | I2C_STAT_XRDY |
		    I2C_STAT_RRDY | I2C_STAT_ARDY | I2C_STAT_NACK |
		    I2C_STAT_AL)) && timeout--);

	if (timeout <= 0) {
		printf("timed out in wait_for_pin: I2C_STAT=%x\n",
		       inw(I2C_STAT));
		outw(0xFFFF, I2C_STAT);
	}
	return status;
}

