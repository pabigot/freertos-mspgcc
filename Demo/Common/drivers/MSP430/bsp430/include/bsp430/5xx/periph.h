/* This intentionally uses the GCC/ISO C11 extensions for unnamed
 * struct/union fields. */

#ifndef BSP430_5XX_PERIPH_H
#define BSP430_5XX_PERIPH_H

/* Register map for (e)USCI_xy peripheral on a MSP430 5xx/6xx MCU. */
typedef struct bsp430_USCI
{
	union { /* 0x00 */
		unsigned int ctlw0;
		struct {
			unsigned char ctl1;
			unsigned char ctl0;
		};
	};
	unsigned int _unused_0x02;
	unsigned int _unused_0x04;
	union { /* 0x06 */
		unsigned int brw;
		struct {
			unsigned char br0;
			unsigned char br1;
		};
	};
	union {			/* 0x08 */
		struct { /* USCI_A */
			unsigned char mctl;
			unsigned char _reserved_0x09;
		};
		unsigned int _reserved_0x08;
	};
	unsigned char stat;			/* 0x0A */
	unsigned char _reserved_0x0B;
	unsigned char rxbuf;		/* 0x0C */
	unsigned char _reserved_0x0D;
	unsigned char txbuf;		/* 0x0E */
	unsigned char _reserved_0x0F;
	union {						/* 0x10 */
		struct {				/* USCI_A */
			unsigned char abctl;
			unsigned char _reserved_0x11;
		};
		unsigned int i2coa;		/* USCI_B */
	};
	union {						/* 0x12 */
		unsigned int irctl;		/* USCI_A */
		struct {
			unsigned char irtctl;
			unsigned char irrctl;
		};
		unsigned int i2csa;		/* USCI_B */
	};
	unsigned int _unused_0x14;
	unsigned int _unused_0x16;
	unsigned int _unused_0x18;
	unsigned int _unused_0x1a;
	union {						/* 0x1C */
		unsigned int ictl;
		struct {
			unsigned char ie;
			unsigned char ifg;
		};
	};
	unsigned int iv;			/* 0x1E */
} bsp430_USCI;

/* Structure for a single DMA channel */
typedef struct bsp430_DMAX_channel {
	unsigned int ctl;			/* 0x00 */
	unsigned long int __attribute__((__a20__)) sa; /* 0x02 */
	unsigned long int __attribute__((__a20__)) da; /* 0x06 */
	unsigned int sz;							   /* 0x0a */
	unsigned int _reserved_0x0c;
	unsigned int _reserved_0x0e;
} bsp430_DMAX_channel;

/* Structure for the DMAX peripheral.  The number of channels is
 * device-specific. */
typedef struct bsp430_DMAX {
	unsigned int ctl0;
	unsigned int ctl1;
	unsigned int ctl2;
	unsigned int ctl3;
	unsigned int ctl4;
	unsigned int iv;
	bsp430_DMAX_channel channel[1];
} bsp430_DMAX;


#endif /* BSP430_5XX_PERIPH_H */
