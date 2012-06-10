#include "platform.h"
#include "serial.h"

/* Register map for USCI_Ax peripheral in UART mode */
typedef struct __attribute__((__packed__)) xUSCI_A_UART
{
	union {
		unsigned int ctlw0;
		struct {
			unsigned char ctl1;
			unsigned char ctl0;
		};
	};
	unsigned int _unused_0x02;
	unsigned int _unused_0x04;
	union {
		unsigned int brw;
		struct {
			unsigned char br0;
			unsigned char br1;
		};
	};
	unsigned char mctl;
	unsigned char _reserved_0x09;
	unsigned char stat;
	unsigned char _reserved_0x0B;
	unsigned char rxbuf;
	unsigned char _reserved_0x0D;
	unsigned char txbuf;
	unsigned char _reserved_0x0F;
	unsigned char abctl;
	unsigned char _reserved_0x11;
	union {
		unsigned int irctl;
		struct {
			unsigned char irtctl;
			unsigned char irrctl;
		};
	};
	unsigned int _unused_0x14;
	unsigned int _unused_0x16;
	unsigned int _unused_0x18;
	unsigned int _unused_0x1a;
	union {
		unsigned int ictl;
		struct {
			unsigned char ie;
			unsigned char ifg;
		};
	};
	unsigned int iv;
} xUSCI_A_UART;

xComPortHandle
xSerialPortInitMinimal (unsigned long ulWantedBaud, unsigned portBASE_TYPE uxQueueLength)
{
	volatile xUSCI_A_UART* uca1 = (volatile xUSCI_A_UART*) __MSP430_BASEADDRESS_USCI_A1__;

	/* Hold the UART in reset during configuration */
	uca1->ctlw0 = UCSWRST | UCSSEL__ACLK;
	uca1->brw = 3;
	uca1->mctl = (0 * UCBRF_1) | (3 * UCBRS_1);
	P5SEL |= BIT6 | BIT7;
	uca1->ctlw0 &= ~UCSWRST;
	return 0;
}

xComPortHandle
xSerialPortInit (eCOMPort ePort, eBaud eWantedBaud, eParity eWantedParity, eDataBits eWantedDataBits, eStopBits eWantedStopBits, unsigned portBASE_TYPE uxBufferLength)
{
	return 0;
}

void
vSerialPutString (xComPortHandle pxPort, const signed char * const pcString, unsigned short usStringLength)
{
}

signed portBASE_TYPE
xSerialGetChar (xComPortHandle pxPort, signed char *pcRxedChar, portTickType xBlockTime)
{
	return -1;
}

signed portBASE_TYPE
xSerialPutChar (xComPortHandle pxPort, signed char cOutChar, portTickType xBlockTime)
{
	return -1;
}

portBASE_TYPE
xSerialWaitForSemaphore (xComPortHandle xPort)
{
	return 0;
}

void
vSerialClose (xComPortHandle xPort)
{
}

#if 0
void
checkValues ()
{
	volatile xUSCI_A_UART* uca1 = (volatile xUSCI_A_UART*) __MSP430_BASEADDRESS_USCI_A1__;
#define CHECK_VAL(_s,_v) printf("Register " #_s " at %p, field at %p\n", &_s, &(uca1->_v))

	printf("uca1 at %p\n", uca1);
	CHECK_VAL(UCA1CTLW0, ctlw0);
	CHECK_VAL(UCA1CTL0, ctl0);
	CHECK_VAL(UCA1CTL1, ctl1);
	CHECK_VAL(UCA1BRW, brw);
	CHECK_VAL(UCA1BR0, br0);
	CHECK_VAL(UCA1BR1, br1);
	CHECK_VAL(UCA1IRCTL, irctl);
	CHECK_VAL(UCA1IRTCTL, irtctl);
	CHECK_VAL(UCA1IRRCTL, irrctl);
	CHECK_VAL(UCA1ICTL, ictl);
	CHECK_VAL(UCA1IE, ie);
	CHECK_VAL(UCA1IFG, ifg);
	CHECK_VAL(UCA1IV, iv);
}
#endif
