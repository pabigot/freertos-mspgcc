#include "platform.h"
#include "portSerial.h"
#include "queue.h"
#include <clocks/ucs.h>

/* Register map for USCI_Ax peripheral in UART mode.  This
 * intentionally uses the GCC/ISO C11 extensions for unnamed
 * struct/union fields. */
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

#define COM_PORT_ACTIVE  0x01

typedef struct xComPort {
	unsigned int flags;
	volatile xUSCI_A_UART * const uca;
	xQueueHandle rx_queue;
	volatile unsigned char * pxsel;
	unsigned char bit_tx;
	unsigned char bit_rx;
} xComPort;

static xComPort prvComPorts[] = {
#if defined(__MSP430_HAS_USCI_A0__)
	{ .uca = (volatile xUSCI_A_UART *)__MSP430_BASEADDRESS_USCI_A0__ },
#endif /* __MSP430_HAS_USCI_A0__ */
#if defined(__MSP430_HAS_USCI_A1__)
	{ .uca = (volatile xUSCI_A_UART *)__MSP430_BASEADDRESS_USCI_A1__ },
#endif /* __MSP430_HAS_USCI_A1__ */
#if defined(__MSP430_HAS_USCI_A2__)
	{ .uca = (volatile xUSCI_A_UART *)__MSP430_BASEADDRESS_USCI_A2__ },
#endif /* __MSP430_HAS_USCI_A2__ */
#if defined(__MSP430_HAS_USCI_A3__)
	{ .uca = (volatile xUSCI_A_UART *)__MSP430_BASEADDRESS_USCI_A3__ },
#endif /* __MSP430_HAS_USCI_A3__ */
};

static const eCOMPort prvCOMPortInvalid = (eCOMPort)(sizeof(prvComPorts)/sizeof(*prvComPorts));
static const xComPort * const prvComPorts_end = prvComPorts + sizeof(prvComPorts)/sizeof(*prvComPorts);

static xComPort*
configurePort_ (xComPort* port,
				unsigned long baud,
				size_t bufsiz)
{
	unsigned long brclk_hz;
	uint16_t br;
	uint16_t brs;

	/* Reject invalid baud rates */
	if ((0 == baud) || (1000000UL < baud)) {
		return NULL;
	}
	
	/* Reject if platform did not call portSerialAssignPins. */
	if (! port->pxsel) {
		return NULL;
	}

	/* Reject if requested queue could not be allocated */
	if (0 < bufsiz) {
		port->rx_queue = xQueueCreate(bufsiz, sizeof(uint8_t));
		if (NULL == port->rx_queue) {
			return NULL;
		}
	}

	/* Prefer ACLK for rates that are low enough.  Use SMCLK for
	 * anything larger. */
	if (portACLK_FREQUENCY_HZ >= (3 * baud)) {
		port->uca->ctlw0 = UCSWRST | UCSSEL__ACLK;
		brclk_hz = portACLK_FREQUENCY_HZ;
	} else {
		port->uca->ctlw0 = UCSWRST | UCSSEL__SMCLK;
		brclk_hz = ulBSP430ucsSMCLK_Hz();
	}
	br = (brclk_hz / baud);
	brs = (1 + 16 * (brclk_hz - baud * br) / baud) / 2;

	port->uca->brw = br;
	port->uca->mctl = (0 * UCBRF_1) | (brs * UCBRS_1);

	*(port->pxsel) |= port->bit_tx | port->bit_rx;

	/* Mark the port active */
	port->flags |= COM_PORT_ACTIVE;

	/* Release the UART */
	port->uca->ctlw0 &= ~UCSWRST;

	if (serCOM2 != (port - prvComPorts)) {
		port->uca->ie |= UCRXIE;
	}

	return port;
}

static void
unconfigurePort_ (xComPort* port)
{
	port->uca->ctlw0 = UCSWRST;
	*(port->pxsel) &= ~(port->bit_tx | port->bit_rx);
	if (0 != port->rx_queue) {
		vQueueDelete(port->rx_queue);
		port->rx_queue = 0;
	}
	port->flags = 0;
}

signed portBASE_TYPE
portSerialAssignPins (eCOMPort ePort,
					  volatile unsigned char * pcPxSEL,
					  unsigned char ucBitTX,
					  unsigned char ucBitRX)
{
	xComPort* port;
	if (ePort >= prvCOMPortInvalid) {
		return pdFAIL;
	}
	port = prvComPorts + (unsigned int)ePort;
	port->pxsel = pcPxSEL;
	port->bit_tx = ucBitTX;
	port->bit_rx = ucBitRX;
	return pdPASS;
}

xComPortHandle
xSerialPortInitMinimal (unsigned long ulWantedBaud, unsigned portBASE_TYPE uxQueueLength)
{
	xComPort *port;

	/* Locate an unused port */
	port = prvComPorts;
	while (port < prvComPorts_end && (port->flags & COM_PORT_ACTIVE)) {
		++port;
	}
	if (prvComPorts_end == port) {
		return NULL;
	}

	port = configurePort_(port, ulWantedBaud, uxQueueLength);
	return (xComPortHandle)port;
}

static unsigned long
prvBaudEnumToValue (eBaud baud_enum)
{
	switch (baud_enum) {
	case ser50: return 50;
	case ser75: return 75;
	case ser110: return 110;
	case ser134: return 134;
	case ser150: return 150;
	case ser200: return 200;
	case ser300: return 300;
	case ser600: return 600;
	case ser1200: return 1200;
	case ser1800: return 1800;
	case ser2400: return 2400;
	case ser4800: return 4800;
	case ser9600: return 9600;
	case ser19200: return 19200;
	case ser38400: return 38400;
	case ser57600: return 57600;
	case ser115200: return 115200;
	default: return 0;
	}
	return 0;
}

xComPortHandle
xSerialPortInit (eCOMPort ePort, eBaud eWantedBaud, eParity eWantedParity, eDataBits eWantedDataBits, eStopBits eWantedStopBits, unsigned portBASE_TYPE uxBufferLength)
{
	xComPort* port;
	if (ePort >= prvCOMPortInvalid) {
		return NULL;
	}
	port = prvComPorts + (unsigned int)ePort;
	if (port->flags & COM_PORT_ACTIVE) {
		unconfigurePort_(port);
	}
	port = configurePort_(port, prvBaudEnumToValue(eWantedBaud), uxBufferLength);
	return (xComPortHandle)port;
}

void
vSerialPutString (xComPortHandle pxPort, const signed char * const pcString, unsigned short usStringLength)
{
}

signed portBASE_TYPE
xSerialGetChar (xComPortHandle pxPort, signed char *pcRxedChar, portTickType xBlockTime)
{
	xComPort* port = (xComPort*)pxPort;

	if (NULL == port) {
		return pdFAIL;
	}
	if (serCOM2 == (port - prvComPorts)) {
		return pdFAIL;
	}
	return xQueueReceive (port->rx_queue, pcRxedChar, xBlockTime);
}

signed portBASE_TYPE
xSerialPutChar (xComPortHandle pxPort, signed char cOutChar, portTickType xBlockTime)
{
	xComPort* port = (xComPort*)pxPort;

	if (NULL == port) {
		return pdFAIL;
	}
	/* Spin until tx buffer ready */
	while (!(port->uca->ifg & UCTXIFG)) {
		;
	}
	/* Transmit the character */
	port->uca->txbuf = cOutChar;

	return pdPASS;
}

portBASE_TYPE
xSerialWaitForSemaphore (xComPortHandle xPort)
{
	return 0;
}

void
vSerialClose (xComPortHandle xPort)
{
	xComPort* port = (xComPort*)xPort;
	if (NULL != xPort) {
		unconfigurePort_(port);
	}
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

static void
__attribute__((__interrupt__(USCI_A0_VECTOR)))
usci_a0_irq (void)
{
	portBASE_TYPE xHigherPriorityTaskWoken;
	portBASE_TYPE rv;
	uint8_t c;
	xComPort* port = prvComPorts + 0;

	switch (port->uca->iv) {
	default:
	case USCI_NONE:
	case USCI_UCTXIFG:
		break;
	case USCI_UCRXIFG:
		c = port->uca->rxbuf;
		rv = xQueueSendToFrontFromISR(port->rx_queue, &c, &xHigherPriorityTaskWoken);
		if ((pdPASS == rv) && (pdTRUE == xHigherPriorityTaskWoken)) {
			portYIELD();
		}
		break;
	}
}
