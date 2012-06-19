#include "platform.h"
#include "portSerial.h"
#include "queue.h"
#include "semphr.h"
#include <bsp430/clocks/ucs.h>
#include <bsp430/5xx/usci.h>

typedef bsp430_USCI xUSCI_A_UART;

#define COM_PORT_ACTIVE  0x01

typedef bsp430_FreeRTOS_USCI xComPort;

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
	
	/* Reject if requested queue could not be allocated */
	if (0 < bufsiz) {
		port->rx_queue = xQueueCreate(bufsiz, sizeof(uint8_t));
		if (NULL == port->rx_queue) {
			return NULL;
		}
		port->tx_queue = xQueueCreate(bufsiz, sizeof(uint8_t));
		if (NULL == port->tx_queue) {
			vQueueDelete(port->rx_queue);
			port->rx_queue = 0;
			return NULL;
		}
		vSemaphoreCreateBinary(port->tx_idle_sema);
		if (NULL == port->tx_idle_sema) {
			vQueueDelete(port->tx_queue);
			port->tx_queue = 0;
			vQueueDelete(port->rx_queue);
			port->rx_queue = 0;
			return NULL;
		}
	}

	/* Prefer ACLK for rates that are low enough.  Use SMCLK for
	 * anything larger. */
	if (portACLK_FREQUENCY_HZ >= (3 * baud)) {
		port->usci->ctlw0 = UCSWRST | UCSSEL__ACLK;
		brclk_hz = portACLK_FREQUENCY_HZ;
	} else {
		port->usci->ctlw0 = UCSWRST | UCSSEL__SMCLK;
		brclk_hz = ulBSP430ucsSMCLK_Hz();
	}
	br = (brclk_hz / baud);
	brs = (1 + 16 * (brclk_hz - baud * br) / baud) / 2;

	port->usci->brw = br;
	port->usci->mctl = (0 * UCBRF_1) | (brs * UCBRS_1);

	vBSP430platformConfigurePeripheralPins ((int)(port->usci), 1);

	/* Mark the port active */
	port->num_rx = port->num_tx = 0;
	port->flags |= COM_PORT_ACTIVE;

	/* Release the USCI and enable the interrupts.  Interrupts are
	 * cleared when UCSWRST is set. */
	port->usci->ctlw0 &= ~UCSWRST;
	if (0 != port->rx_queue) {
		port->usci->ie |= UCRXIE;
	}

	return port;
}

static void
unconfigurePort_ (xComPort* port)
{
	port->usci->ctlw0 = UCSWRST;
	vBSP430platformConfigurePeripheralPins ((int)(port->usci), 0);
	if (0 != port->rx_queue) {
		vQueueDelete(port->rx_queue);
		port->rx_queue = 0;
	}
	if (0 != port->tx_queue) {
		vQueueDelete(port->tx_queue);
		port->tx_queue = 0;
	}
	port->flags = 0;
}

static xComPort*
portToDevice (eCOMPort ePort)
{
	int devid;
	xComPort* port;

	switch (ePort) {
	case serCOM1: devid = BSP430_USCI_A0; break;
	case serCOM2: devid = BSP430_USCI_A1; break;
	case serCOM3: devid = BSP430_USCI_A2; break;
	case serCOM4: devid = BSP430_USCI_A3; break;
	default:
		devid = 0;
		break;
	}
	return bsp430_usci_lookup(devid);
}


xComPortHandle
xSerialPortInitMinimal (unsigned long ulWantedBaud, unsigned portBASE_TYPE uxQueueLength)
{
	extern bsp430_FreeRTOS_USCI usci_devices[];
	extern const bsp430_FreeRTOS_USCI* const end_usci_devices;
	
	xComPort *port = usci_devices;
	/* Locate an unused port */
	while (port < end_usci_devices && (port->flags & COM_PORT_ACTIVE)) {
		++port;
	}
	if (end_usci_devices == port) {
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
	xComPort* port = portToDevice(ePort);
	if (! port) {
		return NULL;
	}
	if (port->flags & COM_PORT_ACTIVE) {
		unconfigurePort_(port);
	}
	port = configurePort_(port, prvBaudEnumToValue(eWantedBaud), uxBufferLength);
	return (xComPortHandle)port;
}

void
vSerialPutString (xComPortHandle pxPort, const signed char * const pcString, unsigned short usStringLength)
{
	unsigned short i;
	for (i = 0; i < usStringLength; ++i) {
		xSerialPutChar(pxPort, pcString[i], 0);
	}
}

signed portBASE_TYPE
xSerialGetChar (xComPortHandle pxPort, signed char *pcRxedChar, portTickType xBlockTime)
{
	xComPort* port = (xComPort*)pxPort;

	if (NULL == port) {
		return pdFAIL;
	}
	if (0 == port->rx_queue) {
		return pdFAIL;
	}
	return xQueueReceive (port->rx_queue, pcRxedChar, xBlockTime);
}

signed portBASE_TYPE
xSerialPutChar (xComPortHandle pxPort, signed char cOutChar, portTickType xBlockTime)
{
	xComPort* port = (xComPort*)pxPort;
	portBASE_TYPE rv;

	if (NULL == port) {
		return pdFAIL;
	}
	if (0 == port->tx_queue) {
		/* Spin until tx buffer ready */
		while (!(port->usci->ifg & UCTXIFG)) {
			;
		}
		/* Transmit the character */
		port->usci->txbuf = cOutChar;
		return pdPASS;
	}
	rv = xQueueSendToBack(port->tx_queue, &cOutChar, xBlockTime);
	if (pdTRUE == rv && xSemaphoreTake(port->tx_idle_sema, 0)) {
		port->usci->ifg |= UCTXIFG;
		port->usci->ie |= UCTXIE;
	}
	return pdTRUE == rv;
}

long ulSerialNumRx (xComPortHandle xPort)
{
	xComPort* port = (xComPort*)xPort;
	return port ? port->num_rx : -1L;
}

long ulSerialNumTx (xComPortHandle xPort)
{
	xComPort* port = (xComPort*)xPort;
	return port ? port->num_tx : -1L;
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

