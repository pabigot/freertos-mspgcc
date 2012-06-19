#include <bsp430/5xx/usci.h>
#include <bsp430/clocks/ucs.h>

enum {
#if defined(__MSP430_HAS_USCI_A0__)
	DEVIDX_USCI_A0,
#endif /* __MSP430_HAS_USCI_A0__ */
#if defined(__MSP430_HAS_USCI_A1__)
	DEVIDX_USCI_A1,
#endif /* __MSP430_HAS_USCI_A1__ */
#if defined(__MSP430_HAS_USCI_A2__)
	DEVIDX_USCI_A2,
#endif /* __MSP430_HAS_USCI_A2__ */
#if defined(__MSP430_HAS_USCI_A3__)
	DEVIDX_USCI_A3,
#endif /* __MSP430_HAS_USCI_A3__ */
#if defined(__MSP430_HAS_USCI_B0__)
	DEVIDX_USCI_B0,
#endif /* __MSP430_HAS_USCI_B0__ */
#if defined(__MSP430_HAS_USCI_B1__)
	DEVIDX_USCI_B1,
#endif /* __MSP430_HAS_USCI_B1__ */
#if defined(__MSP430_HAS_USCI_B2__)
	DEVIDX_USCI_B2,
#endif /* __MSP430_HAS_USCI_B2__ */
#if defined(__MSP430_HAS_USCI_B3__)
	DEVIDX_USCI_B3,
#endif /* __MSP430_HAS_USCI_B3__ */
};

#define DEVID_TO_USCI(_devid) ((volatile bsp430_USCI *)(_devid))

bsp430_FreeRTOS_USCI usci_devices[] = {
#if defined(__MSP430_HAS_USCI_A0__)
	{ .usci = DEVID_TO_USCI(__MSP430_BASEADDRESS_USCI_A0__) },
#endif /* __MSP430_HAS_USCI_A0__ */
#if defined(__MSP430_HAS_USCI_A1__)
	{ .usci = DEVID_TO_USCI(__MSP430_BASEADDRESS_USCI_A1__) },
#endif /* __MSP430_HAS_USCI_A1__ */
#if defined(__MSP430_HAS_USCI_A2__)
	{ .usci = DEVID_TO_USCI(__MSP430_BASEADDRESS_USCI_A2__) },
#endif /* __MSP430_HAS_USCI_A2__ */
#if defined(__MSP430_HAS_USCI_A3__)
	{ .usci = DEVID_TO_USCI(__MSP430_BASEADDRESS_USCI_A3__) },
#endif /* __MSP430_HAS_USCI_A3__ */
#if defined(__MSP430_HAS_USCI_B0__)
	{ .usci = DEVID_TO_USCI(__MSP430_BASEADDRESS_USCI_B0__) },
#endif /* __MSP430_HAS_USCI_B0__ */
#if defined(__MSP430_HAS_USCI_B1__)
	{ .usci = DEVID_TO_USCI(__MSP430_BASEADDRESS_USCI_B1__) },
#endif /* __MSP430_HAS_USCI_B1__ */
#if defined(__MSP430_HAS_USCI_B2__)
	{ .usci = DEVID_TO_USCI(__MSP430_BASEADDRESS_USCI_B2__) },
#endif /* __MSP430_HAS_USCI_B2__ */
#if defined(__MSP430_HAS_USCI_B3__)
	{ .usci = DEVID_TO_USCI(__MSP430_BASEADDRESS_USCI_B3__) },
#endif /* __MSP430_HAS_USCI_B3__ */
};
const bsp430_FreeRTOS_USCI* const end_usci_devices = usci_devices + sizeof(usci_devices)/sizeof(*usci_devices);

bsp430_FreeRTOS_USCI*
bsp430_usci_lookup (int devid)
{
	bsp430_FreeRTOS_USCI* device = usci_devices;
	
	while (device < end_usci_devices) {
		if (device->usci == DEVID_TO_USCI(devid)) {
			return device;
		}
		++device;
	}
	return NULL;
}

#if 0

bsp430_FreeRTOS_USCI*
bsp430_usci_uart_configure (int devid,
							unsigned int control_word,
							unsigned long baud,
							xQueueHandle rx_queue,
							xQueueHandle tx_queue)
{
	unsigned long brclk_hz;
	uint16_t br;
	uint16_t brs;
	bsp430_FreeRTOS_USCI* device;

	/* Reject invalid baud rates */
	if ((0 == baud) || (1000000UL < baud)) {
		return NULL;
	}

	device = find_device(devnum);
	if (NULL == device) {
		return device;
	}

	if (0 != device->configurator && 0 != device->configurator(1)) {
		return NULL;
	}

	/* Reject if in use */
	if (NULL != device->tx_idle_sema) {
		goto failed;
	}
	vSemaphoreCreateBinary(device->tx_idle_sema);
	if (NULL != device->tx_idle_sema) {
		goto failed;
	}

	device->rx_queue = rx_queue;
	device->tx_queue = tx_queue;

	/* Prefer ACLK for rates that are low enough.  Use SMCLK for
	 * anything larger. */
	control_word &= ~((UCSYNC << 8) | (UCSSEL0 | UCSSEL1));
	if (portACLK_FREQUENCY_HZ >= (3 * baud)) {
		device->usci->ctlw0 = UCSWRST | UCSSEL__ACLK | control_word;
		brclk_hz = portACLK_FREQUENCY_HZ;
	} else {
		device->usci->ctlw0 = UCSWRST | UCSSEL__SMCLK | control_word;
		brclk_hz = ulBSP430ucsSMCLK_Hz();
	}
	br = (brclk_hz / baud);
	brs = (1 + 16 * (brclk_hz - baud * br) / baud) / 2;

	device->usci->brw = br;
	device->usci->mctl = (0 * UCBRF_1) | (brs * UCBRS_1);
	
	/* Mark the device active */
	device->num_rx = device->num_tx = 0;

	/* Release the USCI and enable the interrupts.  Interrupts are
	 * cleared when UCSWRST is set. */
	device->usci->ctlw0 &= ~UCSWRST;
	if (0 != device->rx_queue) {
		device->usci->ie |= UCRXIE;
	}
	return device;
 failed:
	if (NULL != device->tx_idle_sema) {
		vSemaphoreDelete(device->tx_idle_sema);
		device->tx_idle_sema = NULL;
	}
	if (0 != device->configurator) {
		(void)device->configurator(0);
	}
	return NULL;
}


int
bsp430_usci_set_config_function (int devid,
								 bsp430_config_fn config)
{
	bsp430_FreeRTOS_USCI* device = find_device(devnum);
	if (NULL != device) {
		device->configurator = config;
	}
	return NULL != device;
}

#if 0

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
	port->num_rx = port->num_tx = 0;
	port->flags |= COM_PORT_ACTIVE;

	/* Release the USCI and enable the interrupts.  Interrupts are
	 * cleared when UCSWRST is set. */
	port->uca->ctlw0 &= ~UCSWRST;
	if (0 != port->rx_queue) {
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
	if (0 != port->tx_queue) {
		vQueueDelete(port->tx_queue);
		port->tx_queue = 0;
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
		while (!(port->uca->ifg & UCTXIFG)) {
			;
		}
		/* Transmit the character */
		port->uca->txbuf = cOutChar;
		return pdPASS;
	}
	rv = xQueueSendToBack(port->tx_queue, &cOutChar, xBlockTime);
	if (pdTRUE == rv && xSemaphoreTake(port->tx_idle_sema, 0)) {
		port->uca->ifg |= UCTXIFG;
		port->uca->ie |= UCTXIE;
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

#endif
#endif

/* Since the interrupt code is the same for all peripherals, on MCUs
 * with multiple USCI devices it is more space efficient to share it.
 * This does add an extra call/return for some minor cost in stack
 * space.
 *
 * Making the implementation function __c16__ ensures it's legitimate
 * to use portYIELD_FROM_ISR().
 *
 * Adding __always_inline__ supports maintainability by having a
 * single implementation but speed by forcing the implementation into
 * each handler.  It's a lot cleaner than defining the body as a
 * macro.  GCC will normally inline the code if there's only one call
 * point; there should be a configPORT_foo option to do so in other
 * cases. */

static void
#if __MSP430X__
__attribute__ ( ( __c16__ ) )
#endif /* CPUX */
/* __attribute__((__always_inline__)) */
usci_irq (bsp430_FreeRTOS_USCI *port)
{
	portBASE_TYPE yield = pdFALSE;
	portBASE_TYPE rv = pdFALSE;
	uint8_t c;

	switch (port->usci->iv) {
	default:
	case USCI_NONE:
		break;
	case USCI_UCTXIFG:
		rv = xQueueReceiveFromISR(port->tx_queue, &c, &yield);
		if (0 == uxQueueMessagesWaiting(port->tx_queue)) {
			signed portBASE_TYPE sema_yield = pdFALSE;
			port->usci->ie &= ~UCTXIE;
			xSemaphoreGiveFromISR(port->tx_idle_sema, &sema_yield);
			yield |= sema_yield;
		}
		if (rv) {
			++port->num_tx;
			port->usci->txbuf = c;
		}
		break;
	case USCI_UCRXIFG:
		c = port->usci->rxbuf;
		++port->num_rx;
		rv = xQueueSendToFrontFromISR(port->rx_queue, &c, &yield);
		break;
	}
	portYIELD_FROM_ISR(yield);
}

/* No current MCU has more than 4 USCI_B instances */

#if defined(__MSP430_HAS_USCI_A0__)
static void
__attribute__((__interrupt__(USCI_A0_VECTOR)))
usci_a0_irq (void)
{
	usci_irq(usci_devices + DEVIDX_USCI_A0);
}
#endif /* __MSP430_HAS_USCI_A0__ */
#if defined(__MSP430_HAS_USCI_A1__)
static void
__attribute__((__interrupt__(USCI_A1_VECTOR)))
usci_a1_irq (void)
{
	usci_irq(usci_devices + DEVIDX_USCI_A1);
}
#endif /* __MSP430_HAS_USCI_A1__ */
#if defined(__MSP430_HAS_USCI_A2__)
static void
__attribute__((__interrupt__(USCI_A2_VECTOR)))
usci_a2_irq (void)
{
	usci_irq(usci_devices + DEVIDX_USCI_A2);
}
#endif /* __MSP430_HAS_USCI_A2__ */
#if defined(__MSP430_HAS_USCI_A3__)
static void
__attribute__((__interrupt__(USCI_A3_VECTOR)))
usci_a3_irq (void)
{
	usci_irq(usci_devices + DEVIDX_USCI_A3);
}
#endif /* __MSP430_HAS_USCI_A3__ */
#if defined(__MSP430_HAS_USCI_B0__)
static void
__attribute__((__interrupt__(USCI_B0_VECTOR)))
usci_b0_irq (void)
{
	usci_irq(usci_devices + DEVIDX_USCI_B0);
}
#endif /* __MSP430_HAS_USCI_B0__ */
#if defined(__MSP430_HAS_USCI_B1__)
static void
__attribute__((__interrupt__(USCI_B1_VECTOR)))
usci_b1_irq (void)
{
	usci_irq(usci_devices + DEVIDX_USCI_B1);
}
#endif /* __MSP430_HAS_USCI_B1__ */
#if defined(__MSP430_HAS_USCI_B2__)
static void
__attribute__((__interrupt__(USCI_B2_VECTOR)))
usci_b2_irq (void)
{
	usci_irq(usci_devices + DEVIDX_USCI_B2);
}
#endif /* __MSP430_HAS_USCI_B2__ */
#if defined(__MSP430_HAS_USCI_B3__)
static void
__attribute__((__interrupt__(USCI_B3_VECTOR)))
usci_b3_irq (void)
{
	usci_irq(usci_devices + DEVIDX_USCI_B3);
}
#endif /* __MSP430_HAS_USCI_B3__ */
