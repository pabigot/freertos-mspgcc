/* Genericized UCSI on 5xx/6xx devices */

#ifndef BSP430_5XX_USCI_H
#define BSP430_5XX_USCI_H

#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
#include <bsp430/5xx/periph.h>

struct bsp430_FreeRTOS_USCI;

typedef struct bsp430_FreeRTOS_USCI {
	/** Flags indicating various things; primarily whether anybody is
	 * using the device. */
	unsigned int flags;
	
	/** Pointer to the peripheral register structure. */
	volatile bsp430_USCI * const usci;

	/** Queue used to collect input via interrupt.  If null,
	 * interrupts are not used for reception. */
	xQueueHandle rx_queue;

	/** Queue used to transmit output via interrupt.  If null,
	 * interrupts are not used for transmission. */
	xQueueHandle tx_queue;

	/* Semaphore to control enable of TX interrupts.  For efficient
	 * interrupt-driven transmission, we rely on receipt of TXIFG to
	 * indicate another character can be transmitted.  If there are no
	 * more characters this notification is lost.  When a new
	 * character arrives, we can't just set TXIFG to wake the ISR,
	 * because we can't tell whether the character we queued caused
	 * the transmit queue to become non-empty.  Use this semaphore as
	 * a hand-off. */
	xSemaphoreHandle tx_idle_sema;

	/* Total number of received octets */
	unsigned long num_rx;

	/* Total number of transmitted octets */
	unsigned long num_tx;
} bsp430_FreeRTOS_USCI;

/** Find the FreeRTOS USCI structure associated with the given USCI device
 *
 * @return The corresponding bsp430_FreeRTOS_USCI Device, or a null
 * pointer if there is no such device. */
bsp430_FreeRTOS_USCI*
bsp430_usci_lookup (int devid);

/** Request and configure a USCI device in UART mode.
 *
 * @param devnum The device that is being requested, e.g. BSP430_USCI_A0.
 *
 * @param control_word The configuration to be written to the device's
 * ctlw0 word.  Most bit fields will be assigned, but UCSYNC will be
 * cleared, and UCSSELx will be set based on baud rate.
 *
 * @param baud The desired baud rate.  This will be configured
 * based on the current clock setting, using ACLK if the rate is low
 * enough and SMCLK otherwise.
 *
 * @param rx_queue A references to a queue to be used for receiving.
 * A non-null value enables interrupt-driven reception, and data
 * should be read from the queue by the application.
 *
 * @param tx_queue A references to a queue to be used for
 * transmitting.  A non-null value enables interrupt-driven
 * transmission, and the application should add data to the queue for
 * transmission.
 *
 * @return A pointer to the allocated and configured USCI peripheral
 * if successful; a null pointer if something went wrong. */
bsp430_FreeRTOS_USCI*
bsp430_usci_uart_open (int devid,
					   unsigned int control_word,
					   unsigned long baud,
					   xQueueHandle rx_queue,
					   xQueueHandle tx_queue);

bsp430_FreeRTOS_USCI*
bsp430_usci_spi_open (int devid,
					  unsigned int control_word,
					  unsigned int prescaler,
					  xQueueHandle rx_queue,
					  xQueueHandle tx_queue);

int bsp430_usci_close (bsp430_FreeRTOS_USCI* device);

#endif /* BSP430_5XX_USCI_H */
