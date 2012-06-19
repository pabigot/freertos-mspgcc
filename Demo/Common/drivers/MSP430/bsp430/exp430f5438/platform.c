#include "platform.h"
#include <bsp430/clocks/ucs.h>
#include <bsp430/timers/timerA0.h>
#include <bsp430/utility/led.h>
#include <bsp430/utility/console.h>
#include "serial.h"

const xLEDDefn pxLEDDefn[] = {
	{ .pucPxOUT = &P1OUT, .ucBIT = BIT0 }, /* Red */
	{ .pucPxOUT = &P1OUT, .ucBIT = BIT1 }, /* Orange */
};
const unsigned char ucLEDDefnCount = sizeof(pxLEDDefn) / sizeof(*pxLEDDefn);

int
exp430f5438_uart_a0_config (int enabledp)
{
	const int bits = BIT4 | BIT5;
	if (enabledp) {
		P3SEL |= bits;
	} else {
		P3SEL &= ~bits;
	}
	return 0;
}

int
exp430f5438_uart_a1_config (int enabledp)
{
	const int bits = BIT6 | BIT7;
	if (enabledp) {
		P5SEL |= bits;
	} else {
		P5SEL &= ~bits;
	}
	return 0;
}

int
exp430f5438_uart_a2_config (int enabledp)
{
	const int bits = BIT4 | BIT5;
	if (enabledp) {
		P9SEL |= bits;
	} else {
		P9SEL &= ~bits;
	}
	return 0;
}

int
exp430f5438_uart_a3_config (int enabledp)
{
	const int bits = BIT4 | BIT5;
	if (enabledp) {
		P10SEL |= bits;
	} else {
		P10SEL &= ~bits;
	}
	return 0;
}

void vBSP430platformSetup ()
{
	/* Hold off watchdog */
	WDTCTL = WDTPW + WDTHOLD;

	/* Enable XT1 functions and clock */
	P7SEL |= BIT0;
	ulBSP430ucsConfigure( configCPU_CLOCK_HZ, -1 );

	/* Enable basic timer */
	vBSP430timerA0Configure();

	/* Set up MCU-specific hardware for USCI */
	portSerialAssignPins(serCOM1, exp430f5438_uart_a0_config);
	portSerialAssignPins(serCOM2, exp430f5438_uart_a1_config);
	portSerialAssignPins(serCOM3, exp430f5438_uart_a2_config);
	portSerialAssignPins(serCOM4, exp430f5438_uart_a3_config);

	/* Enable console */
	xConsoleConfigure(xSerialPortInit(serCOM2, ser9600, serNO_PARITY, serBITS_8, serSTOP_1, 0),
					  500);
}
