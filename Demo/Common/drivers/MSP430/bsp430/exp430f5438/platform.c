#include "platform.h"
#include "clocks/ucs.h"
#include "timers/timerA0.h"
#include "utility/led.h"
#include "serial.h"
#include "portSerial.h"

const xLEDDefn pxLEDDefn[] = {
	{ .pucPxOUT = &P1OUT, .ucBIT = BIT0 }, /* Red */
	{ .pucPxOUT = &P1OUT, .ucBIT = BIT1 }, /* Orange */
};
const unsigned char ucLEDDefnCount = sizeof(pxLEDDefn) / sizeof(*pxLEDDefn);

static xComPortHandle console;

int
putchar (int c)
{
	if (pdPASS == xSerialPutChar(console, c, 0)) {
		return c;
	}
	return -1;
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

	/* Enable console */
	portSerialAssignPins(serCOM1, &P3SEL, BIT4, BIT5);
	portSerialAssignPins(serCOM2, &P5SEL, BIT6, BIT7);
	portSerialAssignPins(serCOM3, &P9SEL, BIT4, BIT5);
	portSerialAssignPins(serCOM4, &P10SEL, BIT4, BIT5);
	console = xSerialPortInit(serCOM2, ser9600, serNO_PARITY, serBITS_8, serSTOP_1, 0);
}
