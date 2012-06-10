#include "platform.h"
#include "clocks/ucs.h"
#include "timers/timerA0.h"
#include "utility/led.h"
#include "serial.h"

const xLEDDefn pxLEDDefn[] = {
	{ .pucPxOUT = &P1OUT, .ucBIT = BIT0 }, /* Red */
	{ .pucPxOUT = &P1OUT, .ucBIT = BIT1 }, /* Orange */
};
const unsigned char ucLEDDefnCount = sizeof(pxLEDDefn) / sizeof(*pxLEDDefn);

static xComPortHandle console;

int
putchar (int c)
{
  /* Spin until tx buffer ready */
  while (!(UCA1IFG & UCTXIFG)) {
    ;
  }
  /* Transmit the character */
  UCA1TXBUF = c;

  return c;
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
	console = xSerialPortInitMinimal(0, 0);
}
