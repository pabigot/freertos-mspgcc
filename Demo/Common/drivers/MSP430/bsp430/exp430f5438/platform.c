#include "platform.h"
#include "clocks/ucs.h"
#include "timers/timerA0.h"
#include "utility/led.h"

const xLEDDefn pxLEDDefn[] = {
	{ .pucPxOUT = &P1OUT, .ucBIT = BIT0 }, /* Red */
	{ .pucPxOUT = &P1OUT, .ucBIT = BIT1 }, /* Orange */
};
const unsigned char ucLEDDefnCount = sizeof(pxLEDDefn) / sizeof(*pxLEDDefn);

static void prvSetupConsole( void )
{
	/* Hold the UART in reset during configuration */
	UCA1CTL1 |= UCSWRST;
	UCA1CTLW0 = UCSWRST | UCSSEL__ACLK;
	UCA1BRW = 3;
	UCA1MCTL = (0 * UCBRF_1) | (3 * UCBRS_1);
	P5SEL |= BIT6 | BIT7;
	UCA1CTL1 &= ~UCSWRST;
}

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

	/* Enable XT1 functions */
	P7SEL |= BIT0;

	ulBSP430ucsConfigure( configCPU_CLOCK_HZ, -1 );
	vBSP430timerA0Configure();
	
	prvSetupConsole();
}
