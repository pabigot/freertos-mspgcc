#include "platform.h"
#include "serial.h"

xComPortHandle
xSerialPortInitMinimal (unsigned long ulWantedBaud, unsigned portBASE_TYPE uxQueueLength)
{
	/* Hold the UART in reset during configuration */
	UCA1CTL1 |= UCSWRST;
	UCA1CTLW0 = UCSWRST | UCSSEL__ACLK;
	UCA1BRW = 3;
	UCA1MCTL = (0 * UCBRF_1) | (3 * UCBRS_1);
	P5SEL |= BIT6 | BIT7;
	UCA1CTL1 &= ~UCSWRST;
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
