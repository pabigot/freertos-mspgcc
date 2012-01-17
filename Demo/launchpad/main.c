/*
  
Copyright (c) 2012, Peter A. Bigot <bigotp@acm.org>

All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice,
  this list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the software nor the names of its contributors may be
  used to endorse or promote products derived from this software without
  specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.

*/

#include "FreeRTOS.h"
#include "task.h"
#include "croutine.h"

/* The number of flash co-routines to create. */
#define mainNUM_FLASH_CO_ROUTINES       ( 2 )

static void prvSetupHardware( void );

void main( void )
{
	unsigned portBASE_TYPE uxCounter;
	
	prvSetupHardware();
	vParTestInitialise();

	vStartFlashCoRoutines( mainNUM_FLASH_CO_ROUTINES );

	/* Start the scheduler. */
	vTaskStartScheduler();
	vParTestSetLED( 0, 1 );
	__disable_interrupt();
	while (1) {
		P1OUT ^= BIT0 | BIT6;
		__delay_cycles(50000);
	}
}

void vApplicationIdleHook( void )
{
	/* Schedule the co-routines. */
	for( ;; )
	{
		vCoRoutineSchedule();
	}
}

static void prvSetupHardware( void )
{
	//	xFLLPLUSDefn xFLLPlus;
	
	WDTCTL = WDTPW + WDTHOLD;
#if 0
	/* Most examples use XCAP14PF, but my crude tests suggest that of
	 * the capacitances available 0pF produces the closest to 32768
	 * Hz. */
	xFLLPlus.ucFLL_CTL0 = DCOPLUS | XCAP0PF;
	/* Retain power-up default in this case */
	xFLLPlus.ucFLL_CTL1 = XT2OFF;
	/* Configure for 2 * (121 + 1) * 32768 = 7995392 Hz */
	xFLLPlus.ucSCFI0 = FLLD_2 | FN_4;
	xFLLPlus.ucSCFQCTL = 121;
	( void ) ucBSP430fllplusConfigure( &xFLLPlus );

	/* For verification, bring out the clock signals: ACLK on P1.5
	 * (H2.6), MCLK on P1.1(H2.2), SMCLK on P1.4 (H2.5). */
	P1SEL |= BIT1 | BIT4 | BIT5;
	P1DIR |= BIT1 | BIT4 | BIT5;
#endif
}

#include "utility/led.h"

const xLEDDefn pxLEDDefn[] = {
	{ .pucPxOUT = &P1OUT, .ucBIT = BIT0 }, /* Red */
	{ .pucPxOUT = &P1OUT, .ucBIT = BIT6 }, /* Green */
};
const unsigned char ucLEDDefnCount = sizeof(pxLEDDefn) / sizeof(*pxLEDDefn);

