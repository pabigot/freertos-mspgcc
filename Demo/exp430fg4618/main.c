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

#define mainLED_TASK_PRIORITY ( tskIDLE_PRIORITY + 1 )

static void prvSetupHardware( void );

void main( void )
{
	unsigned portBASE_TYPE uxCounter;
	
	prvSetupHardware();
	vParTestInitialise();

	vStartLEDFlashTasks( mainLED_TASK_PRIORITY );

	/* Start the scheduler. */
	vTaskStartScheduler();
}

void vApplicationIdleHook( void ) { }

static void prvSetupHardware( void )
{
	WDTCTL = WDTPW + WDTHOLD;
}

#include "utility/led.h"

const xLEDDefn pxLEDDefn[] = {
	{ .pucPxOUT = &P5OUT, .ucBIT = BIT1 }, /* Red */
	{ .pucPxOUT = &P2OUT, .ucBIT = BIT2 }, /* Green */
	{ .pucPxOUT = &P2OUT, .ucBIT = BIT1 }, /* Yellow */
};
const unsigned char ucLEDDefnCount = sizeof(pxLEDDefn) / sizeof(*pxLEDDefn);

