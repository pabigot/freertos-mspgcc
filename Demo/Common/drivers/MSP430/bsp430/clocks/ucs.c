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

#include "clocks/ucs.h"
#include <stdint.h>

unsigned long ulBSP430ucsConfigure ( unsigned long ulFrequency_Hz,
									 short sRSEL )
{
	static const unsigned long pulRSELCutoffs [] = {
#if defined(__MSP430F5438__) || defined(__MSP430F5438A__)
		400000UL, 				/* RSEL0 */
		800000UL, 				/* RSEL1 */
		2000000UL,				/* RSEL2 */
		4000000UL,				/* RSEL3 */
		8000000UL,				/* RSEL4 */
		16000000UL,				/* RSEL5 */
#else
#endif
		UINT32_MAX
	};
	unsigned portBASE_TYPE ctl1;
	unsigned portBASE_TYPE ctl2;
	unsigned portBASE_TYPE ctl3;
	
	if( 0 > sRSEL )
	{
		sRSEL = 0;
		while( pulRSELCutoffs[ sRSEL ] < ulFrequency_Hz )
		{
			++sRSEL;
		}
	}

	portENTER_CRITICAL();

	/* Low frequency XT1 needed; XT2 off.  Spin at high drive to
	   stability, then drop back. */
	UCSCTL6 = XT2OFF | XT1DRIVE_3 | XCAP_0;
	do {
		UCSCTL7 &= ~XT1LFOFFG;
	} while (UCSCTL7 & XT1LFOFFG);
	UCSCTL6 &= ~XT1DRIVE_3;

	/* All supported frequencies can be efficiently achieved using
	 * FFLD set to /2 (>> 1) and FLLREFDIV set to /1 (>> 0).
	 * FLLREFCLK will always be XT1CLK.  FLLN is calculated from
	 * ulFrequency_Hz. */
	ctl1 = sRSEL * DCORSEL0;
	ctl2 = FLLD_1 | ((((ulFrequency_Hz << 1) / (32768 >> 0)) >> 1) - 1);

	TA0CCTL1 = 0;

	__bis_status_register(SCG0);
	UCSCTL0 = 0;
	UCSCTL1 = ctl1;
	UCSCTL2 = ctl2;
	UCSCTL3 = SELREF__XT1CLK | FLLREFDIV_0;
	__bic_status_register(SCG0);
	TA0CCR1 = TA0R + 31 * 32;

	UCSCTL4 = SELA__XT1CLK | SELS__DCOCLKDIV | SELM__DCOCLKDIV;

	while(! ( TA0CCTL1 & CCIFG ) )
	{
		/* nop */
	}
	/* Spin until DCO stabilized */
	do {
		UCSCTL7 &= ~(XT2OFFG + XT1LFOFFG + XT1HFOFFG + DCOFFG);
		SFRIFG1 &= ~OFIFG;
	} while (UCSCTL7 & DCOFFG);

	/* Turn off FLL */
	__bis_status_register(SCG0);
	
	portEXIT_CRITICAL();
}
