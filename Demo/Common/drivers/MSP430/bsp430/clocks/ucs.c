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

/* Standard procedure on most UCS implementations is to leave the FLL
 * disabled, and enable it periodically to adapt to changed
 * conditions.  */

#ifndef portUCS_TRIM_TOLERANCE_DIVISOR
#define portUCS_TRIM_TOLERANCE_DIVISOR 128
#endif /* portUCS_TRIM_TOLERANCE_DIVISOR */

static unsigned long ulFrequency_Hz_;

unsigned long ulBSP430ucsTrimFLL ()
{
	short taps_left = 32;
	unsigned short last_ctl0;
	unsigned short cd;
	unsigned short usFrequency_aclk = (ulFrequency_Hz_ / 32768);
	unsigned short usTolerance_aclk = (usFrequency_aclk / portUCS_TRIM_TOLERANCE_DIVISOR);

	if (0 == usTolerance_aclk) {
		++usTolerance_aclk;
	}
	last_ctl0 = ~0;
	while( 0 < taps_left--) {
		unsigned short c0;
		unsigned short c1;
		short sError_aclk;

		vBSP430ledSet(0, 1);
		/* Capture the SMCLK ticks between adjacent ACLK ticks */
		TB0CTL = TASSEL__SMCLK | MC__CONTINOUS | TBCLR;
		TB0CCTL6 = CM_2 | CCIS_1 | CAP | SCS;
		/* NOTE: CCIFG seems to be set immediately on the second and
		 * subsequent iterations.  Flush the first capture. */
        while (! (TB0CCTL6 & CCIFG)) {
          ; /* nop */
        }
        TB0CCTL6 &= ~CCIFG;
        while (! (TB0CCTL6 & CCIFG)) {
          ; /* nop */
        }
        c0 = TB0CCR6;
        TB0CCTL6 &= ~CCIFG;
        while (! (TB0CCTL6 & CCIFG)) {
          ; /* nop */
        }
        c1 = TB0CCR6;
		TB0CTL = 0;
		TB0CCTL6 = 0;
		cd = (c0 > c1) ? (c0 - c1) : (c1 - c0);
#if 0
		printf("c0=%u c1=%u cd=%u DCO=%u RSEL=%u\n", c0, c1, cd, (UCSCTL0 >> 8) & 0x1F, (UCSCTL0 >> 3) & 0x1F);
#endif
		sError_aclk = ( usFrequency_aclk > cd ) ? ( usFrequency_aclk - cd ) : ( cd - usFrequency_aclk );
		vBSP430ledSet(0, 0);
		if( ( sError_aclk <= usTolerance_aclk )
			|| ( UCSCTL0 == last_ctl0 ) ) {
			break;
		}
		/* Save current DCO/MOD values, then let FLL run for 32 REFCLK
		 * ticks (potentially trying each modulation within one
		 * tap) */
		vBSP430ledSet(1, 1);
		last_ctl0 = UCSCTL0;
		TB0CTL = TASSEL__ACLK | MC__CONTINOUS | TBCLR;
		__bic_status_register(SCG0);
		TB0CCTL0 = 0;
		TB0CCR0 = TB0R + 32;
		while( ! (TB0CCTL0 & CCIFG ) ) {
			/* nop */
		}
		__bis_status_register(SCG0);
		vBSP430ledSet(1, 0);
		TB0CTL = 0;
		TB0CCTL0 = 0;
	}
	return (cd * 32768UL);
}

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
	unsigned long ulReturn;
	
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

	__bis_status_register(SCG0);
	UCSCTL0 = 0;
	UCSCTL1 = ctl1;
	UCSCTL2 = ctl2;
	UCSCTL3 = SELREF__XT1CLK | FLLREFDIV_0;
	UCSCTL4 = SELA__XT1CLK | SELS__DCOCLKDIV | SELM__DCOCLKDIV;

	ulFrequency_Hz_ = ulFrequency_Hz;
	ulReturn = ulBSP430ucsTrimFLL();

	/* Spin until DCO stabilized */
	do {
		UCSCTL7 &= ~(XT2OFFG + XT1LFOFFG + XT1HFOFFG + DCOFFG);
		SFRIFG1 &= ~OFIFG;
	} while (UCSCTL7 & DCOFFG);

#if ! defined(portDISABLE_FLL)
	/* Turn FLL back on */
	__bic_status_register(SCG0);
#endif
	portEXIT_CRITICAL();

	return ulReturn;
}
