/*
    FreeRTOS V7.1.0 - Copyright (C) 2011 Real Time Engineers Ltd.
	

    ***************************************************************************
     *                                                                       *
     *    FreeRTOS tutorial books are available in pdf and paperback.        *
     *    Complete, revised, and edited pdf reference manuals are also       *
     *    available.                                                         *
     *                                                                       *
     *    Purchasing FreeRTOS documentation will not only help you, by       *
     *    ensuring you get running as quickly as possible and with an        *
     *    in-depth knowledge of how to use FreeRTOS, it will also help       *
     *    the FreeRTOS project to continue with its mission of providing     *
     *    professional grade, cross platform, de facto standard solutions    *
     *    for microcontrollers - completely free of charge!                  *
     *                                                                       *
     *    >>> See http://www.FreeRTOS.org/Documentation for details. <<<     *
     *                                                                       *
     *    Thank you for using FreeRTOS, and thank you for your support!      *
     *                                                                       *
    ***************************************************************************


    This file is part of the FreeRTOS distribution.

    FreeRTOS is free software; you can redistribute it and/or modify it under
    the terms of the GNU General Public License (version 2) as published by the
    Free Software Foundation AND MODIFIED BY the FreeRTOS exception.
    >>>NOTE<<< The modification to the GPL is included to allow you to
    distribute a combined work that includes FreeRTOS without being obliged to
    provide the source code for proprietary components outside of the FreeRTOS
    kernel.  FreeRTOS is distributed in the hope that it will be useful, but
    WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
    or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
    more details. You should have received a copy of the GNU General Public
    License and the FreeRTOS license exception along with FreeRTOS; if not it
    can be viewed here: http://www.freertos.org/a00114.html and also obtained
    by writing to Richard Barry, contact details for whom are available on the
    FreeRTOS WEB site.

    1 tab == 4 spaces!

    http://www.FreeRTOS.org - Documentation, latest information, license and
    contact details.

    http://www.SafeRTOS.com - A version that is certified for use in safety
    critical systems.

    http://www.OpenRTOS.com - Commercial support, development, porting,
    licensing and training services.
*/

/*
	Changes from V2.5.2
		
	+ usCriticalNesting now has a volatile qualifier.
*/

/* Standard includes. */
#include <stdlib.h>
#include <msp430.h>

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"

/*-----------------------------------------------------------
 * Implementation of functions defined in portable.h for the MSP430 port.
 *----------------------------------------------------------*/

/* Constants required for hardware setup.  The tick ISR runs off the ACLK, 
not the MCLK. */
#define portACLK_FREQUENCY_HZ			( ( portTickType ) 32768 )
#define portINITIAL_CRITICAL_NESTING	( ( unsigned short ) 10 )
#if portDISABLE_FLL
#define portTASK_INITIAL_R2	( ( portSTACK_TYPE ) ( GIE | SCG0 ) )
#else /* portDISABLE_FLL */
#define portTASK_INITIAL_R2 ( ( portSTACK_TYPE ) GIE )
#endif /* portDISABLE_FLL */

/* We require the address of the pxCurrentTCB variable, but don't want to know
any details of its type. */
typedef void tskTCB;
extern volatile tskTCB * volatile pxCurrentTCB;

/* Most ports implement critical sections by placing the interrupt flags on
the stack before disabling interrupts.  Exiting the critical section is then
simply a case of popping the flags from the stack.  As mspgcc does not use
a frame pointer this cannot be done as modifying the stack will clobber all
the stack variables.  Instead each task maintains a count of the critical
section nesting depth.  Each time a critical section is entered the count is
incremented.  Each time a critical section is left the count is decremented -
with interrupts only being re-enabled if the count is zero.

usCriticalNesting will get set to zero when the scheduler starts, but must
not be initialised to zero as this will cause problems during the startup
sequence. */
volatile unsigned short usCriticalNesting = portINITIAL_CRITICAL_NESTING;
/*-----------------------------------------------------------*/

#if __MSP430_CPU__ & MSP430_CPU_MSP430X

#define portASM_PUSH_GEN_REGS					\
	"pushm.w	#12, r15	\n\t"
#define portASM_POP_GEN_REGS					\
	"popm.w	#12, r15	\n\t"

#else

#define portASM_PUSH_GEN_REGS					\
	"push	r15			\n\t"					\
	"push	r14			\n\t"					\
	"push	r13			\n\t"					\
	"push	r12			\n\t"					\
	"push	r11			\n\t"					\
	"push	r10			\n\t"					\
	"push	r9			\n\t"					\
	"push	r8			\n\t"					\
	"push	r7			\n\t"					\
	"push	r6			\n\t"					\
	"push	r5			\n\t"					\
	"push	r4			\n\t"

#define portASM_POP_GEN_REGS					\
	"pop	r4			\n\t"					\
	"pop	r5			\n\t"					\
	"pop	r6			\n\t"					\
	"pop	r7			\n\t"					\
	"pop	r8			\n\t"					\
	"pop	r9			\n\t"					\
	"pop	r10			\n\t"					\
	"pop	r11			\n\t"					\
	"pop	r12			\n\t"					\
	"pop	r13			\n\t"					\
	"pop	r14			\n\t"					\
	"pop	r15			\n\t"					
						   
#endif

/* 
 * Macro to save a task context to the task stack.  This simply pushes all the 
 * general purpose msp430 registers onto the stack, followed by the 
 * usCriticalNesting value used by the task.  Finally the resultant stack 
 * pointer value is saved into the task control block so it can be retrieved 
 * the next time the task executes.
 */
#define portSAVE_CONTEXT()								\
	__asm__ __volatile__ ( portASM_PUSH_GEN_REGS		\
						   "mov.w	%0, r14		\n\t"	\
						   "push	r14			\n\t"	\
						   "mov.w	%1, r12		\n\t"	\
						   "mov.w	r1, @r12	\n\t"	\
						   :							\
						   : "m"( usCriticalNesting ),	\
							 "m"( pxCurrentTCB )		\
						   );

/* 
 * Macro to restore a task context from the task stack.  This is effectively
 * the reverse of portSAVE_CONTEXT().  First the stack pointer value is
 * loaded from the task control block.  Next the value for usCriticalNesting
 * used by the task is retrieved from the stack - followed by the value of all
 * the general purpose msp430 registers.
 *
 * The bic instruction ensures there are no low power bits set in the status
 * register that is about to be popped from the stack.
 */
#define portRESTORE_CONTEXT()							\
	__asm__ __volatile__ ( "mov.w	%1, r12		\n\t"	\
						   "mov.w	@r12, r1	\n\t"	\
						   "pop	r15				\n\t"	\
						   "mov.w	r15, %0		\n\t"	\
						   portASM_POP_GEN_REGS			\
						   "bic	%2, @r1			\n\t"	\
						   "reti				\n\t"	\
						   : "=m"( usCriticalNesting )	\
						   : "m"( pxCurrentTCB ),		\
							 "i"( portLPM_bits )		\
						   );

/*-----------------------------------------------------------*/

/* 
 * Initialise the stack of a task to look exactly as if a call to 
 * portSAVE_CONTEXT had been called.
 * 
 * See the header file portable.h.
 */
portSTACK_TYPE *pxPortInitialiseStack( portSTACK_TYPE *pxTopOfStack, pdTASK_CODE pxCode, void *pvParameters )
{
	/* The msp430 automatically pushes the PC then SR onto the stack before 
	executing an ISR.  We want the stack to look just as if this has happened
	so place a pointer to the start of the task on the stack first - followed
	by the flags we want the task to use when it starts up. */
	*pxTopOfStack = ( portSTACK_TYPE ) pxCode;
	pxTopOfStack--;
	*pxTopOfStack = portTASK_INITIAL_R2;
	pxTopOfStack--;

	/* When the task starts is will expect to find the function parameter in
	R15. */
	*pxTopOfStack = ( portSTACK_TYPE ) pvParameters;
	pxTopOfStack--;

	/* Next the remaining general purpose registers. */
	*pxTopOfStack = ( portSTACK_TYPE ) 0xeeee;
	pxTopOfStack--;
	*pxTopOfStack = ( portSTACK_TYPE ) 0xdddd;
	pxTopOfStack--;
	*pxTopOfStack = ( portSTACK_TYPE ) 0xcccc;
	pxTopOfStack--;
	*pxTopOfStack = ( portSTACK_TYPE ) 0xbbbb;
	pxTopOfStack--;
	*pxTopOfStack = ( portSTACK_TYPE ) 0xaaaa;
	pxTopOfStack--;
	*pxTopOfStack = ( portSTACK_TYPE ) 0x9999;
	pxTopOfStack--;
	*pxTopOfStack = ( portSTACK_TYPE ) 0x8888;
	pxTopOfStack--;
	*pxTopOfStack = ( portSTACK_TYPE ) 0x7777;
	pxTopOfStack--;
	*pxTopOfStack = ( portSTACK_TYPE ) 0x6666;
	pxTopOfStack--;
	*pxTopOfStack = ( portSTACK_TYPE ) 0x5555;
	pxTopOfStack--;
	*pxTopOfStack = ( portSTACK_TYPE ) 0x4444;
	pxTopOfStack--;

	/* The code generated by the mspgcc compiler does not maintain separate
	stack and frame pointers. The portENTER_CRITICAL macro cannot therefore
	use the stack as per other ports.  Instead a variable is used to keep
	track of the critical section nesting.  This variable has to be stored
	as part of the task context and is initially set to zero. */
	*pxTopOfStack = ( portSTACK_TYPE ) portNO_CRITICAL_SECTION_NESTING;	

	/* Return a pointer to the top of the stack we have generated so this can
	be stored in the task control block for the task. */
	return pxTopOfStack;
}
/*-----------------------------------------------------------*/

/* The TI headers are inconsistent as to whether the first Timer_A is
   controlled via TACTL or TA0CTL. */
#if ! defined(TACTL_) && ! defined(TACTL)
#define TACTL TA0CTL
#define TAR TA0R
#define TACCR0 TA0CCR0
#define TACCTL0 TA0CCTL0
#endif /* TACTL */
#ifndef TIMERA0_VECTOR
#define TIMERA0_VECTOR TIMER0_A0_VECTOR
#endif /* TIMERA0_VECTOR */

/*-----------------------------------------------------------*/

portBASE_TYPE xPortStartScheduler( void )
{
	/* Common practice for the MSPGCC port is for the application to
	   permanently configure the timers after the clocks are
	   configured.  TA0 is expected to be based on a 32 kHz ACLK
	   signal, but the timer itself runs regardless of whether the
	   scheduler is active, allowing time-since-boot to be maintained.

	   In support of legacy configurations, if
	   configMSP430_APP_CONTROLS_TA0 is not set the timer will be
	   configured at this point. */

#if ( configMSP430_APP_CONTROLS_TA0 != 1 )

	/* Ensure the timer is stopped before configuring. */
	TACTL = 0;
	__delay_cycles(50);

	/* Count continuously using ACLK clearing the initial counter. */
	TACTL = TASSEL_1 | MC_2 | TACLR;

#endif /* configMSP430_APP_CONTROLS_TA0 */

	/* Set the compare match value according to the tick rate we want. */
	TACCR0 = TAR + portACLK_FREQUENCY_HZ / configTICK_RATE_HZ;

	/* Enable the interrupts. */
	TACCTL0 = CCIE;

	/* Restore the context of the first task that is going to run. */
	portRESTORE_CONTEXT();

	/* Should not get here as the tasks are now running! */
	return pdTRUE;
}
/*-----------------------------------------------------------*/

void vPortEndScheduler( void )
{
	/* Disable the timeslice interrupt */
	TACCTL0 = 0;
}
/*-----------------------------------------------------------*/

/*
 * Manual context switch called by portYIELD or taskYIELD.  
 *
 * The first thing we do is save the registers so we can use a naked attribute.
 */
 __attribute__ ( ( __naked__ ) )
void vPortYield( void )
{
	/* We want the stack of the task being saved to look exactly as if the task
	was saved during a pre-emptive RTOS tick ISR.  Before calling an ISR the 
	msp430 places the status register onto the stack.  As this is a function 
	call and not an ISR we have to do this manually. */
	__asm__ __volatile__( "push\tr2" );
	__disable_interrupt();

	/* Save the context of the current task. */
	portSAVE_CONTEXT();

	/* Switch to the highest priority task that is ready to run. */
	vTaskSwitchContext();

	/* Restore the context of the new task. */
	portRESTORE_CONTEXT();
}
/*-----------------------------------------------------------*/

/* 
 * The interrupt service routine used depends on whether the pre-emptive
 * scheduler is being used or not.
 */

#if configUSE_PREEMPTION == 1

	/*
	 * Tick ISR for preemptive scheduler.  We can use a naked attribute as
	 * the context is saved at the start of vPortYieldFromTick().  The tick
	 * count is incremented after the context is saved.
	 */
	__attribute__( ( __interrupt__( TIMERA0_VECTOR ), __naked__ ) )
	static void prvTickISR( void )
	{
		/* Save the context of the interrupted task. */
		portSAVE_CONTEXT();

		/* Wake again for the next tick */
		TACCR0 += portACLK_FREQUENCY_HZ / configTICK_RATE_HZ;
		
		/* Increment the tick count then switch to the highest priority task
		that is ready to run. */
		vTaskIncrementTick();
		vTaskSwitchContext();

		/* Restore the context of the new task. */
		portRESTORE_CONTEXT();
	}

#else

	/*
	 * Tick ISR for the cooperative scheduler.  All this does is increment the
	 * tick count.  We don't need to switch context, this can only be done by
	 * manual calls to taskYIELD();
	 */
	__attribute__( ( __interrupt__( TIMERA0_VECTOR ) ) )
	static void prvTickISR( void )
	{
		/* Wake again for the next tick */
		TACCR0 += portACLK_FREQUENCY_HZ / configTICK_RATE_HZ;
		
		vTaskIncrementTick();
	}
#endif


	
