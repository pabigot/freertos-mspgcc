This project provides alternative support for the Texas Instruments MSP430
using GCC (mspgcc) with FreeRTOS.

Because the MSP430 has over 300 variant MCUs and a host of available
development boards, the FreeRTOS practice of separate directories for each
port result in a difficult to maintain environment where core support for
capabilities and peripherals ends up replicated in each port.

In this structure, there is one port for GCC/MSP430.  Each board port uses
this port, adding:

* A platform include file that defines appropriate constants;
* A platform configuration file that defines structures used to initialize
  the MCU peripherals available on the platform;
* A platform Makefile.inc that names the peripheral implementations for
  basic functionality (e.g., whether serial communications uses the USART or
  one of the several USCI implementations).

The layout of the project mirrors that of FreeRTOS itself.  An unmodified
FreeRTOS distribution is kept in the top-level freertos subdirectory, and is
referenced by the demonstration and library routines.  Include and library
paths into the unmodified distribution should be subordinate to the
corresponding paths in this project, to ensure overrides occur where
necessary.

