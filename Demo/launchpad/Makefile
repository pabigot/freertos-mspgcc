# Copyright (c) 2012, Peter A. Bigot <bigotp@acm.org>
# 
# All rights reserved.
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# 
# * Redistributions of source code must retain the above copyright notice,
#   this list of conditions and the following disclaimer.
# 
# * Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.
# 
# * Neither the name of the software nor the names of its contributors may be
#   used to endorse or promote products derived from this software without
#   specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

PLATFORM ?= launchpad
MCU ?= msp430g2553
CROSS_COMPILE ?= msp430-

FREERTOS_ROOT=../../freertos
SOURCE_PATH=$(FREERTOS_ROOT)/Source
PORT_PATH=../../Source/portable/GCC/MSP430
DEMO_PATH=$(FREERTOS_ROOT)/Demo/Common/Minimal
BSP430_PATH=../Common/drivers/MSP430/bsp430

CC=$(CROSS_COMPILE)gcc
LD=$(CROSS_COMPILE)ld
AR=$(CROSS_COMPILE)ar

OPT_CFLAGS ?= -Os -ffunction-sections -fdata-sections
LDFLAGS ?= -Wl,-gc-sections

CPPFLAGS = -I. -I$(BSP430_PATH) -I$(PORT_PATH)
CPPFLAGS += -I$(SOURCE_PATH)/include
CPPFLAGS += -I$(FREERTOS_ROOT)/Demo/Common/include
CFLAGS = -mmcu=$(MCU) $(OPT_CFLAGS)

SRC = \
$(SOURCE_PATH)/tasks.c \
$(SOURCE_PATH)/list.c \
$(SOURCE_PATH)/queue.c \
$(SOURCE_PATH)/croutine.c \
$(SOURCE_PATH)/portable/MemMang/heap_1.c \
$(PORT_PATH)/port.c \
$(DEMO_PATH)/crflash.c \
$(BSP430_PATH)/utility/led.c \
$(BSP430_PATH)/ParTest.c \
main.c

# $(BSP430_PATH)/clocks/fllplus.c \

OBJ = $(SRC:.c=.o)

demo.elf: $(OBJ)
	$(CC) -o $@ $(CFLAGS) $(LDFLAGS) $(OBJ)

clean:
	-rm -f $(OBJ)

realclean: clean
	-rm -f demo.elf

install: demo.elf
	mspdebug rf2500 'prog demo.elf'
