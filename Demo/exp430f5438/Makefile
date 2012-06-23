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

PLATFORM ?= exp430f5438
MCU ?= msp430f5438a
CROSS_COMPILE ?= msp430-

FREERTOS_ROOT=../../freertos
SOURCE_PATH=$(FREERTOS_ROOT)/Source
PORT_PATH=../../Source/portable/GCC/MSP430
DEMO_PATH=$(FREERTOS_ROOT)/Demo/Common/Minimal
BSP430_PATH=../Common/drivers/MSP430/bsp430

CC=$(CROSS_COMPILE)gcc
LD=$(CROSS_COMPILE)ld
AR=$(CROSS_COMPILE)ar

# Can't use -f*-sections until SF 3534425 is fixed
OPT_CFLAGS ?= -Os -ffunction-sections -fdata-sections
WARN_CFLAGS = -Wall -Wno-main
OPT_LDFLAGS ?= -Wl,-gc-sections

CPPFLAGS = -I. -I$(BSP430_PATH) -I$(PORT_PATH)
CPPFLAGS += -I$(BSP430_PATH)/$(PLATFORM) -I$(BSP430_PATH)/include
CPPFLAGS += -I$(SOURCE_PATH)/include
CPPFLAGS += -I$(FREERTOS_ROOT)/Demo/Common/include

# TARGET_FLAGS holds things like -msr20, -md20, -mc20, or -mmemory-model=.
# Like -mmcu, it gets passed in both CFLAGS and LDFLAGS.
ifndef TARGET_FLAGS
ifdef MEMORY_MODEL
TARGET_FLAGS=-mmemory-model=$(MEMORY_MODEL)
endif # MEMORY_MODEL
endif # TARGET_FLAGS

CFLAGS = -mmcu=$(MCU) $(TARGET_FLAGS) $(OPT_CFLAGS) -ggdb $(WARN_CFLAGS) $(XCFLAGS)
LDFLAGS=-mmcu=$(MCU) $(TARGET_FLAGS) $(OPT_LDFLAGS)

SRC = \
$(SOURCE_PATH)/tasks.c \
$(SOURCE_PATH)/list.c \
$(SOURCE_PATH)/queue.c \
$(SOURCE_PATH)/portable/MemMang/heap_1.c \
$(PORT_PATH)/port.c \
$(DEMO_PATH)/flash.c \
$(BSP430_PATH)/utility/led.c \
$(BSP430_PATH)/utility/console.c \
$(BSP430_PATH)/clocks/ucs.c \
$(BSP430_PATH)/5xx/usci.c \
$(BSP430_PATH)/timers/timerA0.c \
$(BSP430_PATH)/$(PLATFORM)/platform.c \
$(BSP430_PATH)/$(PLATFORM)/serial.c \
$(BSP430_PATH)/ParTest.c \
main.c

%.s: %.c
	$(CC) $(CPPFLAGS) $(CFLAGS) -S -o $@ $<

OBJ = $(SRC:.c=.o)
DEP = $(SRC:.c=.d)

demo.elf: $(OBJ)
	$(CC) -o $@ $(LDFLAGS) $(OBJ)
	$(CROSS_COMPILE)size $@

clean:
	-rm -f $(OBJ)

realclean: clean
	-rm -f demo.elf

install: demo.elf
	mspdebug -j uif 'prog demo.elf'

%.d: %.c
	@set -e; rm -f $@; \
	 $(CC) -MM -MT$*.o $(CPPFLAGS) $(CFLAGS) $< > $@.$$$$; \
	 sed 's,\($*\)\.o[ :]*,\1.o $@ : ,g' < $@.$$$$ > $@; \
	 rm -f $@.$$$$

include $(DEP)
