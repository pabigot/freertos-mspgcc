MCU ?= msp430f5438a
CROSS_COMPILE ?= msp430-

FREERTOS_ROOT=../../freertos
SOURCE_PATH=$(FREERTOS_ROOT)/Source
PORT_PATH=../../Source/portable/GCC/MSP430
DEMO_PATH=$(FREERTOS_ROOT)/Demo/Common/Minimal
BSP430_PATH=../Common/drivers/MSP430/bsp430
HAL_PATH=MSP-EXP430F5438_HAL
CORE_PATH=F5XX_6XX_Core_Lib

CC=$(CROSS_COMPILE)gcc
LD=$(CROSS_COMPILE)ld
AR=$(CROSS_COMPILE)ar

# Can't use -f*-sections until SF 3534425 is fixed
OPT_CFLAGS ?= -Os # -ffunction-sections -fdata-sections
WARN_CFLAGS = -Wall -Wno-main
OPT_LDFLAGS ?= # -Wl,-gc-sections

CPPFLAGS = -I. -I$(PORT_PATH)
CPPFLAGS += -I$(HAL_PATH) -I$(CORE_PATH)
CPPFLAGS += -I$(SOURCE_PATH)/include
CPPFLAGS += -I$(FREERTOS_ROOT)/Demo/Common/include

# TARGET_FLAGS holds things like -msr20, -md20, -mc20, or -mmemory-model=.
# Like -mmcu, it gets passed in both CFLAGS and LDFLAGS.
ifndef TARGET_FLAGS
ifdef MEMORY_MODEL
TARGET_FLAGS=-mmemory-model=$(MEMORY_MODEL)
endif # MEMORY_MODEL
endif # TARGET_FLAGS

CFLAGS=-mmcu=$(MCU) $(TARGET_FLAGS) $(OPT_CFLAGS) -ggdb $(WARN_CFLAGS) $(XCFLAGS)
LDFLAGS=-mmcu=$(MCU) $(TARGET_FLAGS) $(OPT_LDFLAGS) -ggdb

# Object file derived from assembler source including preprocessor
%.o: %.S
	$(CC) $(CPPFLAGS) $(CFLAGS) -c -o $@ $<

SRC = \
$(SOURCE_PATH)/tasks.c \
$(SOURCE_PATH)/queue.c \
$(SOURCE_PATH)/list.c \
$(SOURCE_PATH)/timers.c \
$(SOURCE_PATH)/portable/MemMang/heap_1.c \
$(PORT_PATH)/port.c \
$(DEMO_PATH)/dynamic.c \
$(DEMO_PATH)/comtest.c \
$(DEMO_PATH)/GenQTest.c \
$(DEMO_PATH)/TimerDemo.c \
$(DEMO_PATH)/countsem.c \
$(HAL_PATH)/hal_board.c \
$(HAL_PATH)/hal_buttons.c \
$(HAL_PATH)/hal_lcd.c \
$(HAL_PATH)/hal_lcd_fonts.c \
$(CORE_PATH)/HAL_PMM.c \
$(CORE_PATH)/hal_UCS.c \
ParTest.c \
serial.c \
main.c

SRC_S = \
RegTest.S \

OBJ = $(SRC:.c=.o) $(SRC_S:.S=.o)
DEP = $(SRC:.c=.d)

demo.elf: $(OBJ)
	$(CC) -o $@ $(CFLAGS) $(LDFLAGS) $(OBJ)

clean:
	-rm -f $(OBJ)

realclean: clean
	-rm -f demo.elf $(DEP)

install: demo.elf
	mspdebug -j uif 'prog demo.elf'

%.d: %.c
	@set -e; rm -f $@; \
	$(CC) -MM $(CPPFLAGS) $< > $@.$$$$; \
	sed 's,\($*\)\.o[ :]*,\1.o $@ : ,g' < $@.$$$$ > $@; \
	rm -f $@.$$$$

include $(DEP)
