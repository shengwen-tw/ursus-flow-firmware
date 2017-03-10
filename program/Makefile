#======================================================================#

#Output files
TARGET=firmware
EXECUTABLE=$(TARGET).elf
BIN_IMAGE=$(TARGET).bin

#======================================================================#

#Cross Compiler
CC=arm-none-eabi-gcc
LD=arm-none-eabi-gcc
OBJCOPY=arm-none-eabi-objcopy
GDB=arm-none-eabi-gdb

#======================================================================#

#Flags
CFLAGS=-g -O2 -mlittle-endian -mthumb -std=c99
CFLAGS+=-mcpu=cortex-m7
CFLAGS+=-mfpu=fpv5-sp-d16 -mfloat-abi=hard
CFLAGS+=-ffunction-sections -fdata-sections -ffast-math
CFLAGS+=--specs=nano.specs --specs=nosys.specs -fno-builtin-printf
CFLAGS+=-Wall -Wno-strict-aliasing
CFLAGS+=-D STM32F746xx \
	-D USE_HAL_DRIVER

CFLAGS+=-Wl,-T,STM32F767VI_FLASH.ld

LDFLAGS=-Wl,--gc-sections -lm

ARCH=ARM_CM7

#======================================================================#

ST_LIB=lib/STM32Cube_FW_F7_V1.4.0
ST_PERIPHERIAL=$(ST_LIB)/Drivers/STM32F7xx_HAL_Driver
CFLAGS+=-I$(ST_LIB)/Drivers/CMSIS/Include
CFLAGS+=-I$(ST_LIB)/Drivers/CMSIS/Device/ST/STM32F7xx/Include

FREERTOS=./lib/FreeRTOSv9.0.0/FreeRTOS

DRIVER=./driver
SYSTEM=./system

#======================================================================#

SRC=./system_stm32f7xx.c

CFLAGS+=-I$(ST_PERIPHERIAL)/Inc
SRC+=$(ST_PERIPHERIAL)/Src/stm32f7xx_hal_cortex.c \
	$(ST_PERIPHERIAL)/Src/stm32f7xx_hal.c \
	$(ST_PERIPHERIAL)/Src/stm32f7xx_hal_rcc.c \
	$(ST_PERIPHERIAL)/Src/stm32f7xx_hal_dma.c \
	$(ST_PERIPHERIAL)/Src/stm32f7xx_hal_gpio.c \
	$(ST_PERIPHERIAL)/Src/stm32f7xx_hal_uart.c

#Freertos
CFLAGS+=-I$(FREERTOS)/Source/include \
	-I$(FREERTOS)/Source/portable/GCC/$(ARCH)/r0p1/
SRC+=$(FREERTOS)/Source/croutine.c \
	$(FREERTOS)/Source/list.c \
	$(FREERTOS)/Source/queue.c \
	$(FREERTOS)/Source/tasks.c \
	$(FREERTOS)/Source/timers.c \
	$(FREERTOS)/Source/portable/MemMang/heap_4.c \
	$(FREERTOS)/Source/portable/GCC/$(ARCH)/r0p1/port.c

CFLAGS+=-I$(DRIVER)/peripherial
SRC+=$(DRIVER)/peripherial/uart.c

CFLAGS+=-I./$(SYSTEM)
SRC+=./$(SYSTEM)/main.c

CFLAGS+=-I./

#======================================================================#

#STM32 startup file
STARTUP=./startup_stm32f767xx.s

#======================================================================#

#Compiling rules

OBJECTS=$(SRC:.c=.o)
OBJECTS+=$(STARTUP:.s=.o)

all:$(BIN_IMAGE) tags

#Convert elf file to binary file
$(BIN_IMAGE):$(EXECUTABLE)
	@$(OBJCOPY) -O binary $^ $@
	@echo 'OBJCOPY $^'
	@echo 'Built $@'

#Link object files and generate elf file
$(EXECUTABLE): $(OBJECTS)
	@$(LD) $^ $(LDLIBS) $(CFLAGS) -o $@
	@echo '[Linking ELF file]'

#Generate object file from *.c
%.o: %.c
	@$(CC) $(CFLAGS) -c $< -o $@
	@echo 'Building object $@'

#Generate object file from *.s
%.o: %.s
	@$(CC) $(CFLAGS) -c $< -o $@
	@echo 'Building object $@'

#Generate tags file
tags:
	@echo 'Generating ctags'
	@ctags -R

#=====================================================================#

#Make clean
clean:
	rm -rf $(EXECUTABLE)
	rm -rf $(BIN_IMAGE)
	rm -rf $(OBJECTS)

flash: $(TARGET).bin
	st-flash write $(TARGET).bin 0x8000000

openocd:
	openocd -f ./debug/openocd.cfg

gdbauto:
	cgdb -d $(GDB) -x ./debug/openocd_gdb.gdb

#=====================================================================#

.PHONY:all clean flash tags
