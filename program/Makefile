#======================================================================#

#Output files
TARGET=firmware
EXECUTABLE=$(TARGET).elf
BIN_IMAGE=$(TARGET).bin
DUMP_LOG=$(TARGET).dump

#======================================================================#

#Cross Compiler
CC=arm-none-eabi-gcc
LD=arm-none-eabi-gcc
OBJCOPY=arm-none-eabi-objcopy
OBJDUMP=arm-none-eabi-objdump
ELF_SIZE=arm-none-eabi-size
GDB=arm-none-eabi-gdb

#======================================================================#

#Flags
CFLAGS=-g -Ofast -mlittle-endian -mthumb -std=gnu99
CFLAGS+=-mcpu=cortex-m7
CFLAGS+=-mfpu=fpv5-d16 -mfloat-abi=hard
CFLAGS+=-ffunction-sections -fdata-sections -ffast-math
CFLAGS+=--specs=nano.specs --specs=nosys.specs -fno-builtin-printf
CFLAGS+=-Wall -Wno-strict-aliasing
CFLAGS+=-D STM32F767xx \
	-D USE_HAL_DRIVER \
	-D ARM_MATH_CM7 \
	-D __FPU_PRESENT

CFLAGS+=-Wl,-T,stm32f767vi_flash.ld

LDFLAGS=-Wl,--gc-sections -lm

ARCH=ARM_CM7

#======================================================================#

ST_LIB=lib/STM32Cube_FW_F7_V1.4.0
ST_PERIPHERIAL=$(ST_LIB)/Drivers/STM32F7xx_HAL_Driver
ST_USB_LIB=$(ST_LIB)/Middlewares/ST/STM32_USB_Device_Library

FREERTOS=./lib/FreeRTOSv9.0.0/FreeRTOS

DRIVER=./driver
PERIPH=$(DRIVER)/peripherial
DEVICE=$(DRIVER)/device
COMMON=./common
SYSTEM=./system
DSP=$(ST_LIB)/Drivers/CMSIS/DSP_Lib/Source/

#======================================================================#

SRC=./system_stm32f7xx.c

#ST peripherial
CFLAGS+=-I$(ST_LIB)/Drivers/CMSIS/Include
CFLAGS+=-I$(ST_LIB)/Drivers/CMSIS/Device/ST/STM32F7xx/Include
CFLAGS+=-I$(ST_PERIPHERIAL)/Inc
SRC+=$(ST_PERIPHERIAL)/Src/stm32f7xx_hal_cortex.c \
	$(ST_PERIPHERIAL)/Src/stm32f7xx_hal_pwr_ex.c \
	$(ST_PERIPHERIAL)/Src/stm32f7xx_hal.c \
	$(ST_PERIPHERIAL)/Src/stm32f7xx_hal_rcc.c \
	$(ST_PERIPHERIAL)/Src/stm32f7xx_hal_dma.c \
	$(ST_PERIPHERIAL)/Src/stm32f7xx_hal_gpio.c \
	$(ST_PERIPHERIAL)/Src/stm32f7xx_hal_uart.c \
	$(ST_PERIPHERIAL)/Src/stm32f7xx_hal_spi.c \
	$(ST_PERIPHERIAL)/Src/stm32f7xx_hal_pcd.c \
	$(ST_PERIPHERIAL)/Src/stm32f7xx_ll_usb.c \
	$(ST_PERIPHERIAL)/Src/stm32f7xx_hal_pcd_ex.c \
	$(ST_PERIPHERIAL)/Src/stm32f7xx_hal_tim_ex.c \
	$(ST_PERIPHERIAL)/Src/stm32f7xx_hal_tim.c \
	$(ST_PERIPHERIAL)/Src/stm32f7xx_hal_i2c.c \
	$(ST_PERIPHERIAL)/Src/stm32f7xx_hal_i2c_ex.c \
	$(ST_PERIPHERIAL)/Src/stm32f7xx_hal_dcmi.c \
	$(ST_PERIPHERIAL)/Src/stm32f7xx_hal_dma_ex.c

#ST usb library
CFLAGS+=-I$(ST_USB_LIB)/Class/CDC/Inc
SRC+=$(ST_USB_LIB)/Class/CDC/Src/usbd_cdc.c

CFLAGS+=-I$(ST_USB_LIB)/Core/Inc
SRC+=$(ST_USB_LIB)/Core/Src/usbd_core.c \
	$(ST_USB_LIB)/Core/Src/usbd_ctlreq.c \
	$(ST_USB_LIB)/Core/Src/usbd_ioreq.c

#Freertos
#CFLAGS+=-I$(FREERTOS)/Source/include \
#	-I$(FREERTOS)/Source/portable/GCC/$(ARCH)/r0p1/
#SRC+=$(FREERTOS)/Source/croutine.c \
#	$(FREERTOS)/Source/list.c \
#	$(FREERTOS)/Source/queue.c \
#	$(FREERTOS)/Source/tasks.c \
#	$(FREERTOS)/Source/timers.c \
#	$(FREERTOS)/Source/portable/MemMang/heap_4.c \
#	$(FREERTOS)/Source/portable/GCC/$(ARCH)/r0p1/port.c

SRC+=$(DSP)/BasicMathFunctions/arm_abs_f32.c

CFLAGS+=-I$(DRIVER)/peripherial
CFLAGS+=-I$(DRIVER)/peripherial/usb
SRC+=$(DRIVER)/peripherial/core.c \
	$(DRIVER)/peripherial/uart.c \
	$(DRIVER)/peripherial/gpio.c \
	$(DRIVER)/peripherial/spi.c \
	$(DRIVER)/peripherial/interrupt.c \
	$(DRIVER)/peripherial/pwm.c \
	$(DRIVER)/peripherial/timer.c \
	$(DRIVER)/peripherial/i2c.c \
	$(DRIVER)/peripherial/dcmi.c \
	$(DRIVER)/peripherial/usb/usbd_cdc_if.c \
	$(DRIVER)/peripherial/usb/usbd_conf.c \
	$(DRIVER)/peripherial/usb/usbd_desc.c \
	$(DRIVER)/peripherial/usb/usb_device.c

CFLAGS+=-I$(DRIVER)/device
SRC+=$(DRIVER)/device/mpu9250.c \
	$(DRIVER)/device/mt9v034.c \
	$(DRIVER)/device/lidar.c

CFLAGS+=-I$(COMMON)

CFLAGS+=-I$(SYSTEM)
SRC+=$(SYSTEM)/main.c \
	$(SYSTEM)/flow.c \
	$(SYSTEM)/usb_link.c \
	$(SYSTEM)/fcb_link.c \
	$(SYSTEM)/system_time.c

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

objdump:
	make all
	rm -rf $(DUMP_LOG)
	$(OBJDUMP) -d $(EXECUTABLE) >> $(DUMP_LOG)

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
	openocd -f interface/stlink-v2.cfg \
	-c "transport select hla_swd" \
	-f target/stm32f7x.cfg \
	-c "reset_config none separate" \
	-c "init" \
	-c "reset init" \
	-c "halt" \
	-c "flash write_image erase $(TARGET).bin 0x8000000" \
	-c "verify_image $(TARGET).elf" \
	-c "reset run" -c shutdown

openocd:
	openocd -f ./debug/openocd.cfg

gdbauto:
	cgdb -d $(GDB) -x ./debug/openocd_gdb.gdb

gdbtui:
	$(GDB) -tui -x ./debug/openocd_gdb.gdb

size:
	$(ELF_SIZE) $(EXECUTABLE) 

.ONESHELL:
astyle:
	cd ..
	astyle -r --exclude=lib --style=linux --indent=tab=8  *.c *.h
	cd program/

#=====================================================================#

.PHONY:all clean flash astyle tags
