TARGET := Minesweeper_STM32F411
BUILD_DIR := Debug
TOOLCHAIN_PREFIX ?= arm-none-eabi-
CC := $(TOOLCHAIN_PREFIX)gcc
OBJCOPY := $(TOOLCHAIN_PREFIX)objcopy
SIZE := $(TOOLCHAIN_PREFIX)size
CPU := -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard
CUBE := vendor/cube
HAL := vendor/hal
DEVICE := vendor/device
FREERTOS := $(CUBE)/Middlewares/Third_Party/FreeRTOS/Source
USB := $(CUBE)/Middlewares/ST/STM32_USB_Device_Library
INCDIRS := Core/Inc Control/Inc Drivers/Board/Inc Serial/Inc USB_DEVICE/App USB_DEVICE/Target \
 $(HAL)/Inc $(HAL)/Inc/Legacy $(DEVICE)/Include $(CUBE)/Drivers/CMSIS/Include \
 $(FREERTOS)/include $(FREERTOS)/portable/GCC/ARM_CM4F $(FREERTOS)/CMSIS_RTOS \
 $(USB)/Core/Inc $(USB)/Class/CDC/Inc
DEFS := -DUSE_HAL_DRIVER -DSTM32F411xE \
 -D_POSIX_TIMERS=1 -D_DEFAULT_SOURCE=1
CFLAGS := $(CPU) $(DEFS) $(addprefix -I,$(INCDIRS)) -std=gnu11 -Os -g3 \
 -ffunction-sections -fdata-sections -fstack-usage -Wall -Wextra -Werror=implicit-function-declaration
HAL_MODULES := hal hal_cortex hal_rcc hal_rcc_ex hal_gpio hal_dma hal_dma_ex \
 hal_flash hal_flash_ex hal_flash_ramfunc hal_pwr hal_pwr_ex hal_tim hal_tim_ex \
 hal_i2c hal_spi hal_adc hal_adc_ex hal_iwdg hal_pcd hal_pcd_ex ll_usb
C_SOURCES := $(wildcard Core/Src/*.c Control/Src/*.c Drivers/Board/Src/*.c USB_DEVICE/App/*.c USB_DEVICE/Target/*.c) \
 $(addprefix $(HAL)/Src/stm32f4xx_,$(addsuffix .c,$(HAL_MODULES))) \
 $(DEVICE)/Source/Templates/system_stm32f4xx.c \
 $(FREERTOS)/tasks.c $(FREERTOS)/queue.c $(FREERTOS)/list.c \
 $(FREERTOS)/portable/GCC/ARM_CM4F/port.c $(FREERTOS)/portable/MemMang/heap_4.c \
 $(USB)/Core/Src/usbd_core.c $(USB)/Core/Src/usbd_ctlreq.c $(USB)/Core/Src/usbd_ioreq.c \
 $(USB)/Class/CDC/Src/usbd_cdc.c $(wildcard Serial/Src/*.c)
ASM_SOURCE := $(DEVICE)/Source/Templates/gcc/startup_stm32f411xe.s
OBJECTS := $(addprefix $(BUILD_DIR)/,$(C_SOURCES:.c=.o)) $(BUILD_DIR)/$(ASM_SOURCE:.s=.o)
LDFLAGS := $(CPU) -TSTM32F411CEUX_FLASH.ld --specs=nano.specs --specs=nosys.specs \
 -Wl,-Map=$(BUILD_DIR)/$(TARGET).map,--cref,--gc-sections,--print-memory-usage
.PHONY: all clean dependencies test print_cflags check-dependencies
all: check-dependencies $(BUILD_DIR)/$(TARGET).elf $(BUILD_DIR)/$(TARGET).hex $(BUILD_DIR)/$(TARGET).bin
check-dependencies:
	@test -f $(CUBE)/Drivers/CMSIS/Include/core_cm4.h || { echo 'Run python3 scripts/fetch_dependencies.py'; exit 1; }
$(BUILD_DIR)/%.o: %.c
	@mkdir -p $(dir $@)
	$(CC) $(CFLAGS) -MMD -MP -c $< -o $@
$(BUILD_DIR)/$(DEVICE)/Source/Templates/system_stm32f4xx.o: CFLAGS += -DSystemInit=Vendor_SystemInit
$(BUILD_DIR)/%.o: %.s
	@mkdir -p $(dir $@)
	$(CC) $(CPU) -x assembler-with-cpp -g3 -c $< -o $@
$(BUILD_DIR)/$(TARGET).elf: $(OBJECTS) STM32F411CEUX_FLASH.ld
	$(CC) $(OBJECTS) $(LDFLAGS) -Wl,--start-group -lc -lm -Wl,--end-group -o $@
	$(SIZE) $@
$(BUILD_DIR)/%.hex: $(BUILD_DIR)/%.elf
	$(OBJCOPY) -O ihex $< $@
$(BUILD_DIR)/%.bin: $(BUILD_DIR)/%.elf
	$(OBJCOPY) -O binary $< $@
dependencies:
	python3 scripts/fetch_dependencies.py
test:
	bash scripts/test.sh
print_cflags:
	@echo $(CPU) -Os -ffunction-sections -fdata-sections -D_POSIX_TIMERS=1 -D_DEFAULT_SOURCE=1
clean:
	rm -rf $(BUILD_DIR)
-include $(OBJECTS:.o=.d)
