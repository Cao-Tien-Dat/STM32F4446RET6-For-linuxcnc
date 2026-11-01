###############################################################################
# Remora Flexi-HAL — Bare-metal Makefile build (STM32F446RE)
# Replaces Mbed OS / Mbed Studio build system
#
# Usage:
#   make              Build firmware.elf / firmware.hex / firmware.bin
#   make clean        Remove build artifacts
#   make size         Show memory usage
#   make flash        Flash via st-flash (optional, requires st-link)
###############################################################################

# --- Toolchain ---------------------------------------------------------------
PREFIX  ?= arm-none-eabi-
CC      := $(PREFIX)gcc
AS      := $(PREFIX)gcc
OBJCOPY := $(PREFIX)objcopy
OBJDUMP := $(PREFIX)objdump
SIZE    := $(PREFIX)size
RM      := rm -rf
MKDIR   := mkdir -p

# --- Project ----------------------------------------------------------------
PROJECT := firmware
BUILD   := build
SRC     := src
INC     := inc

# --- Target device ----------------------------------------------------------
MCU     := cortex-m4
DEVICE  := STM32F446RE
FPU     := -mfpu=fpv4-sp-d16
FLOAT   := -mfloat-abi=hard

# --- Memory layout (matches Mbed mbed_app_target: app starts at 0x08010000) --
# Flash: 512 KB @ 0x08000000, app offset 0x10000 (bootloader)
# RAM:   128 KB @ 0x20000000

# --- Compiler flags ----------------------------------------------------------
CFLAGS  := -mcpu=$(MCU) $(FPU) $(FLOAT)
CFLAGS  += -mthumb -mthumb-interwork
CFLAGS  += -std=gnu11
CFLAGS  += -Wall -Wextra -Wdouble-promotion
CFLAGS  += -fdata-sections -ffunction-sections
CFLAGS  += -fno-builtin -fno-strict-aliasing
CFLAGS  += -D$(DEVICE) -DSTM32F446xx
CFLAGS  += -DARM_MATH_CM4
CFLAGS  += -O2 -g
CFLAGS  += -ffreestanding -nostdlib
CFLAGS  += -I$(INC)

ASFLAGS := -mcpu=$(MCU) -mthumb -g -x assembler-with-cpp
ASFLAGS += -D$(DEVICE) -DSTM32F446xx
ASFLAGS += -I$(INC)

LDFLAGS := -mcpu=$(MCU) $(FPU) $(FLOAT) -mthumb -mthumb-interwork
LDFLAGS += -specs=nano.specs -specs=nosys.specs
LDFLAGS += -Wl,-gc-sections -Wl,-Map=$(BUILD)/$(PROJECT).map
LDFLAGS += -T STM32F446RE_FLASH.ld
LDFLAGS += -Wl,--cref -Wl,--check-sections
LDFLAGS += -Wl,--gc-sections

# --- Source files ------------------------------------------------------------
# Module logic split into separate files (D01-D05).
APP_SRCS := $(SRC)/main.c \
	$(SRC)/gpio.c \
	$(SRC)/spi.c \
	$(SRC)/timer.c \
	$(SRC)/module.c \
	$(SRC)/system_stm32f4xx.c \
	$(SRC)/stepgen.c \
	$(SRC)/digitalpin.c \
	$(SRC)/pwm.c \
	$(SRC)/qei.c \
	$(SRC)/resetpin.c \
	$(SRC)/estop.c \
	$(SRC)/usart.c \
	$(SRC)/config.c \
	$(SRC)/miniprintf.c

# --- Derived ---------------------------------------------------------------
APP_OBJS := $(patsubst $(SRC)/%.c,$(BUILD)/%.o,$(APP_SRCS))
STARTUP_OBJ := $(BUILD)/startup.o

# --- Build rules -----------------------------------------------------------
.PHONY: all clean size flash

all: $(BUILD)/$(PROJECT).elf $(BUILD)/$(PROJECT).hex $(BUILD)/$(PROJECT).bin

$(BUILD):
	$(MKDIR) $(BUILD)

$(BUILD)/%.o: $(SRC)/%.c | $(BUILD)
	$(CC) $(CFLAGS) -c $< -o $@

$(STARTUP_OBJ): startup_stm32f446xx.s | $(BUILD)
	$(AS) $(ASFLAGS) -c $< -o $@

$(BUILD)/$(PROJECT).elf: $(APP_OBJS) $(STARTUP_OBJ) STM32F446RE_FLASH.ld
	$(CC) $(LDFLAGS) -Wl,--start-group $(APP_OBJS) $(STARTUP_OBJ) -Wl,--end-group -o $@

$(BUILD)/$(PROJECT).hex: $(BUILD)/$(PROJECT).elf
	$(OBJCOPY) -O ihex $< $@

$(BUILD)/$(PROJECT).bin: $(BUILD)/$(PROJECT).elf
	$(OBJCOPY) -O binary $< $@

size: $(BUILD)/$(PROJECT).elf
	$(SIZE) -A $<

clean:
	$(RM) $(BUILD)

flash: $(BUILD)/$(PROJECT).bin
	st-flash write $< 0x08010000
