##############################################################################
BUILD = build
BIN = vcp

##############################################################################
.PHONY: all directory clean size

CC = arm-none-eabi-gcc
OBJCOPY = arm-none-eabi-objcopy
SIZE = arm-none-eabi-size
MKDIR = mkdir

CFLAGS += -W -Wall --std=gnu11 -Os
CFLAGS += -fno-diagnostics-show-caret
CFLAGS += -fdata-sections -ffunction-sections
CFLAGS += -funsigned-char -funsigned-bitfields
CFLAGS += -mcpu=cortex-m4 -mthumb
CFLAGS += -mfloat-abi=hard -mfpu=fpv4-sp-d16
CFLAGS += -MD -MP -MT $(BUILD)/$(*F).o -MF $(BUILD)/$(@F).d

LDFLAGS += -mcpu=cortex-m4 -mthumb
LDFLAGS += -mfloat-abi=hard -mfpu=fpv4-sp-d16
LDFLAGS += -Wl,--gc-sections
LDFLAGS += -Wl,--script=../linker/stm32g441.ld

INCLUDES += \
  -I../include \
  -I../usb \
  -I..

SRCS += \
  ../main.c \
  ../uart.c \
  ../usb/usb_stm32g4.c \
  ../usb/usb_std.c \
  ../usb/usb_cdc.c \
  ../usb_descriptors.c \
  ../startup_stm32g4.c \

DEFINES += \
  -DF_CPU=168000000 \

CFLAGS += $(INCLUDES) $(DEFINES)

OBJS = $(addprefix $(BUILD)/, $(notdir %/$(subst .c,.o, $(SRCS))))

all: directory $(BUILD)/$(BIN).elf $(BUILD)/$(BIN).hex $(BUILD)/$(BIN).bin size

$(BUILD)/$(BIN).elf: $(OBJS)
	@echo LD $@
	@$(CC) $(LDFLAGS) $(OBJS) $(LIBS) -o $@

$(BUILD)/$(BIN).hex: $(BUILD)/$(BIN).elf
	@echo OBJCOPY $@
	@$(OBJCOPY) -O ihex $^ $@

$(BUILD)/$(BIN).bin: $(BUILD)/$(BIN).elf
	@echo OBJCOPY $@
	@$(OBJCOPY) -O binary $^ $@

%.o:
	@echo CC $@
	@$(CC) $(CFLAGS) $(filter %/$(subst .o,.c,$(notdir $@)), $(SRCS)) -c -o $@

directory:
	@$(MKDIR) -p $(BUILD)

size: $(BUILD)/$(BIN).elf
	@echo size:
	@$(SIZE) -t $^

clean:
	@echo clean
	@-rm -rf $(BUILD)

-include $(wildcard $(BUILD)/*.d)

