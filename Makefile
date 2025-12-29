TARGET = sim868_stm32l151
MCU = cortex-m3

CC = arm-none-eabi-gcc
OBJCOPY = arm-none-eabi-objcopy
OBJDUMP = arm-none-eabi-objdump
SIZE = arm-none-eabi-size

CFLAGS = -mcpu=$(MCU) -mthumb -mfloat-abi=soft
CFLAGS += -Os -g
CFLAGS += -Wall -Wextra
CFLAGS += -DUSE_STDPERIPH_DRIVER
CFLAGS += -DSTM32L1XX_MD

LDFLAGS = -mcpu=$(MCU) -mthumb -mfloat-abi=soft
LDFLAGS += -T$(TARGET).ld
LDFLAGS += -Wl,--gc-sections

SRCS = main.c
OBJS = $(SRCS:.c=.o)

all: $(TARGET).elf $(TARGET).bin

$(TARGET).elf: $(OBJS)
	$(CC) $(LDFLAGS) -o $@ $^
	$(SIZE) $@

%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

$(TARGET).bin: $(TARGET).elf
	$(OBJCOPY) -O binary $< $@

clean:
	rm -f $(OBJS) $(TARGET).elf $(TARGET).bin

flash: $(TARGET).bin
	st-flash write $(TARGET).bin 0x08000000

.PHONY: all clean flash
