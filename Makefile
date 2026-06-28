MCU     = atmega328p
F_CPU   = 20000000UL
TARGET  = LawnMower_MotherBoard

CC      = avr-gcc
OBJCOPY = avr-objcopy

# Collecte récursive de tous les .c et tous les dossiers contenant des .h
SRC     := $(shell find LawnMower_MotherBoard -name "*.c")
OBJ     := $(SRC:.c=.o)
INC     := $(shell find LawnMower_MotherBoard -name "*.h" -exec dirname {} \; | sort -u | sed 's/^/-I/')

CFLAGS  = -mmcu=$(MCU) -DF_CPU=$(F_CPU) -Os -Wall $(INC)

all: $(TARGET).hex

$(TARGET).elf: $(OBJ)
	$(CC) $(CFLAGS) -o $@ $^

%.o: %.c
	$(CC) $(CFLAGS) -c -o $@ $<

$(TARGET).hex: $(TARGET).elf
	$(OBJCOPY) -O ihex $< $@

flash:
	avrdude -c pickit4_isp -p m328p -U flash:w:$(TARGET).hex

clean:
	find LawnMower_MotherBoard -name "*.o" -delete
	rm -f $(TARGET).elf $(TARGET).hex