# modified for use with avra assembler - lynf 2020/03/05

# simple AVR Makefile
#
# written by michael cousins (http://github.com/mcous)
# released to the public domain

# Makefile
#
# targets:
#   all:    assemble the source code
#   test:   tests the isp connection to the mcu
#   flash:  writes assembled hex file to the mcu's flash memory
#   clean:  removes all .hex, .eep and .obj files

# parameters (change this stuff accordingly)
# project name
PRJ = main
# avr mcu
MCU = atmega328p
# mcu clock frequency
CLK = 16000000
# avr programmer
PRG = usbasp

# fuse values for avr: low, high, and extended
# these values are from an Arduino Uno (ATMega328P)
# see http://www.engbedded.com/fusecalc/ for other MCUs and options
LFU= 0xf7
HFU = 0xdf
EFU = 0xfd

# fuse values for low voltage 3V3 operation and 8.00 MHz clock
# change as required for particular board application
#LFU = 0xf2
#HFU = 0xdf
#EFU = 0xfd


# program source files (not including external libraries)
SRC = $(PRJ).asm


#################################################################################################
# \/ stuff nobody needs to worry about until such time that worrying about it is appropriate \/ #
#################################################################################################


# executables
AVRDUDE = avrdude -c $(PRG) -p $(MCU) -P usb
OBJCOPY = avr-objcopy
OBJDUMP = avr-objdump
SIZE    = avr-size --format=avr --mcu=$(MCU)
AS      = avra


# user target
# assemble main.asm
all: 
	$(AS) $(SRC) 

# test programmer connectivity
test:
	$(AVRDUDE) -v

# flash program to mcu
flash: all
	$(AVRDUDE) -U flash:w:$(PRJ).hex:i

# write fuses to mcu
fuse:
	$(AVRDUDE) -U lfuse:w:$(LFU):m -U hfuse:w:$(HFU):m -U efuse:w:$(EFU):m

# read fuses to file fuses.hex
readfuse:
	$(AVRDUDE) -U lfuse:r:LFU:h -U hfuse:r:HFU:h -U efuse:r:EFU:h

# remove compiled files
clean:
	rm -f *.hex *.eep.hex *.obj

