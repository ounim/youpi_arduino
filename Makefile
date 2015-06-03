# Arduino Make file. Refer to https://github.com/sudar/Arduino-Makefile

# If not, you might have to include the full path.
#ALTERNATE_CORE_PATH = /home/sudar/Dropbox/code/arduino-sketches/hardware/attiny/

BOARD_TAG    = uno

ARDUINO_LIBS =

BOARDS_TXT = ${ARDUINO_DIR}/hardware/arduino/avr/boards.txt

ARDUINO_CORE_PATH = $(ARDUINO_DIR)/hardware/arduino/avr/cores/arduino

ARDUINO_VAR_PATH = $(ARDUINO_DIR)/hardware/arduino/avr/variants

include /usr/share/arduino/Arduino.mk

# !!! Important. You have to use make ispload to upload when using ISP programmer
