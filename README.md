# RS485PiAVR
Raspberry Pi Master and AVR slave. Pi master written in Python, AVR slave written in C

Custom shields for Raspberry Pi and AVR.

Self determines attached DS18x20 temperature sensors as well as attached 
current transformers for energy monitoring.

Uses a Microduino Core+ with an ATmega644PA running at 5v with 16Mhz clock

AVRDude command to upload code.
sudo avrdude -P /dev/ttyUSB0 -p m644p -c arduino -U flash:w:slave.hex

