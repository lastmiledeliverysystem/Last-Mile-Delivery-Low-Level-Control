avr-gcc -Os -DF_CPU=16000000UL -mmcu=atmega128 -c main.c
avr-gcc -mmcu=atmega128 *.o -o run
avr-objcopy -O ihex -R .eeprom run run.hex
avrdude -F -V -c usbasp -p ATMEGA128 -P /dev/ttyACM0 -b 115200 -U flash:w:run.hex
 