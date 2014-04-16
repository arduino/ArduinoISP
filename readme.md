ArduinoISP firwmare sources:
 
To compile sources you need avr-gcc.
You also need an AVRISP mkII programmer to write the firwmare into the device. If you want to use another programmer you have to modify the makefile and change the command line for avrdude in order to match your programmer.
Just move into the code folder with the terminal and use:

*make hex*
 
Then write the binary file into the microcontroller with

*sudo make program*
