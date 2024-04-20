# GPS-Based-Date-Time-Clock
Arduino and Adafruit Ultimate GPS Clock using MAX7219 (8 Digit, 7-Segment Display Drivers)

This Arduino sketch began with the max7219.h/.cpp library by Johnathan Evans.  Since this project has two displays (at this time), Mr. Evan's library would not work, but it was a great starting point. I attempted to use other libraries that support one or several MAX7219 drivers, but I could not get them to work at all. I used the concepts from Mr. Evan's work and began writing the Arduino sketch to not use any library functions.  This was partially successful so far.  At this point, the clock project is functioning and work on the circuit boards has begun.

To date, MOST of Mr. Evan's max7219 library has been delinked from this project.  They only part not delinked is the PIN usage.  Although the sketch has this code natively, when I attempt to remove the #include <max7219.h> the sketch fails to update the clock.  I am working to figure this out.  Nevertheless, it works as is.

This project uses an Arduino Uno, an Adafruit Ultimate GPS v3, two MAX7219 display drivers, and sixteen 7-segment 1.3" tall display modules. Basically, the code receives GPS output via a 9600 baud serial connection.  It waits until a valid $GPRMC sentence is received, then parses the DATE and TIME fields to two strings.  Each character of each digit (or character) (DATE and TIME) is evaluated, converted to the necessary binary 8-bit sequence to represent which LEDs to turn on in each display.

Since the MAX7219 cannot be addressed individually, the code must send 16 bits for each 7-segment display as serial output.  Since two MAX7219s are daisy-chained together, 32-bits need to be sent for each set of digits; i.e., one digit on the TIME display and one digit on the DATE display.  This is repeated for all eight digits in each display.  Therefore 256 bits are sent to complete one instance of DATE; i.e., 20240420 (20 Apr 2024) and TIME; i.e, 00153020 (3:15 PM and 20 seconds).  The first two digits of the TIME (00) are not displayed because the MAX7219 is configured (via the code) to only display the right 6 digits; i.e., 153020, of the time display.

The last part to explain (at this point) is that the MAX7219 displays each digit by determining its position address (1 to 8) and then send the 7-segment binary pattern to display.  The position address is 8 bits and the binary address is 8 bits (16 bits total).  Additionally, special functions in the MAX7219 are configured using the same pattern; i.e., address then data.


