/*
* GPS-based Date and Time Clock.  Extracts NEMA 0183 $GPRMC sentence (Date
*   and Time) information from a GPS module's output data stream and sends 
*   this data to two MAX7219 7-Segment 8-digit common cathode display drivers.
*   This is sort of like a STRATUM 0 device. Commercial STRATUM clocks are  
*   very expensive.
* 
* The software waits for a $GPRMC sentence from a GPS device; i.e., Adafruit 
*   Ultimate GPS and parses the message for DATE and TIME information. The
*   sketch waits for valid messages, extracts the DATE and TIME fields as
*   which are strings, then converts each character of of the string to binary 
*   character equivalents representing LEDs in each 7-segment display, then
*   sends this data along with appropriate register addresses (opcodes) to the 
*   MAX2719 devices.  Since two MAX7219 are daisy-chained, two bytes are 
*   sent in sequence for each of 16 individual display modudles constituting 
*   256 bits every second. Data is sent matching the following pattern:
*   right digit 1 opcode, right digit 1 data, left digit 1 opcode, left digit
*   1 data, then right digit 2 opcode, right digit 2 data and so forth for
*   all 16 digits (8 right and 8 left).  It has to be done this way because
*   the MAX7219 are not individually addressible.
*
* The TIME data is corrected to account for UTC + local offset. Additionally,
*   a switch is evaluated for Daylight Savings Time. If the switch is on/off 
*   that value adjusts the UTC offset plus/minus 1 hour.
* -----------------------------------------------------------------------------
* THIS SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
*   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
*   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
*   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
*   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING 
*   FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER 
*   DEALINGS IN THE SOFTWARE.
* -----------------------------------------------------------------------------
*/
#include <max7219.h>
#include <SoftwareSerial.h>

// Define all gaz-ins and gaz-outs.
#define GPS_RX_PIN 2 // Connect GPS TX to this pin
#define GPS_TX_PIN 3 // Connect GPS RX to this pin
#define DST_PIN 7 // Interfaces to Daylight Savings Time switch
//#define COLON_PIN 8  // Pin used to flash clock colon, once per second
#define MAX7219_CLK 10 // Clock Pin for Max7219
#define MAX7219_CS 11 // LOAD/CS Pin for Max7219
#define MAX7219_DIN 12 // Data Pin for Max7219

// Define opcodes (MAX7219 register addresses).
#define REG_NO_OP 0x00 // No-Op, not used, will probably delete this
#define REG_DECODE 0x09 // Decode digits or standard 7-segment digits
#define REG_INTENSITY 0x0a // Brightness
#define REG_SCAN_LIMIT 0x0b // Scan Limit for registers
#define REG_SHUTDOWN 0x0c // Shutdown or Run
#define REG_DISP_TEST 0x0f // Display test pattern (on/off)

// Define 7-Segment display LED positions; i.e., segment A, B, C, D, E, F, G,
//   and DP.  A one value turns on an LED segment.  The MSB is sent first. For
//   example: for a number 2, the transmitted bits would be MSB 01101101 LSB.
//   Bit 8 (from right) is DP, i.e. 0b.ABCEDEFG).
const byte sevenSegmentDigits[10] = {
  0b01111110, // 0, 0x7e            
  0b00110000, // 1, 0x30            A 
  0b01101101, // 2, 0x6d          -----
  0b01111001, // 3, 0x79         |     | B
  0b00110011, // 4, 0x33       F |  G  |
  0b01011011, // 5, 0x5b          -----
  0b01011111, // 6, 0x1f         |     | C
  0b01110000, // 7, 0x70       E |     |
  0b01111111, // 8, 0x7f          -----   DP
  0b01111011  // 9, 0x73            D  
};

// Define Register addresses (opcodes) for the MAX7219 devices.
//   Bit 0 - 3 are used, bits 4 - 7 are don't care.  
const byte sevenSegmentPosition[8] = {
  0x08, // Module 7, 0b00001000 (left most)
  0x07, // Module 6, 0b00000111               M A X 7 2 1 9
  0x06, // Module 5, 0b00000110      M o d u l e   P o s i t i o n s
  0x05, // Module 4, 0b00000101     ---------------------------------
  0x04, // Module 3, 0b00000100     | 8 | 7 | 6 | 5 | 4 | 3 | 2 | 1 |
  0x03, // Module 2, 0b00000011     ---------------------------------
  0x02, // Module 1, 0b00000010
  0x01, // Module 0, 0b00000001 (right most)
};

// Constants and ints for UTC offset and Daylight Savings Time.
int timeZoneOffsetHours = -4;  // For example, UTC-5 for Eastern Standard Time (EST).
int timeZoneOffsetMinutes = 0; // No minutes offset.
int dstStatus = 0; // Stores value of the DST switch position. 
const int daylightSavingsTimeOn = -4; // Used for summer.
const int daylightSavingsTimeOff = -5; // Used when it's winter.

// Set up a serial port for the GPS input.  This will run at 9600 baud.
SoftwareSerial gpsSerial(GPS_RX_PIN, GPS_TX_PIN);  

MAX7219 max7219;  // Working to remove this dependancy. 

// Function to convert a string character to its corresponding 7-segment 
//   LED binary representation.
byte charToSevenSegment(char character) {
  if (character >= '0' && character <= '9') {
    return sevenSegmentDigits[character - '0'];
  } else {
    // If character is not a digit, return 0 (all segments off).
    return 0;
  }
}

// Function to write various register addresses (opcodes) and the register's 
//   data.  Register addresses  are  sent  first, then the register's 
//   data. MSB goes first.  The MAX7219 furthest from the first DIN input
//   must be written to first.  See initializeMax7219 functions below 
//   for details on registers and data structure.
//
// When the actual DATE and TIME are sent, a time digit is sent first,
//   followed by one digit of the DATE.  This is done from right to left
//   eight times to complete the displayed date and time.  
// 
// Since the shiftOut command can only handle 8 bits at a time, the command
//   is repeated four times to send all bits; for each two (one right, and
//   one left) digits.  Every digit requires one address byte, and one
//   data byte (16 bits).  256 bits will be shifted every second, in 8 sets 
//   of 32 bits to complete the displayed date and time.
// 
// The register address is sent first, followed by the address's data.  See 
//   page 6 of the MAX7219 datasheet. The last sentence of "Serial_
//   Addressing Modes" shows that data bit D15 is to be sent first... this is
//   crazy important.
//
// D15  D14  D13  D12  D11  D10  D09  D08  D07  D06  D05  D04  D03  D02  D01  D00
//  X    X    X    X   |  A D D R E S S |  |MSB  .    .   D A T A    .   .   LSB|
//  X = Don't Care, 15 addressible registers. Eight are for digits, and 6 are for
//  opcodes (addresses).
// 
unsigned int writeRegisters(int opcodeR, int dataR, int opcodeL, int dataL){
  digitalWrite(MAX7219_CS, LOW);
  shiftOut(MAX7219_DIN, MAX7219_CLK, MSBFIRST, opcodeR); // opcode right
  shiftOut(MAX7219_DIN, MAX7219_CLK, MSBFIRST, dataR);   // data right
  shiftOut(MAX7219_DIN, MAX7219_CLK, MSBFIRST, opcodeL); // opcode left
  shiftOut(MAX7219_DIN, MAX7219_CLK, MSBFIRST, dataL);   // data left
  digitalWrite(MAX7219_CS, HIGH);
}

// Initialize MAX7219 before starting to display output data.
void initializeMax7219(){
  // writeRegisters( Right opcode, right data, left opcode, dleft data ).
  writeRegisters(REG_SCAN_LIMIT, 0x05, REG_SCAN_LIMIT, 0x07); // Set Scan Limit to 5 (time) and 7 (date).
  writeRegisters(REG_DECODE, 0x00, REG_DECODE, 0x00); // Set Decode Mode to 7-Segment (No decode).
  writeRegisters(REG_SHUTDOWN, 0x01, REG_SHUTDOWN, 0x01); // End Shutdown Mode.
  writeRegisters(REG_DISP_TEST, 0x00, REG_DISP_TEST, 0x00); // End Test Pattern.
  writeRegisters(REG_INTENSITY, 0x06, REG_INTENSITY, 0x07); // Set display brightness, 07 = midrange.
  clearLEDRegisters();  
}

// Reset all Module LED register data. LED registers are 1 thru 8.
void clearLEDRegisters(){
  for (int i = 0; i < 8; i++) {
  //              Left Digit   Right Digit
  writeRegisters(i + 1, 0x00, i + 1, 0x00); // Registers 1 - 8, set to 0
  }
}

// Function to parse GPS data for date and time.
void parseGPSData(String data) {
  // Check if the data starts with the GPRMC sentence.
  if (data.startsWith("$GPRMC")) {
    // Split the data by commas
    String parts[13];
    int commaIndex = 0;
    int partsIndex = 0;
    for (int i = 0; i < data.length(); i++) {
      if (data[i] == ',') {
        parts[partsIndex++] = data.substring(commaIndex, i);
        commaIndex = i + 1;
      }
    }
    parts[partsIndex] = data.substring(commaIndex); // Last part after the last comma

    // Check if the data is valid (the second element is 'A' for active).
    //   Parsing NEMA $GPRMC sentence for "A".
    if (parts[2] == "A") {
      
      // Extract time (UTC) from the first part (index 1)
      String timeStrUTC = parts[1].substring(0, 2) + parts[1].substring(2, 4) + 
                          parts[1].substring(4, 6);

      // Extract date from the ninth part (index 8).  Since the GPS data is only
      //   a two-digit year "20" is added to make "2024".  Parts 2 and 0 add 
      //   the month and day.
      String dateStr = "20" + parts[9].substring(4, 6) + parts[9].substring(2, 4) + 
                        parts[9].substring(0, 2);
      
      // Apply time zone offset to UTC time
      int hours = timeStrUTC.substring(0, 2).toInt() + timeZoneOffsetHours;
      int minutes = timeStrUTC.substring(2, 4).toInt() + timeZoneOffsetMinutes;
      int seconds = timeStrUTC.substring(4, 6).toInt();

      // Adjust hours, minutes, and seconds for overflow and underflow.
      if (minutes < 0) {
        hours--;
        minutes += 60;
      }
      if (minutes >= 60) {
        hours++;
        minutes -= 60;
      }
      if (hours < 0) {
        hours += 24;
      }
      if (hours >= 24) {
        hours -= 24;
      }
      // Format adjusted time to account for UTC.  Also, add "00" to the start
      //   of the time string since only eight digits are needed, but there are
      //   eight digit positions.  The initializeMax7219 function turns off the
      //   first two digits of the TIME display - see REG_SCAN_LIMIT.
      String adjustedTimeStr = "00" + String(hours / 10) + String(hours % 10) + 
                              String(minutes / 10) + String(minutes % 10) + 
                              String(seconds / 10) + String(seconds % 10);
      
      // Send zeros to all module registers between updates.
      //clearLEDRegisters();  // This call is causing the LEDs to blink once between updates.

      // Assign Date and Time strings to strings used for bit calculations.
      //   dateString looks like 20240410 for 10 Apr 2024, and adjustedTimeStr
      //   looks like 150622 for 3:06 PM and 22 seconds.
      String string1 = dateStr.c_str(); // Parsed date
      String string2 = adjustedTimeStr.c_str(); // Parsed time

      // Convert strings to binary representations
      byte binaryData1[16]; // Binary data for display 1
      byte binaryData2[16]; // Binary data for display 2

      // Calculate the digit position address (0bXXXX0001 to 0bXXXX1000).
      for (int i = 0; i < 8; i++){
        byte address1 = sevenSegmentPosition[i];
        byte address2 = sevenSegmentPosition[i + 1];

        // Get the digit at position [i] in string1 and string2.
        char character1 = string1[i];
        char character2 = string2[i + 1];
        
        // Get the 7-segment binary representation for the current digit.
        byte segment1 = charToSevenSegment(character1);
        byte segment2 = charToSevenSegment(character2);

        // Assign the address and segment data to the binary data arrays.
        binaryData1[i] = address1; // Max7219 #1, most right (opcode)
        binaryData1[i + 1] = segment1; // Max7219 #1 (data)
        binaryData2[i] = address2; // Max7219 #2 (opcode)
        binaryData2[i + 1] = segment2; // Max7219 #2 (data)

        // Send results to displays via the writeRegisters function for, 32 
        //   bits total. This  writes the data first to the right module, 
        //   then the left, alternating right then left.  It does this eight 
        //   times every second.
        writeRegisters(binaryData2[i], binaryData2[i + 1], binaryData1[i], binaryData1[i + 1]);
      }
      //Serial.println("  Date:" + dateStr + ", Time:" + adjustedTimeStr); 
    }
  }
}

// Runs at start-up.  Configures pins and functions.
void setup() {
  Serial.begin(9600);
  initializeMax7219(); // Get the MAX7219 ready
  gpsSerial.begin(9600);
  //pinMode(COLON_PIN, OUTPUT); // Colon for clock, Pin 8.
  pinMode(DST_PIN, INPUT); // Daylight Savings time switch, Pin 7
  pinMode(MAX7219_CLK, OUTPUT); // Clock Pin 10 
  pinMode(MAX7219_CS, OUTPUT); // CS Pin 11
  pinMode(MAX7219_DIN, OUTPUT); // Data Pin 12
}

// This loop runs constantly and is a key part of the clock's capabilities.
//   In this loop the code waits for a GPS NEMA 0183 $GPRMC sentence.  Once 
//   received, the parseGPSData function is called and the LED modules are 
//   updated via functions (above).
void loop() {
  if (gpsSerial.available() > 0) {

    // Read data from GPS module.
    String data = gpsSerial.readStringUntil('\n');

    // Check if data contains time information (assuming it starts with $GPRMC).
    if (data.startsWith("$GPRMC")) {
      // Parse GPS data for time and do the other juicy stuff.
      parseGPSData(data);
    }

    // Check to see if the DST switch is on. If so, then update timeZoneOffsetHours 
    //   accordingly to account for either 4 or 5 hours of offset from UTC.
    dstStatus = digitalRead(DST_PIN); // Sets value of dstStatus depending on pin 7
    if (dstStatus == HIGH) { 
      timeZoneOffsetHours = daylightSavingsTimeOn;
    }
    else {
      timeZoneOffsetHours = daylightSavingsTimeOff;
    }
    // Code for the CLOCK's colon is not here yet.
  }
}
