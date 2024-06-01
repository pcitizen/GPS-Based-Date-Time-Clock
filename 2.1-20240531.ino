/*
* GPS-Based DATE and TIME Clock, v2.1.20240531
* 
* Full disclosure: After the first version of this sketch was completed and
*   was working properly, the code was submitted to chatGPT for optimization.  
*   This version is the end result.  The most significant change was to the
*   "displayDateAndTime" function.  The original code worked by force.  This
*   version is more elegant; which was chatGPT's input.  Additionally, all 
*   dependancies on the max7219.h library were removed since that library only
*   supports one MAX7219 device; i.e, eight 7-segment displays.  This sketch 
*   currently supports two and will eventually drive six or maybe seven sets of
*   eight 7-segment displays.
* 
* What does this code do? It...
*   Extracts NEMA 0183 $GPRMC sentence (Date and Time) information from a GPS 
*   module's output data stream and sends this data to a set of MAX7219 
*   7-Segment 8-digit common cathode display drivers. This is sort of like a 
*   STRATUM 0 device but without network communications.  Networking will come
*   later. :)
* 
* How does it do it?  It...
*   Waits for a $GPRMC sentence from a GPS device that is received via serial
*   interface (currently pins 2 and 3).  Once received, the sketch parses the 
*   sentence for DATE and TIME information; which is received as text-based
*   strings. Then it converts each character of the strings to binary 
*   character equivalents representing LEDs in each 7-segment display, then
*   this binary information is sent along with appropriate register addresses 
*   (opcodes) to the MAX2719 devices. Since three MAX7219 are daisy-chained, 
*   six bytes are sent in sequence for each of 24 individual display modules 
*   constituting 384 bits every second. Data is sent matching the following 
*   pattern: right display digit 1 opcode, right display digit 1 data, center
*   display digit 1 opcode, center display digit 1 data, then right display 
*   digit display 2 opcode, right display digit 2 data and so forth for all 24
*   digits (8 right, 8 center, and 8 left).  Each digit requires two bytes. One
*   byte is the position of the digit in the display and the second byte is for
*   7-segment display LEDs. Data has to be sent in series like this because 
*   the MAX7219 device is not individually addressible.
*
* What else does it do?  
*   The DATE and TIME data is corrected to account for UTC and local offset. 
*   Additionally, a manual switch position is evaluated for Daylight Savings 
*   Time. If the switch is on/off that value adjusts the UTC offset plus/minus
*   1 hour. Also, the software tabulates the number of days since January 1 of 
*   the current year and displays this information as well using three digits.
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

// Include a library for serial output.
#include <SoftwareSerial.h>

// Define all gaz-ins and gaz-outs used in the sketch.
#define ARD_RX_PIN 2   // Connect GPS TX to this pin
#define ARD_TX_PIN 3   // Connect GPS RX to this pin
#define DST_PIN 7      // Supports Daylight Savings Time switch
#define MAX7219_CLK 10 // Clock Pin for Max7219
#define MAX7219_CS 11  // LOAD/CS Pin for Max7219
#define MAX7219_DIN 12 // Data In Pin for Max7219
#define COLON_PIN 4    // Used for clock colons

// Define opcodes (MAX7219 register configuration addresses).
#define REG_NO_OP 0x00      // No-Op, not used, will probably delete this
#define REG_DECODE 0x09     // Decode digits or standard 7-segment digits
#define REG_INTENSITY 0x0a  // Brightness
#define REG_SCAN_LIMIT 0x0b // Scan Limit for registers
#define REG_SHUTDOWN 0x0c   // Shutdown or Run
#define REG_DISP_TEST 0x0f  // Display test pattern (on/off)

// Define 7-Segment display LED positions; i.e., segment A, B, C, D, E, F, G,
//   and DP.  A one value turns on an LED segment.  The MSB is sent first. For
//   example: for a number 2, the transmitted bits would be MSB 01101101 LSB.
//   Bit 8 (from right) is DP, i.e. 0b.ABCEDEFG).  
const byte sevenSegmentDigits[10] = {
  0b01111110, // 0, #126, 0x7e             
  0b00110000, // 1, #048, 0x30            A 
  0b01101101, // 2, #109, 0x6d          -----
  0b01111001, // 3, #121, 0x79         |     | B
  0b00110011, // 4, #051, 0x33       F |  G  |
  0b01011011, // 5, #091, 0x5b          -----
  0b01011111, // 6, #095, 0x1f         |     | C
  0b01110000, // 7, #112, 0x70       E |     |
  0b01111111, // 8, #127, 0x7f          -----   DP
  0b01111011  // 9, #123, 0x73            D  
};

// Define Register addresses (opcodes) for the MAX7219 devices. Only bit
//   0 - 3 are used, bits 4 - 7 are don't care/ignored.  
const byte sevenSegmentPosition[8] = {
  0x08, // Module 7, 0b00001000 (left most)
  0x07, // Module 6, 0b00000111               M A X 7 2 1 9
  0x06, // Module 5, 0b00000110      M o d u l e   P o s i t i o n s
  0x05, // Module 4, 0b00000101     ---------------------------------
  0x04, // Module 3, 0b00000100     | 8 | 7 | 6 | 5 | 4 | 3 | 2 | 1 |
  0x03, // Module 2, 0b00000011     ---------------------------------
  0x02, // Module 1, 0b00000010       Left                    Right
  0x01, // Module 0, 0b00000001 (right most)
};

// Constants and ints for UTC offset and Daylight Savings Time.
int timeZoneOffsetHours = -4;  // UTC-5 for Eastern Standard Time (EST).
int timeZoneOffsetMinutes = 0; // No minutes offset.
int dstStatus = 0; // Stores the value of the DST switch position. 
const int daylightSavingsTimeOn = -4; // Used for summer.
const int daylightSavingsTimeOff = -5; // Used when it's winter.

// Set up a serial port for the GPS's input.  This will run at 9600 baud.
SoftwareSerial gpsSerial(ARD_RX_PIN, ARD_TX_PIN);

// Function prototypes... I am not sure if these are necessary.
void initializeMax7219();
void displayDateAndTime(String date, int hours, int minutes, int seconds, 
    int NumberOfDays);
void sendRegister(byte opcodeR, byte dataR, byte opcodeL, byte dataL, 
    byte opcodeD, byte dataD);
void clearLEDRegisters();

// Runs at start-up. Configures pins and functions.
void setup() {
  Serial.begin(9600);
  gpsSerial.begin(9600);
  pinMode(MAX7219_CLK, OUTPUT);
  pinMode(MAX7219_CS, OUTPUT);
  pinMode(MAX7219_DIN, OUTPUT);
  pinMode(COLON_PIN, OUTPUT);
  pinMode(DST_PIN, INPUT);
  initializeMax7219();
}

// This function is used to ensure the local date's displayed information does
//   not change when the UTC date rolls over to the next day. 
void adjustDate(String &dateStr, int localHours, int utcHours) {
  int year = dateStr.substring(0, 4).toInt();
  int month = dateStr.substring(4, 6).toInt();
  int day = dateStr.substring(6, 8).toInt();
  
  // If local time is behind UTC time, adjust the date to the previous day.
  if (localHours < utcHours) {
    day -= 1;
    if (day == 0) {
      month -= 1;
      if (month == 0) {
        month = 12;
        year -= 1;
      }
      day = daysInMonth(month, year); // Get number of days in current month.
    }
  }

  // Assemble date string data into the current DATE; i.e., YYMMDD.
  dateStr = String(year) + String(month / 10) + String(month % 10) + 
            String(day / 10) + String(day % 10);
}

// This loop runs constantly and is required for Arduino sketches.
//   In this loop the code waits for GPS NEMA 0183 $GPRMC sentences.  Once 
//   received, the parseGPSData function is called and the LED modules are 
//   updated.
void loop() {
  if (gpsSerial.available() > 0) {
    String data = gpsSerial.readStringUntil('\n');
    if (data.startsWith("$GPRMC")) {
      parseGPSData(data);
    }
    // Check to see if the DST switch is on. If so, then update 
    //   timeZoneOffsetHours accordingly to account for offset from UTC.
    dstStatus = digitalRead(DST_PIN);
    if (dstStatus == HIGH) {
      timeZoneOffsetHours = daylightSavingsTimeOn;
    }
    else {
      timeZoneOffsetHours = daylightSavingsTimeOff;
    }
  }
  // Used for testing... using static test variables
  //displayDateAndTime("20240428", 8, 3, 55); // 28 Apr 2024, 8:03:01 AM
}

// Function that parses the GPS data's $GPRMC output sentence then 
//   cuts the sentence's string values into parts; i.e, DATE and TIME. Once
//   done the "parts" are subdivided for preparation of other functions.
void parseGPSData(String data) {
  if (data.startsWith("$GPRMC")) {   // Looking for the $GPRMC string from GPS
    String parts[13];  // The $GPRMC sentence has 13 parts
    int commaIndex = 0;
    int partsIndex = 0;
    for (int i = 0; i < data.length(); i++) { // Using comma as field separator
      if (data[i] == ',') {
        parts[partsIndex++] = data.substring(commaIndex, i);
        commaIndex = i + 1;
      }
    }
    // Assign string parts to an array.
    parts[partsIndex] = data.substring(commaIndex); 
    
    // Here... the substrings are chopped into parts.
    if (parts[2] == "A") {  // Testing to see of the $GPRMC string is valid 
      String timeStrUTC = parts[1].substring(0, 2) + parts[1].substring(2, 4) + 
                          parts[1].substring(4, 6);
      String dateStr = "20" + parts[9].substring(4, 6) + 
                        parts[9].substring(2, 4) + parts[9].substring(0, 2);
      int utcHours = timeStrUTC.substring(0, 2).toInt();
      int localHours = utcHours + timeZoneOffsetHours;
      int minutes = timeStrUTC.substring(2, 4).toInt() + timeZoneOffsetMinutes;
      int seconds = timeStrUTC.substring(4, 6).toInt();

      // Adjust hours, minutes, and seconds for overflow and underflow should 
      //   it occur.
      if (minutes >= 60) {
        minutes -= 60;
        localHours += 1;
      }
      else if (minutes < 0) {
        minutes += 60;
        localHours -= 1;
      }
      if (localHours >= 24) {
        localHours -= 24;
        adjustDate(dateStr, localHours, utcHours);
      }
      else if (localHours < 0) {
        localHours += 24;
        adjustDate(dateStr, localHours, utcHours);
      }

      // Tabulate the number of days since Jan 1 of the current year.
      int numberOfDays = getNumberOfJulianDays(dateStr);

      // Send all date to displays.
      displayDateAndTime(dateStr, localHours, minutes, seconds, numberOfDays);
    }
  }
}

// Function to calculate number of days in the year so far.
int getNumberOfJulianDays(String date) {
    int year = date.substring(0, 4).toInt(); // Get year part of date
    int month = date.substring(4, 6).toInt(); // Get month part of date
    int day = date.substring(6, 8).toInt(); // Get day part of date
    int daysCount = 0; // Initialize daysCount
    // Iterate from January to the current previous month and add up days
    for (int cnt = 1; cnt <= month - 1; cnt++) {
        int days = daysInMonth(cnt, year); // Add days of each month together
        daysCount += days;
    }
    return daysCount + day; // Return the total number of Julian days
}

// Array for the days in each month and adjusts for leapyear too. Thanks
//   chatGPT.
int daysInMonth(int month, int year) {
  int daysInMonth[] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
  if ((year % 4 == 0 && year % 100 != 0) || (year % 400 == 0)) {
    daysInMonth[1] = 29;
  }
    return daysInMonth[month - 1];
}

//  Initialize the MAX7219 device's configuration on startup or otherwise.
//    This is showing values for the rightmost then leftmost displays.
void initializeMax7219() {
  // Set No. digits
  sendRegister(REG_SCAN_LIMIT, 0x04, REG_SCAN_LIMIT, 0x05, 
    REG_SCAN_LIMIT, 0x07); 
  // No display decoding
  sendRegister(REG_DECODE, 0x00, REG_DECODE, 0x00, REG_DECODE, 0x00);
  // Enable displays
  sendRegister(REG_SHUTDOWN, 0x01, REG_SHUTDOWN, 0x01, REG_SHUTDOWN, 0x01);
  // Turn off test mode
  sendRegister(REG_DISP_TEST, 0x00, REG_DISP_TEST, 0x00, REG_DISP_TEST, 0x00);
  // Set brightness 
  sendRegister(REG_INTENSITY, 0x00, REG_INTENSITY, 0x0a, REG_INTENSITY, 0x0b);
  clearLEDRegisters(); 
}

// Function to write register addresses (opcodes) and the register's date.
//
// When the DATE and TIME are sent, a time digit is sent first, followed by
//   one digit of the DATE.  This is done from right to left eight times to 
//   complete the displayed date and time.  
// 
// Since the shiftOut command can only handle 8 bits at a time, the command
//   is repeated four times to send all bits; for each two (one right, and
//   one left) digits.  Every digit requires one address byte, and one
//   data byte (16 bits).  256 bits will be shifted every second, in 8 sets 
//   of 32 bits to complete the displayed date and time at that second.
// 
// The register address is sent first, followed by the address's data.  See 
//   page 6 of the MAX7219 datasheet. The last sentence of "Serial_
//   Addressing Modes" shows that data bit D15 (MSB) is to be sent first. 
//   The MAX7219 furthest from the first DIN input must be written to first.
//
// D15 D14 D13 D12 D11 D10 D09 D08 D07 D06 D05 D04 D03 D02 D01 D00
//  X   X   X   X  |   ADDRESS   | |MSB  .  .  D A T A  . .   LSB|
//  X = Don't Care, 15 addressible registers. Eight are for digits, and 6 are 
//    for opcodes (addresses).
// 
void sendRegister(byte opcodeJDays, byte dataJDays, byte opcodeTime, byte dataTime, 
    byte opcodeDate, byte dataDate) {
  digitalWrite(MAX7219_CS, LOW);
  // First out is the furthest module from the dataOut pin of Arduino.
  shiftOut(MAX7219_DIN, MAX7219_CLK, MSBFIRST, opcodeJDays);  // opcode JDays
  shiftOut(MAX7219_DIN, MAX7219_CLK, MSBFIRST, dataJDays);    // data JDays
  shiftOut(MAX7219_DIN, MAX7219_CLK, MSBFIRST, opcodeTime);  // opcode Time
  shiftOut(MAX7219_DIN, MAX7219_CLK, MSBFIRST, dataTime);    // data Time
  shiftOut(MAX7219_DIN, MAX7219_CLK, MSBFIRST, opcodeDate);  // opcode Date
  shiftOut(MAX7219_DIN, MAX7219_CLK, MSBFIRST, dataDate);    // data Date
  digitalWrite(MAX7219_CS, HIGH);
}

// Reset all Module LED register data. LED registers are 1 thru 8.
void clearLEDRegisters() {
  for (int i = 0; i < 8; i++) {
    sendRegister(i + 1, 0x00, i + 1, 0x00, i + 1, 0x00); //Clear all segments
  }
}

// This function takes the current DATE and TIME (in segments) and sends it to 
//   the MAX7219 devices.  The function cuts up the strings into eight 
//   separate digits, determine their segment (charToSevenSegemnt) derivatives,
//   and each digit's position (sevenSegmentPosition) then uses the 
//   sendRegister function to send the data to the display.
void displayDateAndTime(String date, int hours, int minutes, int seconds, 
    int numberOfDays) {
  hours %= 24; // Ensure hours are within 0-23
  // If the local time is between 8PM and midnight, adjust the date -1 day.
  //   This accounts for UTC -offset.
  if (hours >= 20 && hours < 24) {
    int year = date.substring(0, 4).toInt();
    int month = date.substring(4, 6).toInt();
    int day = date.substring(6, 8).toInt();
    day -= 1; // Subtract one day
    numberOfDays -= 1; // Subtract a day from Julian days too
    if (day == 0) {
      month -= 1;
      if (month == 0) {
        month = 12;
        year -= 1;
      }
      day = daysInMonth(month, year);
    }
    // Assemble date string parts (year, month, day) into the current DATE for
    //  display; i.e, YYYYMMDD.
    date = String(year) + String(month / 10) + String(month % 10) + 
            String(day / 10) + String(day % 10);
  }

  // Format number of Julian days with preceding two digits for the year plus
  //  three 0s. The zeros are not displayed since the SCAN LIMIT is set to 4.
  String julianDays = "000" + date.substring(2, 4) + 
      String(numberOfDays / 100) + String(numberOfDays % 100);

  // Two zeros are added to the TIME to right justify the data. These two
  //   digits are not seen because when the MAX7219 is initialized it is 
  //   configured to only display six characters. See the initializeMax7219 
  //   function (REG_SCAN_LIMIT) set to 0x05; i.e., digits 0 to 5, therefore a 
  //   value of 5 displays 6 digits.
  String timeStr = "00" + String(hours / 10) + String(hours % 10) +
                   String(minutes / 10) + String(minutes % 10) +
                   String(seconds / 10) + String(seconds % 10);

  // Check each character in the DATE, TIME and DAYS strings and get their 
  //   segment patterns and positions then send them to the MAX7219's 
  //   registers via the sendRegister function.                 
  for (int i = 0; i < 8; i++) {
    char timeDigit = timeStr.charAt(i);
    char dateDigit = date.charAt(i);
    char daysDigit = julianDays.charAt(i);
    byte timeData = charToSevenSegment(timeDigit); // Convert char to segments
    byte dateData = charToSevenSegment(dateDigit); // Convert char to segments
    byte daysData = charToSevenSegment(daysDigit); // Convert char to segments
    byte position = sevenSegmentPosition[i]; // Set digit position
    // Send the data.
    sendRegister(position, daysData, position, timeData, position, dateData);
  }
}

// Function to convert a string character to its corresponding 7-segment 
//   LED binary representation.  If a digit is not found, then return 0.
byte charToSevenSegment(char character) {
  if (character >= '0' && character <= '9') {
    return sevenSegmentDigits[character - '0'];
  }
  else {
    return 0;
  }
}
