/*
* GPS-Based DATE and TIME Clock, v2.5.20250308
*
* Changed v2.5.20250308. Changed the Julian Day code because it was one day behind the current
*   day after Feb 28th of the non-Leap Year year.
*
* Change v2.4.20250111. Minor tweak.  Adjusted the SCAN LIMIT for the Julian Day display to
*   move the character one character to the right. This makes the YYDDD display, closer to center.
*
* Change v2.4.20250110. Change 'void setup()' to use Gramin GPS 18X LVC instead of the 
*   Ultimate GPS module.  I found that the ultimate GPS showed update delays to the seconds
*   about every four seconds.  I changed the baud rate faster with no decernible results.
*   There is no delay with the Garmin device. This change also reflects that the 
*   Serial.begin is set to 19200 baud.
*
* Bug Fix in v2.3.20250101.  The Julian Date display was incorrect when the 
*   julianDateConfiguration switch was in the OFF position.  The display should have shown
*   25001, but instead showed 00251.  Modified the "if (calendarConfiguration == 1){" code
*   segment to correct the function.
* 
* Full disclosure: After the first version of this sketch was completed and was working properly, 
*   the code was submitted to chatGPT for optimization.  This version is the end result.  The 
*   most significant change was to the "displayDateAndTime" function.  The original code worked 
*   by force.  This version is more elegant; which was chatGPT's input.  Additionally, all 
*   dependancies on the max7219.h library were removed since that library only supports one 
*   MAX7219 device; i.e, eight 7-segment displays.  This sketch currently supports two and will 
*   eventually drive six or maybe seven sets of eight 7-segment displays.
* 
* What does this code do? It...
*   Extracts NEMA 0183 $GPRMC sentence (Date and Time) information from a GPS module's output 
*   data stream and sends this data to a set of MAX7219 7-Segment 8-digit common cathode 
*   display drivers. This is sort of like a STRATUM 0 device but without network communications.
*   Networking will come later. :)
* 
* How does it do it?  It...
*   Waits for a $GPRMC sentence from a GPS device that is received via serial interface 
*   (currently pins 2 and 3).  Once received, the sketch parses the sentence for DATE and TIME
*   information; which is received as text-based strings. Then it converts each character of 
*   the strings to binary character equivalents representing LEDs in each 7-segment display, 
*   then this binary information is sent along with appropriate register addresses (opcodes) to 
*   the MAX2719 devices. Since three MAX7219 are daisy-chained, six bytes are sent in sequence 
*   for each of 24 individual display modules constituting 384 bits every second. Data is sent
*   matching the following pattern: right display digit 1 opcode, right display digit 1 data, 
*   center display digit 1 opcode, center display digit 1 data, then right display digit display 
*   2 opcode, right display digit 2 data and so forth for all 24 digits (8 right, 8 center, and 8 
*   left).  Each digit requires two bytes. One byte is the position of the digit in the display 
*   and the second byte is for 7-segment display LEDs. Data has to be sent in series like this 
*   because the MAX7219 device is not individually addressible.
*
* What else does it do?  
*   The DATE and TIME data is corrected to account for UTC and local offset. Additionally, a manual
*   switch position is evaluated for Daylight Savings Time. If the switch is on/off that value 
*   adjusts the UTC offset plus/minus 1 hour. Also, the software tabulates the number of days since 
*   January 1 of the current year and displays this information (Julian Days) using three digits.
*   Another switch is used to add/removed the current year, which is prepended before the Julian Days
*   A third switch was added to allow the user to configure the DATE display between YYYYMMDD and 
*   MMDDYYYY.
* -----------------------------------------------------------------------------
* THIS SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT 
*   NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND 
*   NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, 
*   DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, 
*   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
* -----------------------------------------------------------------------------
*/

// Include a library for software-based serial output.
#include <SoftwareSerial.h>

// Define all gaz-ins and gaz-outs used in the sketch.
#define ARD_RX_PIN 2   // Connect GPS TX to this pin
#define ARD_TX_PIN 3   // Connect GPS RX to this pin
#define DST_PIN 7      // Supports Daylight Savings Time switch
#define CAL_CONF_PIN 6 // Used for clock display pattern yyyymmdd or ddmmyyyy
#define JUL_DATE_PIN 5 // Julian date configuration YYDDD or DDD.
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

// Define 7-Segment display LED positions; i.e., segment A, B, C, D, E, F, G, and DP.  A one value
//   turns on an LED segment.  The MSB is sent first. For example: for a number 2, the transmitted 
//   bits would be MSB 01101101 LSB. Bit 8 (from right) is DP, i.e. 0b.ABCEDEFG).  
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

// Define register addresses (opcodes) for digit positions. Only bit 0 - 3 are used.  
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
int calendarConfiguration = 0; // Keeps track of date configuration pattern.
int julianDateConfiguration = 0; // Keeps track of Julian display configuration.
const int daylightSavingsTimeOn = -4; // Used for summer.
const int daylightSavingsTimeOff = -5; // Used when it's winter.
int offset = -4; // Used for determining which date to display. See Line 354ish.
String dateStr = "20250105";
String julianDays = "00005";

// Set up a serial port for the GPS's input.  This will run at 9600 baud.
SoftwareSerial gpsSerial(ARD_RX_PIN, ARD_TX_PIN);

// Function prototypes... I am not sure if these are necessary.
void initializeMax7219();
void displayDateAndTime(String date, int hours, int minutes, int seconds, int NumberOfDays);
void sendRegister(byte opcodeR, byte dataR, byte opcodeL, byte dataL, byte opcodeD, byte dataD);
void clearLEDRegisters();

// Runs at start-up. Configures pins and functions.
void setup() {
  Serial.begin(19200); // Used for comms with MAX7219
  gpsSerial.begin(4800); // Garmin GPS 18X LVS (4800 Baud)
  //gpsSerial.begin(9600); // Ultimate GPS.  Works, but there is a delay about every 4 seconds.
  pinMode(MAX7219_CLK, OUTPUT);
  pinMode(MAX7219_CS, OUTPUT);
  pinMode(MAX7219_DIN, OUTPUT);
  pinMode(COLON_PIN, OUTPUT);
  pinMode(DST_PIN, INPUT);
  pinMode(CAL_CONF_PIN, INPUT);
  pinMode(JUL_DATE_PIN, INPUT);
  initializeMax7219();
}

// This function is used to ensure the local date's displayed information does not change when the 
//   UTC date rolls over to the next day. 
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
}

// This loop runs constantly and is required for Arduino sketches. In this loop the code waits for
//   GPS NEMA 0183 $GPRMC sentences.  Once received, the parseGPSData function is called and the 
//   LED modules are updated.
void loop() {
  if (gpsSerial.available() > 0) {
    String data = gpsSerial.readStringUntil('\n');
    if (data.startsWith("$GPRMC")) {
      parseGPSData(data);
    }
  }

  // Check to see if the DST switch is on. If so, then update timeZoneOffsetHours accordingly to 
  //   account for offset from UTC.
  if (digitalRead(DST_PIN) == HIGH) {
    timeZoneOffsetHours = daylightSavingsTimeOn;
  }
  else {
    timeZoneOffsetHours = daylightSavingsTimeOff;
  }

  // Check date display configuration pin.
  if (digitalRead(CAL_CONF_PIN) == HIGH) {
    calendarConfiguration = 1;
  }
  else {
    calendarConfiguration = 0;
  }

  // Check Julian day display configuration pin.
  if (digitalRead(JUL_DATE_PIN) == HIGH) {
    julianDateConfiguration = 1;
  }
  else {
    julianDateConfiguration = 0;
  }
  
  // Used for testing... using static test variable displayDateAndTime("20240428", 8, 3, 1); // 
  //   28 Apr 2024, 8:03:01 AM
}

// Function that parses the GPS data's $GPRMC output sentence then cuts the sentence's string values
//   into parts; i.e, DATE and TIME. Once done the "parts" are subdivided for preparation of other 
//   functions.
void parseGPSData(String data) {
  //data = "$GPRMC,000001,A,3704.8298,N,07624.7225,W,0.12,266.69,070125,,,D*7F"; // Test data
  if (data.startsWith("$GPRMC")) { // Looking for the $GPRMC string from GPS
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
    if (parts[2] == "A") {  // Check to see of the $GPRMC string is valid 
      String timeStrUTC = parts[1].substring(0, 2) + parts[1].substring(2, 4) + parts[1].substring(4, 6);
      // Change date display order, YYYYMMDD or MMDDYYYY.
      if (calendarConfiguration == 1){
        dateStr = "20" + parts[9].substring(4, 6) + parts[9].substring(2, 4) + parts[9].substring(0, 2);
      }
      else {
        dateStr =  parts[9].substring(2, 4) + parts[9].substring(0, 2) + "20" + parts[9].substring(4, 6);
      }
      int utcHours = timeStrUTC.substring(0, 2).toInt();
      int localHours = utcHours + timeZoneOffsetHours;
      int minutes = timeStrUTC.substring(2, 4).toInt() + timeZoneOffsetMinutes;
      int seconds = timeStrUTC.substring(4, 6).toInt();
      // Adjust hours, minutes, and seconds for overflow and underflow should it occur.
      if (minutes >= 60) {
        minutes -= 60;
        localHours += 1;
      }
      else if (minutes < 0) {
        minutes += 60;
        localHours -= 1;
      }
      //if (localHours >= 24) { // This does not appear to be used.
      //  localHours -= 24;
      //  adjustDate(dateStr, localHours, utcHours);
        //Serial.println(">=24");
      //}
      if (localHours < 0) { // Used between 190000 and 045959L standard time.
      //else if (localHours < 0) { // Used between 190000 and 045959L standard time.
        localHours += 24;
        adjustDate(dateStr, localHours, utcHours);
        //Serial.println("<0");
      }
      // Get the number of days since Jan 1 .
      int numberOfDays = getNumberOfJulianDays(dateStr);

      // Send all data to displays.
      displayDateAndTime(dateStr, localHours, minutes, seconds, numberOfDays);
    }
  }
}

int getNumberOfJulianDays(String date) {
  int year, month, day;
  
  if (calendarConfiguration == 1) {
    year = date.substring(0, 4).toInt();
    month = date.substring(4, 6).toInt();
    day = date.substring(6, 8).toInt();
  } else {
    year = date.substring(4, 8).toInt();
    month = date.substring(0, 2).toInt();
    day = date.substring(2, 4).toInt();
  }

  int daysCount = 0;
  
  // Sum the days of the months before the current month
  for (int cnt = 1; cnt < month; cnt++) {  // Changed "<=" to "<" for clarity
      daysCount += daysInMonth(cnt, year);
  }

  return daysCount + day; // Add days of previous months plus current day.
}

int daysInMonth(int month, int year) {
  if (month == 2 && ((year % 4 == 0 && year % 100 != 0) || (year % 400 == 0))) {
    return 29;  // Return leap day for February if leap year
  }
  
  int daysInMonth[] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
  return daysInMonth[month - 1]; 
}

//  Initialize the MAX7219 device's configuration on startup or otherwise. This is showing values 
//     for the rightmost then leftmost displays.
void initializeMax7219() {
  //           RIGHT-MOST..................................LEFT-MOST
  sendRegister(REG_SCAN_LIMIT, 0x05, REG_SCAN_LIMIT, 0x05, REG_SCAN_LIMIT, 0x07); // Set No. digits
  sendRegister(REG_DECODE, 0x00, REG_DECODE, 0x00, REG_DECODE, 0x00); // No display decoding
  sendRegister(REG_SHUTDOWN, 0x01, REG_SHUTDOWN, 0x01, REG_SHUTDOWN, 0x01); // Enable displays
  sendRegister(REG_DISP_TEST, 0x00, REG_DISP_TEST, 0x00, REG_DISP_TEST, 0x00); // Turn off test mode
  sendRegister(REG_INTENSITY, 0x00, REG_INTENSITY, 0x0a, REG_INTENSITY, 0x0b); // Set brightness 
  clearLEDRegisters(); 
}

// Function to write register addresses (opcodes) and the register's date.  
//
// When data is sent, is sent one character per display at a time; i.e., the rightmost display's 
//   rightmost character is sent, then one for the next display, and then one for the next.  This
//   repeats until eight characters for each display (24 total) are sent.  
// 
// Since the shiftOut command can only handle 8 bits at a time, the command is repeated 2 times the
//   number of displays. Every digit requires one address byte, and one data byte (16 bits).  So, 
//   if there are 16 digits , then 256 bits will be shifted every second, in 8 sets of 32 bits to 
//   complete the displayed data.
// 
// When data is sent to the MAX7219, the register address is sent first, followed by the address's 
//   data.  See page 6 of the MAX7219 datasheet. The last sentence of "Serial_Addressing Modes" 
//   shows that data bit D15 (MSB) is to be sent first. 
//
// D15 D14 D13 D12 D11 D10 D09 D08 D07 D06 D05 D04 D03 D02 D01 D00
//  X   X   X   X  |   ADDRESS   | |MSB  .  .  D A T A  . .   LSB|
//  X = Don't Care, 15 addressible registers. Eight are for digits, and 6 are for opcodes (addresses).
// 
void sendRegister(byte opcodeJDays, byte dataJDays, byte opcodeTime, byte dataTime, byte opcodeDate, byte dataDate) {
  digitalWrite(MAX7219_CS, LOW);
  // First out is the furthest module from the dataOut pin of Arduino.
  shiftOut(MAX7219_DIN, MAX7219_CLK, MSBFIRST, opcodeJDays); // opcode JDays
  shiftOut(MAX7219_DIN, MAX7219_CLK, MSBFIRST, dataJDays);   // data JDays
  shiftOut(MAX7219_DIN, MAX7219_CLK, MSBFIRST, opcodeTime);  // opcode Time
  shiftOut(MAX7219_DIN, MAX7219_CLK, MSBFIRST, dataTime);    // data Time
  shiftOut(MAX7219_DIN, MAX7219_CLK, MSBFIRST, opcodeDate);  // opcode Date
  shiftOut(MAX7219_DIN, MAX7219_CLK, MSBFIRST, dataDate);    // data Date
  digitalWrite(MAX7219_CS, HIGH);
}

// Reset all Module LED register data. LED registers are 1 thru 8.
void clearLEDRegisters() {
  for (int i = 0; i < 8; i++) {
    // Repeat "i + 1, 0x00" for each MAX7219.
    sendRegister(i + 1, 0x00, i + 1, 0x00, i + 1, 0x00); //Clear all segments
  }
}

// This function takes the current data (in segments) and sends it to the MAX7219 devices.  The 
//   function cuts up the strings into eight separate digits, determine their segment 
//   (charToSevenSegemnt) derivatives, and each digit's position (sevenSegmentPosition) then uses 
//   the sendRegister function to send the data to the display.
void displayDateAndTime(String date, int hours, int minutes, int seconds, int numberOfDays) {
  hours %= 24; // Ensure hours are within 0-23
  // If the local time is between 8PM and midnight, adjust the date -1 day. This accounts for UTC 
  //   -offset.].
  //if (hours >= 20 && hours < 24) {
  if (hours >= (24 + timeZoneOffsetHours) && hours < 24) {
    int year = 0;
    int month = 0;
    int day = 0;
    if (calendarConfiguration == 1){
      year = date.substring(0, 4).toInt();
      month = date.substring(4, 6).toInt();
      day = date.substring(6, 8).toInt();
    }
    else {
      // Added on 20240615 because display was incorrect after 8 PM if Cal Format was on.
      year = date.substring(4, 8).toInt();
      month = date.substring(0, 2).toInt();
      day = date.substring(2, 4).toInt();
    }
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

    // Assemble date string parts (year, month, day) into the current DATE for display; i.e, YYYYMMDD
    //   or MMDDYYYY based on the Cal. Config. switch.
    if (calendarConfiguration == 1){
      date = String(year) + String(month / 10) + String(month % 10) + String(day / 10) + String(day % 10);
    }
    else {
      date = String(month / 10) + String(month % 10) + String(day / 10) + String(day % 10) + String(year);
    }
  }

  // Format number of Julian days with preceding two digits for the year plus three 0s. The zeros 
  //   are not displayed since the SCAN LIMIT is set to 4. If Julian Date Config. switch is on then 
  //   don't display the year.
  // When the switch is OFF, the signal is pulled HIGH.
  if (julianDateConfiguration == 1){
      // julianDateConfiguration switch OFF... display only the 2 digit year and Julian day number.
      // Use Cal Config. switch to know where to get the year digits from in the DATE string.
    if (calendarConfiguration == 1){    
      //julianDays = "000" + date.substring(2, 4) + String(numberOfDays);// / 100) + String(numberOfDays % 100);
      if (numberOfDays < 10) {
          julianDays = "00" + date.substring(2, 4) + "00" + String(numberOfDays);
      } else if (numberOfDays < 100){
        julianDays = "00" + date.substring(2, 4) + "0" + String(numberOfDays);
      } else {
        julianDays = "00" + date.substring(2, 4) + String(numberOfDays);
      }
    }
    else {
      // julianDateConfiguration switch ON... display only the Julian day number.
      //julianDays = "000" + date.substring(6, 8) + String(numberOfDays);// / 100) + String(numberOfDays % 100);
      if (numberOfDays < 10) {
          julianDays = "00" + date.substring(6, 8) + "00" + String(numberOfDays);
      } else if (numberOfDays < 100){
        julianDays = "00" + date.substring(6, 8) + "0" + String(numberOfDays);
      } else {
        julianDays = "00" + date.substring(6, 8) + String(numberOfDays);
      }
    }
  }
  else {
    // Only DDD is displayed if the Julian Date switch is on.
    // julianDays = "000" + String(numberOfDays / 100) + String(numberOfDays % 100);
    if (numberOfDays < 10) {
        julianDays = "0000" + String(numberOfDays);
    } else if (numberOfDays < 100){
      julianDays = "000" + String(numberOfDays);
    } else {
      julianDays = "00" + String(numberOfDays);
    }
  }    

  // Two zeros are added to the TIME to right justify the data. These two digits are not seen because
  //   when the MAX7219 is initialized it is configured to only display six characters. See the 
  //   initializeMax7219 function (REG_SCAN_LIMIT) set to 0x05; i.e., digits 0 to 5, therefore a value 
  //   of 5 displays 6 digits.
  String timeStr = "00" + String(hours / 10) + String(hours % 10) + String(minutes / 10) + String(minutes % 10) + String(seconds / 10) + String(seconds % 10);
  //String timeStr = "00211632";
  // Check each character in the DATE, TIME, and DAYS strings and get their segment patterns and 
  //   positions then send them to the MAX7219's registers via the sendRegister function.
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

// Function to convert a string character to its corresponding 7-segment LED binary representation.
//   If a digit is not found, then return 0.
byte charToSevenSegment(char character) {
  if (character >= '0' && character <= '9') {
    return sevenSegmentDigits[character - '0'];
  }
  else {
    return 0;
  }
}
