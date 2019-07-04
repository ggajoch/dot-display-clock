#include "SPI.h"
#include "font-5x7.h"

#include <ESP8266WiFiMulti.h>
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>

#include <time.h>
#include <inttypes.h>

const char* ssid     = "ssid";
const char* password = "pass";

byte latchPin = 5;
byte enablePin = 16;
byte mosiPin = 0;
byte sckPin = 4;
// SPI MOSI + SCK for data shift

#if not __AVR__

enum    _MONTHS_ { 
  JANUARY, FEBRUARY, MARCH, APRIL, 
  MAY, JUNE, JULY, AUGUST, 
  SEPTEMBER, OCTOBER, NOVEMBER, DECEMBER 
};

enum    _WEEK_DAYS_ { 
  SUNDAY, MONDAY, TUESDAY, WEDNESDAY, 
  THURSDAY, FRIDAY, SATURDAY 
};

#define   ONE_HOUR   3600

#endif 


int eu_dst(const time_t * timer) {
    struct tm       tmptr;
    uint8_t         month, mday, hour, day_of_week, d;
    int             n;

    /* obtain the variables */
                    gmtime_r(timer, &tmptr);
                    month = tmptr.tm_mon;
                    day_of_week = tmptr.tm_wday;
                    mday = tmptr.tm_mday - 1;
                    hour = tmptr.tm_hour;

    if              ((month > MARCH) && (month < OCTOBER))
                        return ONE_HOUR;

    if              (month < MARCH)
                        return 0;
    if              (month > OCTOBER)
                        return 0;

    /* determine mday of last Sunday */
                    n = tmptr.tm_mday - 1;
                    n -= day_of_week;
                    n += 7;
                    d = n % 7;  /* date of first Sunday */

                    n = 31 - d;
                    n /= 7; /* number of Sundays left in the month */

                    d = d + 7 * n;  /* mday of final Sunday */

    if              (month == MARCH) {
        if (d < mday)
            return 0;
        if (d > mday)
            return ONE_HOUR;
        if (hour < 1)
            return 0;
        return ONE_HOUR;
    }
    if              (d < mday)
                        return ONE_HOUR;
    if              (d > mday)
                        return 0;
    if              (hour < 1)
                        return ONE_HOUR;
                    return 0;

}

#include "SoftwareSPI.h"
SoftSPI SSPI(mosiPin, sckPin);

//#define SSPI SPI

unsigned int localPort = 2390;      // local port to listen for UDP packets


IPAddress timeServerIP;


const int NTP_PACKET_SIZE = 48; // NTP time stamp is in the first 48 bytes of the message

byte packetBuffer[NTP_PACKET_SIZE]; //buffer to hold incoming and outgoing packets

// A UDP instance to let us send and receive packets over UDP
WiFiUDP UDP;

constexpr uint8_t COLUMNS = 28;
constexpr uint8_t ROWS = 7;

uint8_t columns_to_address[COLUMNS] = {
  0b01100,
  0b01110,
  0b01011,
  0b01010,
  0b01001,
  0b01101,
  0b01111,
  0b10100,
  0b10011,
  0b10110,
  0b10010,
  0b10001,
  0b10101,
  0b10111,
  0b11111,
  0b11101,
  0b11110,
  0b11100,
  0b11011,
  0b11010,
  0b11001,
  0b00111,
  0b00101,
  0b00110,
  0b00100,
  0b00011,
  0b00010,
  0b00001,
};

uint8_t rows_set_to_address[ROWS] = {
  0b01111,
  0b01110,
  0b01101,
  0b01100,
  0b01011,
  0b01010,
  0b01001
};

uint8_t rows_clear_to_address[ROWS] = {
  0b10111,
  0b10110,
  0b10101,
  0b10100,
  0b10011,
  0b10010,
  0b10001
};







const char* NTPServerName = "pool.ntp.org";

//const int NTP_PACKET_SIZE = 48;  // NTP time stamp is in the first 48 bytes of the message

byte NTPBuffer[NTP_PACKET_SIZE]; // buffer to hold incoming and outgoing packets








uint32_t getTime() {
  if (UDP.parsePacket() == 0) { // If there's no response (yet)
    return 0;
  }
  UDP.read(NTPBuffer, NTP_PACKET_SIZE); // read the packet into the buffer
  // Combine the 4 timestamp bytes into one 32-bit number
  uint32_t NTPTime = (NTPBuffer[40] << 24) | (NTPBuffer[41] << 16) | (NTPBuffer[42] << 8) | NTPBuffer[43];
  // Convert NTP time to a UNIX timestamp:
  // Unix time starts on Jan 1 1970. That's 2208988800 seconds in NTP time:
  const uint32_t seventyYears = 2208988800UL;
  // subtract seventy years:
  uint32_t UNIXTime = NTPTime - seventyYears;
  return UNIXTime;
}

void sendNTPpacket(IPAddress& address) {
  memset(NTPBuffer, 0, NTP_PACKET_SIZE);  // set all bytes in the buffer to 0
  // Initialize values needed to form NTP request
  NTPBuffer[0] = 0b11100011;   // LI, Version, Mode
  // send a packet requesting a timestamp:
  UDP.beginPacket(address, 123); // NTP requests are to port 123
  UDP.write(NTPBuffer, NTP_PACKET_SIZE);
  UDP.endPacket();
}

inline int getSeconds(uint32_t UNIXTime) {
  return UNIXTime % 60;
}

inline int getMinutes(uint32_t UNIXTime) {
  return UNIXTime / 60 % 60;
}

inline int getHours(uint32_t UNIXTime) {
  return UNIXTime / 3600 % 24;
}

void startUDP() {
  Serial.println("Starting UDP");
  UDP.begin(123);                          // Start listening for UDP messages on port 123
  Serial.print("Local port:\t");
  Serial.println(UDP.localPort());
  Serial.println();
}

void setup() {

  Serial.begin(115200);

  Serial.println("uruchomiono kaktus...");
  
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi polaczono kaktus");
  Serial.print("IP : ");
  Serial.println(WiFi.localIP());


  startUDP();

  if(!WiFi.hostByName(NTPServerName, timeServerIP)) { // Get the IP address of the NTP server
    Serial.println("DNS lookup failed. Rebooting.");
    Serial.flush();
    ESP.reset();
  }
  Serial.print("Time server IP:\t");
  Serial.println(timeServerIP);


  pinMode(enablePin, OUTPUT);
  digitalWrite(enablePin, HIGH);

  pinMode(latchPin, OUTPUT);

  SSPI.begin();
  SSPI.setClockDivider(SPI_CLOCK_DIV8);


  for(int r = 0; r < ROWS; ++r) {
    for(int c = 0; c < COLUMNS; ++c) {
      set_pixel(r, c);
    }
  }

  delay(1000);

  for(int r = 0; r < ROWS; ++r) {
    for(int c = 0; c < COLUMNS; ++c) {
      clear_pixel(r, c);
    }
  }

  delay(1000);
  
  print_unix_timestamp_utc(1547930250);
}

void send_bytes(uint8_t byteRows, uint8_t byteColumns) {
  byteRows = ~byteRows;
  byteColumns = ~byteColumns;

  //SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE3));
  SSPI.transfer(byteColumns);
  SSPI.transfer(byteRows);
  //SPI.endTransaction();

  delayMicroseconds(100);
  digitalWrite(latchPin, HIGH);
  delayMicroseconds(100);
  digitalWrite(latchPin, LOW);
  delayMicroseconds(100);

  digitalWrite(enablePin, LOW);
  delayMicroseconds(2000);
  digitalWrite(enablePin, HIGH);
}

void clear_pixel(int row, int column) {
  uint8_t byte_row = (0 << 6) + rows_clear_to_address[row];
  uint8_t byte_column = (1 << 6) + columns_to_address[column];

  send_bytes(byte_row, byte_column);
}

void set_pixel(int row, int column) {
  uint8_t byte_row = (1 << 6) + rows_set_to_address[row];
  uint8_t byte_column = (0 << 6) + columns_to_address[column];

  send_bytes(byte_row, byte_column);
}

void write_pixel(int row, int column, bool value) {
  if (row >= ROWS || column >= COLUMNS || column < 0) {
    return;
  }
  if (value) {
    set_pixel(row, column);
  } else {
    clear_pixel(row, column);
  }
}

void print_char(char character, int start_column) {
  for(int r = 0; r < ROWS; ++r) {
      for(int c = 0; c < 5; ++c) {
        int offset = 5*(character - ' ');

        write_pixel(r, start_column + c, Font5x7[offset+c] & (1 << r));
      }
      write_pixel(r, start_column + 5, 0);
    }
}


void print_char_4_column(char character, int start_column) {
  for(int r = 0; r < ROWS; ++r) {
      for(int c = 0; c < 4; ++c) {
        int offset = 5*(character - ' ');

        write_pixel(r, start_column + c, Font5x7[offset+c] & (1 << r));
      }
      write_pixel(r, start_column + 4, 0);
    }
}

void print_hour(int h, int m, int s)
{
  char h_str[3];
  char m_str[3];

  sprintf(h_str,"%02d",h);
  sprintf(m_str,"%02d", m);

  if(s%2)
  {
    print_char_4_column(':', 12);
  }
  else
  {
    print_char_4_column(' ', 12);
  }

  print_char(h_str[0], 0);
  print_char(h_str[1], 6);
  print_char(m_str[0], 17);
  print_char(m_str[1], 23);
}


void print_unix_timestamp(const long int time) {
    print_hour(getHours(time), getMinutes(time), getSeconds(time));
}


void print_unix_timestamp_utc(long int time) {
    auto diff = eu_dst(&time);
    time += diff;
    time += ONE_HOUR;
    print_hour(getHours(time), getMinutes(time), getSeconds(time));
}




unsigned long intervalNTP = 60000; // Request NTP time every minute
unsigned long prevNTP = 0;
unsigned long lastNTPResponse = millis();
uint32_t timeUNIX = 0;

unsigned long prevActualTime = 0;


byte initialized = 0;

void loop()
{
    unsigned long currentMillis = millis();

    if (currentMillis - prevNTP > intervalNTP || initialized == 0) { // If a minute has passed since last NTP request
      prevNTP = currentMillis;
      Serial.println("\r\nSending NTP request ...");
      sendNTPpacket(timeServerIP);               // Send an NTP request
      initialized = 1;
    }

    uint32_t time = getTime();                   // Check if an NTP response has arrived and get the (UNIX) time
    if (time) {                                  // If a new timestamp has been received
      timeUNIX = time;
      Serial.print("NTP response:\t");
      Serial.println(timeUNIX);
      lastNTPResponse = currentMillis;
    } else if ((currentMillis - lastNTPResponse) > 3600000) {
      Serial.println("More than 1 hour since last NTP response. Rebooting.");
      Serial.flush();
      ESP.reset();
    }


    uint32_t actualTime = timeUNIX + (currentMillis - lastNTPResponse)/1000;
    if (actualTime != prevActualTime && timeUNIX != 0) { // If a second has passed since last print
      prevActualTime = actualTime;
      Serial.printf("\rUTC time:\t%02d:%02d:%02d   \r\n", getHours(actualTime), getMinutes(actualTime), getSeconds(actualTime));
      print_unix_timestamp_utc(actualTime);
    }

}
