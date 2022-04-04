// ------------------------------------------
// ----------------- CONFIG -----------------
// ------------------------------------------

//BOARD- NodeMCU 1.0
//ntp description https://werner.rothschopf.net/202011_arduino_esp8266_ntp_en.htm
//time docs https://github.com/esp8266/Arduino/blob/master/cores/esp8266/time.cpp


const char* ssid     = "xx";
const char* password = "xxx";

byte latchPin = 5;
byte enablePin = 16;
byte mosiPin = 0;
byte sckPin = 4;
// SPI MOSI + SCK for data shift

// ------------------------------------------
// ------------------------------------------
// ------------------------------------------

#include <time.h>
#include <inttypes.h>

#include <ESP8266WiFiMulti.h>
#include <ESP8266WiFi.h>
#include "SPI.h"
#include "font-5x7.h"
#include "SoftwareSPI.h"


SoftSPI SSPI(mosiPin, sckPin);

unsigned int localPort = 2390;      // local port to listen for UDP packets
IPAddress timeServerIP;

const char* NTPServerName = "pool.ntp.org";

#define TIME_ZONE "CET-1CEST,M3.5.0/02,M10.5.0/03" // https://github.com/nayarsystems/posix_tz_db/blob/master/zones.csv


time_t timeUNIX = 0;
tm tm; 

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

inline int getSeconds() {
  return tm.tm_sec;
}

inline int getMinutes() {
  return tm.tm_min;
}

inline int getHours() {
  return tm.tm_hour;
}


void showTime() {
  Serial.print("year:");
  Serial.print(tm.tm_year + 1900);  // years since 1900
  Serial.print("\tmonth:");
  Serial.print(tm.tm_mon + 1);      // January = 0 (!)
  Serial.print("\tday:");
  Serial.print(tm.tm_mday);         // day of month
  Serial.print("\thour:");
  Serial.print(tm.tm_hour);         // hours since midnight  0-23
  Serial.print("\tmin:");
  Serial.print(tm.tm_min);          // minutes after the hour  0-59
  Serial.print("\tsec:");
  Serial.print(tm.tm_sec);          // seconds after the minute  0-61*
  Serial.print("\twday");
  Serial.print(tm.tm_wday);         // days since Sunday 0-6
  if (tm.tm_isdst == 1)             // Daylight Saving Time flag
    Serial.print("\tDST");
  else
    Serial.print("\tstandard");
  Serial.println();
}
void setup() {

  Serial.begin(115200);

  Serial.println("uruchomiono kaktus...");


  configTime(TIME_ZONE, NTPServerName); 
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


  time(&timeUNIX); 
  showTime();

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
  
  print_unix_timestamp(1547930250);
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


void print_unix_timestamp(time_t time) 
{
    localtime_r(&time, &tm); 
    print_hour(getHours(), getMinutes(), getSeconds());
}


unsigned long intervalNTP = 60000; // Request NTP time every minute
unsigned long prevNTP = 0;
unsigned long lastNTPResponse = millis();
unsigned long prevActualTime = 0;



void loop()
{
    unsigned long currentMillis = millis();
    
    if (currentMillis - prevNTP > intervalNTP) { // If a minute has passed since last NTP request
      prevNTP = currentMillis;
      Serial.println("Updating NTP time");
      Serial.flush();
      time(&timeUNIX); 
      localtime_r(&timeUNIX, &tm);           // update the structure tm with the current time
      showTime();
      lastNTPResponse = currentMillis;
      
    }
      
    if ((currentMillis - lastNTPResponse) > 3600000) 
    {
      Serial.println("More than 1 hour since last NTP response. Rebooting.");
      Serial.flush();
      ESP.reset();
    }


    time_t actualTime = timeUNIX + (currentMillis - lastNTPResponse)/1000;
    if (actualTime != prevActualTime && timeUNIX != 0) { // If a second has passed since last print
      prevActualTime = actualTime;
      localtime_r(&actualTime, &tm);  
      Serial.printf("\rUTC time:\t%02d:%02d:%02d   \r\n", getHours(), getMinutes(), getSeconds());
      print_unix_timestamp(actualTime);
    }

}
