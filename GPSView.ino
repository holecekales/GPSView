// Test for minimum program size.

// SPI Screen pin out
#define OLED_DATA   9
#define OLED_CLK   10
#define OLED_DC    11
#define OLED_CS    12
#define OLED_RESET 13

// GPS Connection
#define GPS_RX 3
#define GPS_TX 2
#define GPS_BAUD 9600

// User interface
#define BUTTON_PIN 4  // Button

#include <SPI.h>

#include "SSD1306Ascii.h"
#include "SSD1306AsciiSoftSpi.h"
#include "Adafruit_GPS.h"

// #include "TinyGPS.h"
// #include "NewSoftSerial.h"
// TinyGPS gps;
// NewSoftSerial  ss(GPS_RX, GPS_TX);

#include "SoftwareSerial.h"

SSD1306AsciiSoftSpi oled;

SoftwareSerial ss(GPS_RX, GPS_TX);
Adafruit_GPS GPS(&ss);

// Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
// Set to 'true' if you want to debug and listen to the raw GPS sentences. 
#define GPSECHO  false

// this keeps track of whether we're using the interrupt
// off by default!
boolean usingInterrupt = false;
uint32_t timer = millis();

#define LONGPRESS_LEN    500  // millis that make up a long press (1s)
#define DEBOUNCE 10
enum { EV_NONE=0, EV_SHORTPRESS, EV_LONGPRESS};

boolean button_was_pressed = false; // previous state
uint32_t button_pressed_timer = millis(); // press running duration


//------------------------------------------------------------------------------
void setup() {
 
  Serial.begin(115200);
  GPS.begin(9600);
  // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  // uncomment this line to turn on only the "minimum recommended" data
  //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  // For parsing data, we don't suggest using anything but either RMC only or RMC+GGA since
  // the parser doesn't care about other sentences at this time
  
  // Set the update rate
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rate
  // For the parsing code to work nicely and have time to sort thru the data, and
  // print it out we don't suggest using anything higher than 1 Hz

  // Request updates on antenna status, comment out to keep quiet
  GPS.sendCommand(PGCMD_ANTENNA);

  // the nice thing about this code is you can have a timer0 interrupt go off
  // every 1 millisecond, and read data from the GPS for you. that makes the
  // loop code a heck of a lot easier!
  useInterrupt(true);
 
  // set up display                
  oled.begin(&Adafruit128x64, OLED_CS, OLED_DC, OLED_CLK, OLED_DATA, OLED_RESET);
  oled.setFont(Stang5x7);
  // oled.set2X();
  oled.clear();

  pinMode(BUTTON_PIN, INPUT);
  digitalWrite(BUTTON_PIN, HIGH); // pull-up

  // oled.print("Hello world!");
}
//------------------------------------------------------------------------------

void loop() {
   handleGPS();
   handleButton();
}

//------------------------------------------------------------------------------
// Interrupt is called once a millisecond, looks for any new GPS data, and stores it

SIGNAL(TIMER0_COMPA_vect) {
  char c = GPS.read();
  // if you want to debug, this is a good time to do it!
#ifdef UDR0
  if (GPSECHO)
    if (c) UDR0 = c;  
    // writing direct to UDR0 is much much faster than Serial.print 
    // but only one character can be written at a time. 
#else
  Serial.print(c);
#endif
}

//------------------------------------------------------------------------------
void useInterrupt(boolean v) {
  if (v) {
    // Timer0 is already used for millis() - we'll just interrupt somewhere
    // in the middle and call the "Compare A" function above
    OCR0A = 0xAF;
    TIMSK0 |= _BV(OCIE0A);
    usingInterrupt = true;
  } else {
    // do not call the interrupt function COMPA anymore
    TIMSK0 &= ~_BV(OCIE0A);
    usingInterrupt = false;
  }
}

//------------------------------------------------------------------------------
void displayTimefromGPS(uint8_t col, uint8_t row) {
  char c[15];
  sprintf(c,"%02d:%02d %02d\/%02d\/%02d", GPS.hour,GPS.minute,GPS.month,GPS.day,GPS.year);// build integer string using C integer formatters  
  oled.setCursor(col, row);
  oled.print("Time: "); oled.print(c);
}

//------------------------------------------------------------------------------
void displayKeyValueInt(const char* name, int value, uint8_t row = -1, uint8_t col = 0) {
  if(row < 8)
    oled.setCursor(col, row);
    oled.print(name); 
    if(value < 10) 
      oled.print(0);
    oled.print(value);
}

//------------------------------------------------------------------------------
void displayLongLat() {
    oled.set1X();
    displayKeyValueInt("Sat:", GPS.satellites, 0, 0);
    displayKeyValueInt(" Fix:", GPS.fixquality,0, 86);
    
    displayTimefromGPS(3,7);
    if (GPS.fix) {

      oled.setCursor(0, 2);
      oled.set2X();

      char f[11];
      oled.println(dtostrf(GPS.latitudeDegrees, 10,5,f));
      oled.println(dtostrf(GPS.longitudeDegrees, 10,5,f));

      /*
      Serial.print("Speed (knots): "); Serial.println(GPS.speed);
      Serial.print("Angle: "); Serial.println(GPS.angle);
      Serial.print("Altitude: "); Serial.println(GPS.altitude);
        */
    }
}


//------------------------------------------------------------------------------
void handleGPS() {

 // in case you are not using the interrupt above, you'll
  // need to 'hand query' the GPS, not suggested :(
  if (! usingInterrupt) {
    // read data from the GPS in the 'main loop'
    char c = GPS.read();
    // if you want to debug, this is a good time to do it!
    if (GPSECHO)
      if (c) Serial.print(c);
  }
  
  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) {
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences! 
    // so be very wary if using OUTPUT_ALLDATA and trytng to print out data
    //Serial.println(GPS.lastNMEA());   // this also sets the newNMEAreceived() flag to false
  
    if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
      return;  // we can fail to parse a sentence in which case we should just wait for another
  }

  // if millis() or timer wraps around, we'll just reset it
  if (timer > millis())  timer = millis();

  // approximately every 2 seconds or so, print out the current stats
  if (millis() - timer > 500) { 
    timer = millis(); // reset the timer
    displayLongLat();  
  }
}

//------------------------------------------------------------------------------
int raiseButtonEvent() {
  int event = EV_NONE;
  int button_now_pressed = !digitalRead(BUTTON_PIN);  // pin low -> pressed
  delay(DEBOUNCE); // $$$ fix this! this is BLOCKING
  int debounceVal = !digitalRead(BUTTON_PIN);         // pin low -> pressed

  if(debounceVal == button_now_pressed) {
    // The button just got released. Now we need to decide if it was short 
    // or long press and generate the right event.
    if (!button_now_pressed && button_was_pressed) {
      if (millis() - button_pressed_timer < LONGPRESS_LEN)
        event = EV_SHORTPRESS;
      else
        event = EV_LONGPRESS;
    }

    if (button_now_pressed && !button_was_pressed)
      button_pressed_timer = millis();
    
    button_was_pressed = button_now_pressed;
  }
  return event;
}

//------------------------------------------------------------------------------
void handleButton() {
  // handle button
  int event = raiseButtonEvent();

  // do other things
  switch (event) {
    case EV_NONE:
    break;
    case EV_SHORTPRESS:
      Serial.println("S");
    break;
    case EV_LONGPRESS:
      Serial.println("L");
    break;
  }
}
