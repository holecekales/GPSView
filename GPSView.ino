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
#include <Wire.h>

#include "SSD1306Ascii.h"
#include "SSD1306AsciiSoftSpi.h"
#include "Adafruit_GPS.h"

#include "SoftwareSerial.h"
#include "TimeLib.h"

// Offset hours from gps time (UTC)
//#define TIME_ZONE   1*SECS_PER_HOUR  // Central European Time
#define TIME_ZONE  -7*SECS_PER_HOUR  // Pacific Delight Time  (USA)
//#define TIME_ZONE  -8*SECS_PER_HOUR  // Pacific Standart Time (USA)

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
#define SCREEN_UPDATE    500   // update screen interval
// User interface input globals

#define LONGPRESS_LEN    500  // millis that make up a long press or repeat
#define DEBOUNCE 10
enum { EV_NONE=0, EV_SHORTPRESS, EV_LONGPRESS, EV_LONGHOLD, EV_LONGREP};
boolean button_was_pressed = false; // previous state
uint32_t button_longhold   = 0;
uint32_t button_pressed_timer = 0; // press running duration

enum { DM_LATLONG=0, DM_MOTION, DM_WAYPOINT, DM_NAVI, DM_COMPASS, DM_MAX};
char* tabName[] = {"Lat/Long", "Motion", "Waypoint", "Navigate", "Compass" };

uint8_t displayMode      = 0;
uint8_t prevDisplayMode  = -1;
boolean prevGPSFIX = false;

// formating print buffer
char printBuffer[25];

// Navigation data
float WP_LAT = 46.626636;       // (sample point)
float WP_LON = -124.060092;
uint8_t WP_OK = 1;
boolean WP_Capture = false;

// conversions Motion tab
#define MPH_PER_KNOT 1.15077945       // miles per hour
#define MPS_PER_KNOT 0.51444444       // meters per second
#define KMPH_PER_KNOT 1.852           // kilometer per hour
#define MILES_PER_METER 0.00062137112 
enum {U_KN=0, U_MPS, U_KMPH, U_LAST};
uint8_t spdConverstion = U_MPS;

//------------------------------------------------------------------------------
class MedianOf3 {
  int16_t newest, recent, oldest; 
public:
  MedianOf3(int16_t initVal = 0) : newest(initVal), recent(initVal), oldest(initVal) { }

  void push(int16_t val) {
    oldest = recent;
    recent = newest;
    newest = val;
  }
  
  int16_t get() const {
    int16_t the_max = max( max( oldest, recent ), newest );
    int16_t the_min = min( min( oldest, recent ), newest );
    // unnecessarily clever code
    int16_t the_median = the_max ^ the_min ^ oldest ^ recent ^ newest;
    return( the_median );
  }
};
//------------------------------------------------------------------------------
// Compass definitions
#define HMC6352Address 0x42
#define HMC6352SlaveAddress (HMC6352Address >> 1)
byte compassHeading[2];
enum {COMPASS_MEASSURE = 0, COMPASS_CALIBRATE_START, COMPASS_CALIBRATING, COMPASS_CALIBRATE_END, COMPASS_LAST_STATE};
uint8_t compassMode = COMPASS_MEASSURE; 
MedianOf3 mgDir;
// exists for two reasons: 90deg = because of mounting on the board 
// and declination because of position on earth 
#define MAG_CORRECTION 90 

//------------------------------------------------------------------------------
void setup() {
 
  Serial.begin(115200);
  GPS.begin(9600);
  Wire.begin();
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
  oled.setFont(System5x7);
  oled.clear();

  pinMode(BUTTON_PIN, INPUT);
  digitalWrite(BUTTON_PIN, HIGH); // pull-up
}

//------------------------------------------------------------------------------
void loop() {
   handleGPS();
   handleCompassReq();
   handleButton();  // stalls for 10ms :(
   handleCompassRead();  
   updateDisplay();
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
inline void invalScreen() {
  prevDisplayMode = -1;
}

//------------------------------------------------------------------------------
void displayTime(uint8_t row) {
  sprintf(printBuffer,"%02d:%02d.%02d   %02d\/%02d\/%02d", hour(),minute(),second(), month(),day(),year());// build integer string using C integer formatters 
  oled.setCursor(0, row);
  oled.print(printBuffer);
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
void displayContext(const char* screenName, boolean showTime) {
  oled.set1X();
  displayKeyValueInt("Sat:", GPS.satellites, 0, 0);
  if(WP_OK>0) {
    displayKeyValueInt("Fix:", /*GPS.fixquality*/ WP_OK ,0, 92);
  } else {
    oled.setCol(92);
    oled.print("Fix:NO");
  }
  
  if(screenName) {
    // pixel (column) 40 is a good begining
    // i can accomomdate at most 8 cahracters
    // calculate offset from position 40 toward the center of the screen
    // based on the length of the string
    // each char is 6 wide (right now), but i need to split it between 
    // begining and end so i will multiple by 6/2 == 3
    uint8_t off = (8-strlen(screenName)) * 3; 
    oled.setCol(40+off);
    oled.print(screenName);
  }

  if(showTime)
    displayTime(7);  
}


//------------------------------------------------------------------------------
void printBigLine(uint8_t row, uint8_t off, const char* name, const char* value, const char* unit) {
  oled.setCursor(0, row);
  oled.set2X();
  oled.print(name); 
  oled.setCol(oled.col() + off);
  oled.print(value);
  oled.set1X();
  oled.print(unit);
}


//------------------------------------------------------------------------------
void displayLongLat(float lat, float lon) {
    printBigLine(2, 0, "", dtostrf(lat, 10,5,printBuffer), "o");
    printBigLine(4, 0, "", dtostrf(lon, 10,5,printBuffer), "o");
}

//------------------------------------------------------------------------------
void displayMotion() {
  char* u = "";
  float v = 0;
  switch(spdConverstion) {
    case U_KN:   v = GPS.speed;                   u = "kn  ";   break;
    case U_MPS:  v = GPS.speed * MPS_PER_KNOT;    u = "m/s ";  break;
    case U_KMPH: v = GPS.speed * KMPH_PER_KNOT;   u = "km/h"; break;
  }
  printBigLine(2, 7, "Spd", dtostrf(v,5,1,printBuffer), u);
  printBigLine(4, 7, "Ang", dtostrf(GPS.angle,5,1,printBuffer), "o");
  printBigLine(6, 7, "Alt", dtostrf(GPS.altitude,5,1,printBuffer), "m");
}

//------------------------------------------------------------------------------
void displayNavigation() {
  if(WP_OK == 0) {
    oled.setCursor(12, 3);
    oled.set2X();
    oled.print("Waypoint?"); 
    return;
  }
  
  float dist = distance_between(GPS.latitudeDegrees, GPS.longitudeDegrees, WP_LAT, WP_LON);
  char* u = "m";
  if(dist >= 1000) {
    u = "km";
    dist /= 1000.f;
  }
  float crs = course_to(GPS.latitudeDegrees, GPS.longitudeDegrees, WP_LAT, WP_LON);
  printBigLine(2, 7, "Dst", dtostrf(dist,6,1,printBuffer), u);
  printBigLine(4, 7, "DOT", dtostrf(crs,6,1,printBuffer), "o");
  printBigLine(6, 7, "Div", dtostrf(crs-GPS.angle,6,1,printBuffer), "o");
  // printBigLine(6, 17, "Dir", cardinal(crs), " ");
}

//------------------------------------------------------------------------------
void displayCompass() {
  if(compassMode == COMPASS_MEASSURE) {
    // int headingValue = compassHeading[0]*256 + compassHeading[1]; // Put the MSB and LSB together

    // becaause of mounting on breadboard, i need to add 90 degrees.
    uint16_t headingValue = compassHeading[0]*256 + compassHeading[1]; // Put the MSB and LSB together
    mgDir.push(headingValue);

    uint16_t hv = int(mgDir.get() / 10) + MAG_CORRECTION;
    if(hv >= 360) hv -= 360;   // correct for overflow
    sprintf(printBuffer,"%03d", hv);
    // sprintf(printBuffer,"%03d.%d", hv,int(headingValue % 10));
    printBigLine(2, 18, "Hdg", printBuffer, "o");
    float crs = course_to(GPS.latitudeDegrees, GPS.longitudeDegrees, WP_LAT, WP_LON);
    printBigLine(4, 7, "Ang", dtostrf(/*crs-*/GPS.angle,6,1,printBuffer), "o");
    printBigLine(6, 7, "DOT", dtostrf(crs,6,1,printBuffer), "o");
    
  }
  else {
    printBigLine(4, 8, "", "CALIBRATE", "");
  }
}

//------------------------------------------------------------------------------
void handleCompassReq() {
  char* c = NULL;
  
  switch(compassMode) {
    case COMPASS_MEASSURE:        c = "A"; break; // The "Get Data" command
    // send start calibration command and move to the CALIBRATING state
    case COMPASS_CALIBRATE_START: c = "C"; compassMode = COMPASS_CALIBRATING; break; 
    // send calibration EXIT and go back to meassuring  
    case COMPASS_CALIBRATE_END:   c = "E"; compassMode = COMPASS_MEASSURE;  invalScreen();  break; 
  }
  
  if(c != NULL) {
    Wire.beginTransmission(HMC6352SlaveAddress);
    Wire.write(c); // send command
    Wire.endTransmission(); 
  } 
}

// The HMC6352 needs at least a 70us (microsecond) delay after the REQ command
// Using 10ms is docunebted safe and we already have 10ms delay due to the button
// debounce. So we will just split the req and read and put it around the button
// handler. At some point we should remove all of this to make the loop run as
// fast as possible
// Read the 2 heading bytes, MSB first
// The resulting 16bit word is the compass heading in 10th's of a degree
// For example: a heading of 1345 would be 134.5 degrees
void handleCompassRead() {
  if(compassMode == COMPASS_MEASSURE) {
    Wire.requestFrom(HMC6352SlaveAddress, 2);// Request the 2 byte heading (MSB comes first)
    int i = 0;
    while(Wire.available() && i < 2) {
      compassHeading[i] = Wire.read();
      i++;
    } 
  }
}

//------------------------------------------------------------------------------
void updateDisplay() {
  // if millis() or timer wraps around, we'll just reset it
  if (timer > millis())  timer = millis();

  // redraw if page changed  or if we just aquiring fix to having fix
  boolean cls = (prevDisplayMode != displayMode) || (prevGPSFIX != GPS.fix); // clear screen
 
  if(cls) {
    oled.clear();
    prevDisplayMode = displayMode;
  }

  if(cls || ((millis() - timer) > SCREEN_UPDATE)) {
    timer = millis(); // reset the timer

    if(!GPS.fix) {
      displayContext(tabName[displayMode], false);
      // if we don't have a fix, we will just display that we're looking for 
      // satellites. Time should be always good on the GPS (as long as the battery is fine)
      printBigLine(2, 10, "", "ACQUIRING", "");
      printBigLine(4, 4, "", "SATELLITES",  "");
    } 
    else {

      if(!prevGPSFIX) {
        // set the time - since we now have a fix
        setTime(GPS.hour, GPS.minute, GPS.seconds, GPS.day, GPS.month, GPS.year);        
        adjustTime(TIME_ZONE);      
      }

      switch(displayMode) {
        case DM_LATLONG:
          displayContext(tabName[displayMode], true);
          displayLongLat(GPS.latitudeDegrees, GPS.longitudeDegrees);  
        break;
        case DM_MOTION:
          displayContext(tabName[displayMode], false);
          displayMotion();      
        break;
        case DM_WAYPOINT:
          displayContext(tabName[displayMode], true);
          if(WP_OK > 0)
            displayLongLat(WP_LAT, WP_LON);
        break;
        case DM_NAVI:
            displayContext(tabName[displayMode], WP_OK == 0);
            displayNavigation();
        break;
        case DM_COMPASS:
            displayContext(tabName[displayMode], false);
            displayCompass();
        break;
        default:
          displayContext(tabName[displayMode], true);
        break;
      }
    }
  }
  prevGPSFIX = GPS.fix;
}

//------------------------------------------------------------------------------
void handleGPS() {
  // in case you are not using the interrupt above, you'll
  // need to 'hand query' the GPS, not suggested :(
  if (!usingInterrupt) {
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
}

//------------------------------------------------------------------------------
int raiseButtonEvent() {
  int event = EV_NONE;
  int button_now_pressed = !digitalRead(BUTTON_PIN);  // pin low -> pressed
  delay(DEBOUNCE); // $$$ fix this! this is BLOCKING
  int debounceVal = !digitalRead(BUTTON_PIN);         // pin low -> pressed

  if(debounceVal == button_now_pressed) { // debounced

    // the butotn is being held down
    if(button_now_pressed && button_was_pressed) {
     uint32_t ms = millis();  
     if((ms - button_pressed_timer) > LONGPRESS_LEN)  {
       if(button_longhold == 0) {
        // long press event - this is the first 
        button_longhold++;
        event = EV_LONGHOLD;
       }
       else if((ms - button_pressed_timer) > button_longhold*LONGPRESS_LEN) {
        // long press repeats issues if someone continues to hold the button 
        button_longhold++;
        event = EV_LONGREP;
       }
      }
    }

    // The button just got released. Now we need to decide if it was short 
    // or long press and generate the right event.
    if(!button_now_pressed && button_was_pressed) {
      if (millis() - button_pressed_timer < LONGPRESS_LEN)
        event = EV_SHORTPRESS;
      else
        event = EV_LONGPRESS;
    }

    if(button_now_pressed && !button_was_pressed) {
      button_longhold = 0; // arm the longhold event
      button_pressed_timer = millis();
    }
    
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
      displayMode = (displayMode + 1) % DM_MAX;
      if(compassMode == COMPASS_CALIBRATING)
        compassMode = COMPASS_CALIBRATE_END;
    break;
    case EV_LONGPRESS:
    break;
    case EV_LONGHOLD:
      if(displayMode==DM_LATLONG) {
        // in LAT/LONG mode a capture way point
        WP_Capture = true;
      } 
      else if(displayMode == DM_MOTION) {
        spdConverstion = (spdConverstion + 1) % U_LAST;
      }
      else if(displayMode==DM_WAYPOINT) {
        // remove waypoint - no need, just for fun :)
        WP_OK  = 0;
        WP_LAT = 0;
        WP_LON = 0;
        invalScreen();
      } else if(displayMode==DM_COMPASS) {
        // the transitions to the other modes happen in 
        // updateCompass() and handleCompass() 
        switch(compassMode) {
          case COMPASS_MEASSURE:
            compassMode = COMPASS_CALIBRATE_START;
            invalScreen(); 
          break;
          case COMPASS_CALIBRATING:
            compassMode = COMPASS_CALIBRATE_END;
          break;
          default:
            compassMode = COMPASS_MEASSURE; 
          break;
        }
      }
    break;
    case EV_LONGREP:
      if(displayMode == DM_MOTION) {
        spdConverstion = (spdConverstion + 1) % U_LAST;
        invalScreen();
      }
    break;
  }

  // capture new waypoint waypoint
  if(GPS.fix && WP_Capture) {
    WP_Capture = false;
    WP_OK += 1;
    WP_LAT = GPS.latitudeDegrees;
    WP_LON = GPS.longitudeDegrees;
  }
}

//------------------------------------------------------------------------------
float distance_between (float lat1, float long1, float lat2, float long2) {
  // returns distance in meters between two positions, both specified 
  // as signed decimal-degrees latitude and longitude. Uses great-circle 
  // distance computation for hypothetical sphere of radius 6372795 meters.
  // Because Earth is no exact sphere, rounding errors may be up to 0.5%.
  // Courtesy of Maarten Lamers
  float delta = radians(long1-long2);
  float sdlong = sin(delta);
  float cdlong = cos(delta);
  lat1 = radians(lat1);
  lat2 = radians(lat2);
  float slat1 = sin(lat1);
  float clat1 = cos(lat1);
  float slat2 = sin(lat2);
  float clat2 = cos(lat2);
  delta = (clat1 * slat2) - (slat1 * clat2 * cdlong); 
  delta = sq(delta); 
  delta += sq(clat2 * sdlong); 
  delta = sqrt(delta); 
  float denom = (slat1 * slat2) + (clat1 * clat2 * cdlong); 
  delta = atan2(delta, denom); 
  return delta * 6372795; 
}


//------------------------------------------------------------------------------
float course_to (float lat1, float long1, float lat2, float long2) {
  // returns course in degrees (North=0, West=270) from position 1 to position 2,
  // both specified as signed decimal-degrees latitude and longitude.
  // Because Earth is no exact sphere, calculated course may be off by a tiny fraction.
  // Courtesy of Maarten Lamers
  float dlon = radians(long2-long1);
  lat1 = radians(lat1);
  lat2 = radians(lat2);
  float a1 = sin(dlon) * cos(lat2);
  float a2 = sin(lat1) * cos(lat2) * cos(dlon);
  a2 = cos(lat1) * sin(lat2) - a2;
  a2 = atan2(a1, a2);
  if (a2 < 0.0) {
    a2 += TWO_PI;
  }
  return degrees(a2);
}

//------------------------------------------------------------------------------
const char* cardinal (float course) {
  static const char* directions[] = {"N  ", "NNE", "NE ", "ENE", "E  ", "ESE", "SE ", "SSE", "S  ", "SSW", "SW ", "WSW", "W  ", "WNW", "NW ", "NNW"};

  int direction = (int)((course + 11.25f) / 22.5f);
  return directions[direction % 16];
}



