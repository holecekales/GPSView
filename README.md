# GPSView
This was an experiment in trying to compare GPS and compass reading on [Arduino Nano](https://store.arduino.cc/arduino-nano). I was trying to solve several problems:

1. Read/parse the NEMA sentences from [GPS](https://www.adafruit.com/product/746) connected over Serial link
2. Display the readings on small [OLED screen](https://www.adafruit.com/product/326)
3. Connect Compass [HMC6352](https://www.sparkfun.com/products/retired/7915) and read those readings 
4. Compare the two in motion
5. Develop simple single button UX. I really like the simplicity of the button control and with the right HW, it worked remarkable well.   

# Usage
1. Install [Arduino IDE](https://www.arduino.cc/en/Main/Software) (1.8.2 or greater)
2. Install the libraries listed in the dependency section
3. Connect circut
4. Compile and upload to Arduino


# Dependencies
1. [SSD1306Ascii](https://github.com/greiman/SSD1306Ascii) is a library to display text display on small momochrome OLED modules. It also comes with SSD1306AsciiSoftSpi, to talk to the display. 
3. [Adafruit_GPS](https://github.com/adafruit/Adafruit_GPS) - interrupt drive GPS librabry by Adafruit
4. [TimeLib](https://github.com/PaulStoffregen/Time) - using to keep track of time, so I don't have to do it by hand
5. [SoftwareSerial](https://www.arduino.cc/en/Reference/SoftwareSerial) - using it to talk to the GPS. In something that is more *production*, this could be controlled by the HW serial after the chip gets programmed. 

# Notes
There are couple of other things, I really want to try:
1) There is a fork of SSD1306Ascii library that supports inverted text. I think it would really help with making the UI more readable and glancable on the small screen
2) Replace the Adafruit library with TinyGPS.The Adafruit library is really cool, but probably bigger than it needs to be. I am parsing only the very basic GPS senteces
3) The GPS comes with a logger. It would be fun to figure out how to use it
4) Figure more real-time and stable navigation algorithm  







