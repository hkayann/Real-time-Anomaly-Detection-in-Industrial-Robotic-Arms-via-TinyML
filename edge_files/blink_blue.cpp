// Intialise library which communicates with RGB driver
// Functions accessible under 'nicla' namespace
#include "Nicla_System.h" 
#include "Arduino_BHY2.h"

void setup() {
  //run this code once when Nicla Sense ME board turns on
  nicla::begin();               // initialise library
  nicla::leds.begin();    
  BHY2.begin();      // Start I2C connection
}

void loop() {
  //run this code in a loop
  nicla::leds.setColor(blue);  //turn green LED on
  BHY2.update(10);
  delay(10);                  //wait 100 seconds
}