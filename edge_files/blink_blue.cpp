// Intialise library which communicates with RGB driver
// Functions accessible under 'nicla' namespace
#include "Nicla_System.h" 
#include "Arduino_BHY2.h"

void setup() {
  //run this code once when Nicla Sense ME board turns on
  nicla::begin();
  nicla::leds.begin();    
  BHY2.begin(NICLA_BLE);
}

void loop() {
  //run this code in a loop
  nicla::leds.setColor(blue);  //turn blue LED on
  BHY2.update(10);
  delay(10);                  //wait 10 miliseconds
}