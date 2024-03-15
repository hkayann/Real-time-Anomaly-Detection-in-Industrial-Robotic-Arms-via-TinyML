// Intialise library which communicates with RGB driver
// Functions accessible under 'nicla' namespace
#include "Nicla_System.h" 
#include "Arduino_BHY2.h"

SensorXYZ accelerometer(SENSOR_ID_ACC);
float accX, accY, accZ, gyroX, gyroY, gyroZ;

void setup() {
  //run this code once when Nicla Sense ME board turns on
  nicla::begin();               // initialise library
  nicla::leds.begin();    
  BHY2.begin(NICLA_BLE);
  if (!accelerometer.begin()) {
    Serial.println("Failed to initialize accelerometer!");
    while (1);
  }      // Start I2C connection
}

void loop() {
  //run this code in a loop
  nicla::leds.setColor(red);  //turn green LED on
  BHY2.update(10);
  delay(10);
}