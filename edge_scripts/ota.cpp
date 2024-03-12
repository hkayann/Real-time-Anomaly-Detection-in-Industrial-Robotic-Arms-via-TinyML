/* 
 * Use this sketch if you want to control nicla from 
 * an external device acting as a host.
 * Here, nicla just reacts to external stimuli coming from
 * the eslov port or through BLE 
 * 
 * NOTE: Remember to choose your Nicla configuration! 
 * If Nicla is used as a Shield, provide the NICLA_AS_SHIELD parameter.
 * If you want to enable just one between I2C and BLE,
 * use NICLA_I2C or NICLA_BLE parameters.
 *
*/

#include "Arduino_BHY2.h"

// Set DEBUG to true in order to enable debug print
#define DEBUG true

void setup()
{
#if DEBUG
  Serial.begin(115200);
  BHY2.debug(Serial);
#endif
  BHY2.begin(NICLA_BLE);
}

void loop()
{
  static unsigned long lastUpdateTime = 0; // Keep track of the last update time
  const unsigned long updateInterval = 333; // Minimum time between updates in milliseconds

  unsigned long currentMillis = millis();
  if (currentMillis - lastUpdateTime > updateInterval) {
    // Enough time has passed since the last update; it's time to read and process new sensor data
    lastUpdateTime = currentMillis; // Update the last update time
    BHY2.update(); 
  }
}