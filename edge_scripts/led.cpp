// //This sketch is only for the Nicla Sense ME, not the Nicla Vision
// #ifdef ARDUINO_NICLA_VISION
//   #error "Run the standard Blink.ino sketch for the Nicla Vision"
// #endif

// // Intialise library which communicates with RGB driver
// // Functions accessible under 'nicla' namespace
// #include "Nicla_System.h" 

// void setup() {
//   //run this code once when Nicla Sense ME board turns on
//   nicla::begin();               // initialise library
//   nicla::leds.begin();          // Start I2C connection
// }

// void loop() {
//   //run this code in a loop
//   nicla::leds.setColor(green);  //turn green LED on
//   delay(100000);                  //wait 100 seconds
// }