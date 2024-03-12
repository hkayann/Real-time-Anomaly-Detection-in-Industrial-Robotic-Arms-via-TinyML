#include "Arduino_BHY2.h"

SensorXYZ accelerometer(SENSOR_ID_ACC);
SensorXYZ gyro(SENSOR_ID_GYRO);
SensorXYZ mag(SENSOR_ID_MAG);

float accX, accY, accZ, gyroX, gyroY, gyroZ, magX, magY, magZ;

void setup() {
    Serial.begin(115200);
    BHY2.begin();
    accelerometer.begin();
    gyro.begin();
    mag.begin();
}
void loop() {
    // Update function should be continuously polled
    BHY2.update();

    // get raw acc.
    accX = accelerometer.x();
    accY = accelerometer.y();
    accZ = accelerometer.z();
    // conver to m/s-2
    accX = (accX / 4096) * 9.80665;
    accY = (accY / 4096) * 9.80665;
    accZ = (accZ / 4096) * 9.80665;
    // get raw gyro
    gyroX = gyro.x();
    gyroY = gyro.y();
    gyroZ = gyro.z();
    // convert to dps
    gyroX = gyroX / (65536 / 4000);
    gyroY = gyroY / (65536 / 4000);
    gyroZ = gyroZ / (65536 / 4000);
    // get raw mag
    magX = mag.x();
    magY = mag.y();
    magZ = mag.z();
    //convert to uT
    magX = magX / (65536 / 2600); 
    magY = magY / (65536 / 2600); 
    magZ = magZ / (65536 / 5000);

    // Print accelerometer data
    // Serial.print("Acc: ");
    Serial.print(accX, 5);
    Serial.print(", ");
    Serial.print(accY, 5);
    Serial.print(", ");
    Serial.print(accZ, 5);

    // Print gyroscope data
    // Serial.print("Gyro: ");
    Serial.print(", ");
    Serial.print(gyroX, 5);
    Serial.print(", ");
    Serial.print(gyroY, 5);
    Serial.print(", ");
    Serial.print(gyroZ, 5);

    // Print magnetometer data
    // Serial.print("Mag: ");
    Serial.print(", ");
    Serial.print(magX, 5);
    Serial.print(", ");
    Serial.print(magY, 5);
    Serial.print(", ");
    Serial.println(magZ, 5);
}