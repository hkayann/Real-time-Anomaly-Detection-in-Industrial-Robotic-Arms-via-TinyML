/**
 * Nicla Sense Specs:
 * 512KB Flash / 64KB RAM,
 * 2MB SPI Flash for storage,
 * 2MB QSPI dedicated for BHI260AP.
*/

#include <math.h>
#include "model.h"
// #include "BLEHandler.h"
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/micro/micro_mutable_op_resolver.h"
#include "tensorflow/lite/micro/system_setup.h"
#include "tensorflow/lite/schema/schema_generated.h"
#include "tensorflow/lite/core/c/common.h"
#include "tensorflow/lite/micro/micro_log.h"
#include "Arduino_BHY2.h"
#include "ArduinoBLE.h"
// #include "Nicla_System.h"

/**
 * Important that the Arduino include comes last if on the Arduino platform, as it has an `abs()` function
 * that will screw with the stdlib abs() function. If need you can use the following lines
 * as well to redeclare the abs() function to be compatible
*/
#include "Arduino.h"
#ifdef ARDUINO
#define abs(x) ((x)>0?(x):-(x))
#endif 


//----------------------------------------------------------------------------------------------------------------------
// BLE UUIDS
//----------------------------------------------------------------------------------------------------------------------

#define BLE_UUID_IMU_SERVICE              "5543e0d651ca11ecbf630242ac130002"
#define BLE_UUID_ACC_CHAR                 "5543e32e51ca11ecbf630242ac130002"
#define BLE_UUID_GYRO_CHAR                "5543e55451ca11ecbf630242ac130002"
#define BLE_UUID_MAG_CHAR                 "5543e64451ca11ecbf630242ac130002"
#define BLE_UUID_STATUS_CHAR              "5543e74451ca11ecbf630242ac130002"

//----------------------------------------------------------------------------------------------------------------------
// APP & I/O
//----------------------------------------------------------------------------------------------------------------------

//#define NUMBER_OF_SENSORS 3

#define ACC_SENSOR_UPDATE_INTERVAL                (300)
#define MAG_SENSOR_UPDATE_INTERVAL                (300)
#define GYRO_SENSOR_UPDATE_INTERVAL               (300)

union sensor_data {
  struct __attribute__((packed)) {
    float values[3]; // float array for data (it holds 3)
    bool updated = false;
  };
  uint8_t bytes[3 * sizeof(float)]; // size as byte array 
};

union sensor_data accData;
union sensor_data gyroData;
union sensor_data magData;

SensorXYZ accelerometer(SENSOR_ID_ACC);
SensorXYZ gyro(SENSOR_ID_GYRO);
SensorXYZ mag(SENSOR_ID_MAG);

float accX, accY, accZ, gyroX, gyroY, gyroZ, magX, magY, magZ;
bool anomalyStatus = false;

//----------------------------------------------------------------------------------------------------------------------
// BLE
//----------------------------------------------------------------------------------------------------------------------

#define BLE_DEVICE_NAME                           "NICLA"
#define BLE_LOCAL_NAME                            "NICLA"

BLEService IMUService(BLE_UUID_IMU_SERVICE);
BLECharacteristic accCharacteristic(BLE_UUID_ACC_CHAR, BLERead | BLENotify, sizeof accData.bytes);
BLECharacteristic gyroCharacteristic(BLE_UUID_GYRO_CHAR, BLERead | BLENotify, sizeof gyroData.bytes);
BLECharacteristic magCharacteristic(BLE_UUID_MAG_CHAR, BLERead | BLENotify, sizeof magData.bytes);
BLECharacteristic statusCharacteristic(BLE_UUID_STATUS_CHAR, BLERead | BLENotify, sizeof(float));

#define BLE_LED_PIN                               LED_BUILTIN

//----------------------------------------------------------------------------------------------------------------------
// SENSOR TASKS
/*
 * We define bool function for each sensor.
 * Function returns true if sensor data are updated.
 * Allows us to define different update intervals per sensor data.
 */
//----------------------------------------------------------------------------------------------------------------------

bool accSensorTask() {
  static long previousMillis = 0;
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis < ACC_SENSOR_UPDATE_INTERVAL) {
    return false;
  }
  previousMillis = currentMillis;
  
  // get raw acc.
  accX = accelerometer.x();
  accY = accelerometer.y();
  accZ = accelerometer.z();
  // conver to m/s-2
  accX = (accX / 4096) * 9.80665;
  accY = (accY / 4096) * 9.80665;
  accZ = (accZ / 4096) * 9.80665;
  // set the values
  accData.values[0] = accX;
  accData.values[1] = accY;
  accData.values[2] = accZ;
  accData.updated = true;
  
  return accData.updated;
}

bool gyroSensorTask() {
  static long previousMillis = 0;
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis < GYRO_SENSOR_UPDATE_INTERVAL) {
    return false;
  }
  previousMillis = currentMillis;
  // get raw gyro
  gyroX = gyro.x();
  gyroY = gyro.y();
  gyroZ = gyro.z();
  // convert to dps
  gyroX = gyroX / (65536 / 4000);
  gyroY = gyroY / (65536 / 4000);
  gyroZ = gyroZ / (65536 / 4000);
  // set the values
  gyroData.values[0] = gyroX;
  gyroData.values[1] = gyroY;
  gyroData.values[2] = gyroZ;
  gyroData.updated = true;
  
  return gyroData.updated;
}

bool magSensorTask() {
  static long previousMillis = 0;
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis < MAG_SENSOR_UPDATE_INTERVAL) {
    return false;
  }
  previousMillis = currentMillis;
  // get raw mag
  magX = mag.x();
  magY = mag.y();
  magZ = mag.z();
  //convert to uT
  magX = magX / (65536 / 2600); 
  magY = magY / (65536 / 2600); 
  magZ = magZ / (65536 / 5000);
  // set the values  
  magData.values[0] = magX;
  magData.values[1] = magY;
  magData.values[2] = magZ;
  magData.updated = true;
  
  return magData.updated;
}

//----------------------------------------------------------------------------------------------------------------------
// PRINT TASKS
/*
 * Print tasks per sensor type.
 * Useful to test accuracy of sensor data before sending over BLE.
 */
//----------------------------------------------------------------------------------------------------------------------

void accPrintTask() {
  Serial.print("AccX = ");
  Serial.print(accData.values[0], 10);
  Serial.println(" G");

  Serial.print("AccY = ");
  Serial.print(accData.values[1], 10);
  Serial.println(" G");

  Serial.print("AccZ = ");
  Serial.print(accData.values[2], 10);
  Serial.println(" G");

  Serial.print("Acc. Subscription Status: ");
  Serial.println(accCharacteristic.subscribed());
}

void gyroPrintTask() {
  Serial.print("gyroX = ");
  Serial.print(gyroData.values[0], 10);
  Serial.println(" dps");

  Serial.print("gyroY = ");
  Serial.print(gyroData.values[1], 10);
  Serial.println(" dps");

  Serial.print("gyroZ = ");
  Serial.print(gyroData.values[2], 10);
  Serial.println(" dps");

  Serial.print("Gyro. Subscription Status: ");
  Serial.println(gyroCharacteristic.subscribed());
}

void magPrintTask() {
  Serial.print("magX = ");
  Serial.print(magData.values[0], 10);
  Serial.println(" uT");

  Serial.print("magY = ");
  Serial.print(magData.values[1], 10);
  Serial.println(" uT");

  Serial.print("magZ = ");
  Serial.print(magData.values[2], 10);
  Serial.println(" uT");

  Serial.print("Mag. Subscription Status: ");
  Serial.println(magCharacteristic.subscribed());
}

void anomalyStatusPrintTask() {
  Serial.print("Anomaly Status: ");
  Serial.println(anomalyStatus ? "Anomaly Detected" : "No Anomaly");

  Serial.print("Anomaly Status Subscription: ");
  Serial.println(statusCharacteristic.subscribed() ? "Subscribed" : "Not Subscribed");
}

void connectionStatusPrintTask() {
  Serial.print("BLE Connection Status: ");
  if (BLE.connected()) {
    Serial.println("Connected");
  } else {
    Serial.println("Not Connected");
  }
}

//----------------------------------------------------------------------------------------------------------------------
// Event Handlers & Initializers
/*
 * These are handlers that inform connection status
 * Useful when testing, might be removed later on.
 */
//----------------------------------------------------------------------------------------------------------------------

void blePeripheralConnectHandler(BLEDevice central) {
  digitalWrite(BLE_LED_PIN, HIGH);
  Serial.print(F( "Connected to central: " ));
  Serial.println(central.address());
}

void blePeripheralDisconnectHandler(BLEDevice central) {
  digitalWrite(BLE_LED_PIN, LOW);
  Serial.print(F("Disconnected from central: "));
  Serial.println(central.address());
}

//----------------------------------------------------------------------------------------------------------------------
//  BLE SETUP
/*
 * Determine which services/characteristics to be advertised.
 * Determine the device name.
 * Set event handlers.
 * Set inital value for characteristics.
 */
//----------------------------------------------------------------------------------------------------------------------

bool setupBleMode() {

    // set advertised local name and service UUID:
    BLE.setDeviceName(BLE_DEVICE_NAME);
    BLE.setLocalName(BLE_LOCAL_NAME);
    BLE.setAdvertisedService(IMUService);

    // BLE add characteristics
    IMUService.addCharacteristic(accCharacteristic);
    IMUService.addCharacteristic(gyroCharacteristic);
    IMUService.addCharacteristic(magCharacteristic);
    IMUService.addCharacteristic(statusCharacteristic);

    // add service
    BLE.addService(IMUService);

    // set the initial value for the characteristic:
    accCharacteristic.writeValue(accData.bytes, sizeof accData.bytes);
    gyroCharacteristic.writeValue(gyroData.bytes, sizeof gyroData.bytes);
    magCharacteristic.writeValue(magData.bytes, sizeof magData.bytes);

    float anomalyStatusValue = anomalyStatus ? 1.0f : 1.0f; // Convert boolean to float
    statusCharacteristic.writeValue((uint8_t*)&anomalyStatusValue, sizeof(anomalyStatusValue)); // Cast float to uint8_t* for writeValue

    // set BLE event handlers
    BLE.setEventHandler(BLEConnected, blePeripheralConnectHandler);
    BLE.setEventHandler(BLEDisconnected, blePeripheralDisconnectHandler);

    // start advertising
    BLE.advertise();
    return true;
}

void bleTask() {
    const uint32_t BLE_UPDATE_INTERVAL = 300;
    static uint32_t previousMillis = 0;
    uint32_t currentMillis = millis();

    if (currentMillis - previousMillis >= BLE_UPDATE_INTERVAL) {
        previousMillis = currentMillis;
        BLE.poll();

        if (accData.updated) {
        accCharacteristic.writeValue(accData.bytes, sizeof accData.bytes);
        accData.updated = false;
        }

        if (gyroData.updated) {
        gyroCharacteristic.writeValue(gyroData.bytes, sizeof gyroData.bytes);
        gyroData.updated = false;
        }

        if (magData.updated) {
        magCharacteristic.writeValue(magData.bytes, sizeof magData.bytes);
        magData.updated = false;
        }

        // Update anomaly status over BLE as a float
        float anomalyStatusValue = anomalyStatus ? 1.0f : 1.0f;
        statusCharacteristic.writeValue((uint8_t*)&anomalyStatusValue, sizeof(anomalyStatusValue)); // Cast float to uint8_t* for writeValue

    }
}

//----------------------------------------------------------------------------------------------------------------------
// Initializers
//----------------------------------------------------------------------------------------------------------------------

void initializeSensors()
{
    while (!accelerometer.begin())
    {
        delay(100); // Delay for 1 second
        Serial.println("Failed to initialize accelerometer!");
    }
    while (!gyro.begin())
    {
        delay(100); // Delay for 1 second
        Serial.println("Failed to initialize gyroscope!");
    }
    while (!mag.begin())
    {
        delay(100); // Delay for 1 second
        Serial.println("Failed to initialize magnetometer!");
    }
    while (!setupBleMode())
        delay(100); // Delay for 1 second before retrying
    Serial.println("Characteristics are set. Waiting for clients to connect.");
}

//----------------------------------------------------------------------------------------------------------------------
// TFLite Micro Operations
//----------------------------------------------------------------------------------------------------------------------

// Globals, used for compatibility with Arduino-style sketches.
namespace {
const tflite::Model* model = nullptr;
tflite::MicroInterpreter* interpreter = nullptr;
TfLiteTensor* input = nullptr;
TfLiteTensor* output = nullptr;
// int inference_count = 0;

// Arena size just a round number. The exact arena usage can be determined
// using the RecordingMicroInterpreter.
constexpr int kTensorArenaSize = 7012; // in bytes;
// Keep aligned to 16 bytes for CMSIS
alignas(16) uint8_t tensor_arena[kTensorArenaSize];
}  // namespace

void setup() {
  
    Serial.begin(115200);
    while(!Serial); // For debugging remove otherwise

    // Block until BHY2 sensor is initialized
    // while(!BHY2.begin(NICLA_STANDALONE)){;}
    while(!BHY2.begin(NICLA_BLE)){;}
    // BHY2.begin(NICLA_BLE);
    // BHY2.debug(Serial);
    // Block until BLE module is initialized
    // while(!BLE.begin()) {;}
    initializeSensors();
    delay(500);
    Serial.println("Initializing ML model...");
    tflite::InitializeTarget();
    Serial.println("TFlite initialized successfully!");


    // Map the model into a usable data structure. This doesn't involve any
    // copying or parsing, it's a very lightweight operation.
    Serial.println("Fetching model...");
    model = tflite::GetModel(g_model);
    Serial.println("Model is fetch!");
    if (model->version() != TFLITE_SCHEMA_VERSION) {
        Serial.print("Model provided is schema version ");
        Serial.print(model->version());
        Serial.print(" not equal to supported version ");
        Serial.println(TFLITE_SCHEMA_VERSION);
    } else {
        Serial.print("Model version: ");
        Serial.println(model->version());
    }
  
    // This pulls in all the operation implementations we need.
    // NOLINTNEXTLINE(runtime-global-variables)
    Serial.println("Pulling all operations...");
    // This pulls in all the operation implementations we need.
    // 4 for LSTM 5 for CNN
    static tflite::MicroMutableOpResolver<5> resolver;
  
    // Operation for CNN
    // resolver.AddConv2D();
    // resolver.AddMaxPool2D();
    // resolver.AddRelu();
    // resolver.AddReshape();
    // resolver.AddFullyConnected();

    // Needed for LSTM
    resolver.AddUnidirectionalSequenceLSTM();
    resolver.AddReshape();
    resolver.AddStridedSlice();
    resolver.AddFullyConnected();

    Serial.println("Operations/layers are pulled and set.");

    // Build an interpreter to run the model with
    static tflite::MicroInterpreter static_interpreter(
        model, resolver, tensor_arena, kTensorArenaSize);
    interpreter = &static_interpreter;
    Serial.println("Interpreter is built.");
  
    // Allocate memory from the tensor_arena for the model's tensors.
    TfLiteStatus allocate_status = interpreter->AllocateTensors();
    if (allocate_status != kTfLiteOk) {
        // TF_LITE_REPORT_ERROR(error_reporter, "Tensor allocation failed");
        Serial.println("AllocateTensors() failed.");
        Serial.println(allocate_status);
        return;
    } else {
        Serial.println("AllocateTensors() successed.");
        size_t used_bytes = interpreter->arena_used_bytes();
        Serial.print("Tensor arena used bytes: ");
        Serial.println(used_bytes);
    }

    // Print out the input tensor's details to verify
    // the model is working as expected
    // Obtain pointers to the model's input and output tensors.
    input = interpreter->input(0);
    output = interpreter->output(0);
    Serial.print("Input # dimensions: ");
    Serial.println(input->dims->size);
    Serial.print("Input bytes: ");
    Serial.println(input->bytes);
    Serial.print("Output # dimensions: ");
    Serial.println(output->dims->size);
    Serial.print("Output bytes: ");
    Serial.println(output->bytes);
    for (int i = 0; i < input->dims->size; i++) {
        Serial.print("Input dim ");
        Serial.print(i);
        Serial.print(": ");
        Serial.println(input->dims->data[i]);
    }
    for (int i = 0; i < output->dims->size; i++) {
        Serial.print("Output dim ");
        Serial.print(i);
        Serial.print(": ");
        Serial.println(output->dims->data[i]);
    }
}

const int numRows = 45;
const int numFloatsPerRow = 9;
float floatValues[numRows * numFloatsPerRow];
const int outputSize = 9; 
float modelOutput[outputSize];
int updateCount = 0;

void printFloatValues() {
    Serial.println("Printing Sensor Data Buffer:");
    for (int i = 0; i < numRows; ++i) {
        Serial.print("Timestep ");
        Serial.print(i + 1);
        Serial.print(": ");
        for (int j = 0; j < numFloatsPerRow; ++j) {
            Serial.print(floatValues[i * numFloatsPerRow + j], 6); // Print with precision of 6 decimal places
            if (j < numFloatsPerRow - 1) {
                Serial.print(", ");
            }
        }
        Serial.println(); // New line for the next timestep
    }
}

void updateSensorDataBuffer(float accX, float accY, float accZ, float gyroX, float gyroY, float gyroZ, float magX, float magY, float magZ) {
    
    if (updateCount < numRows) {
        updateCount++;
    }
    
    // Standardize the sensor data
    accX = (accX - ACCX_MEAN) / ACCX_STD;
    accY = (accY - ACCY_MEAN) / ACCY_STD;
    accZ = (accZ - ACCZ_MEAN) / ACCZ_STD;
    
    gyroX = (gyroX - GYROX_MEAN) / GYROX_STD;
    gyroY = (gyroY - GYROY_MEAN) / GYROY_STD;
    gyroZ = (gyroZ - GYROZ_MEAN) / GYROZ_STD;
    
    magX = (magX - MAGX_MEAN) / MAGX_STD;
    magY = (magY - MAGY_MEAN) / MAGY_STD;
    magZ = (magZ - MAGZ_MEAN) / MAGZ_STD;
    
    // Shift existing data to make room for new data
    // Since we're dealing with a flat array, we need to shift all elements except the last 9 features
    for (int i = 0; i < (numRows - 1) * numFloatsPerRow; i++) {
        floatValues[i] = floatValues[i + numFloatsPerRow];
    }

    // Index where new data should be added
    int startIndex = (numRows - 1) * numFloatsPerRow;

    // Add new standardized data at the end of the buffer
    floatValues[startIndex] = accX;
    floatValues[startIndex + 1] = accY;
    floatValues[startIndex + 2] = accZ;
    floatValues[startIndex + 3] = gyroX;
    floatValues[startIndex + 4] = gyroY;
    floatValues[startIndex + 5] = gyroZ;
    floatValues[startIndex + 6] = magX;
    floatValues[startIndex + 7] = magY;
    floatValues[startIndex + 8] = magZ;

}

float outputMeans[] = {ACCX_MEAN, ACCY_MEAN, ACCZ_MEAN, GYROX_MEAN, GYROY_MEAN, GYROZ_MEAN, MAGX_MEAN, MAGY_MEAN, MAGZ_MEAN};
float outputStdDevs[] = {ACCX_STD, ACCY_STD, ACCZ_STD, GYROX_STD, GYROY_STD, GYROZ_STD, MAGX_STD, MAGY_STD, MAGZ_STD};
float predictedAccX = 0, predictedAccY = 0, predictedAccZ = 0;
float predictedGyroX = 0, predictedGyroY = 0, predictedGyroZ = 0;
float predictedMagX = 0, predictedMagY = 0, predictedMagZ = 0;
float cumulativeTotalRmse = 0;
int inferenceCount = 0;
int rmseIndex = 0;
float rmseValues[45] = {0};

void loop() {
    BHY2.update();
    bleTask();
    if (accSensorTask() && gyroSensorTask() && magSensorTask()) {
        updateSensorDataBuffer(accX, accY, accZ, gyroX, gyroY, gyroZ, magX, magY, magZ);
        if (updateCount == numRows) {
            float rmseAccX = sqrt(pow(predictedAccX - accX, 2));
            float rmseAccY = sqrt(pow(predictedAccY - accY, 2));
            float rmseAccZ = sqrt(pow(predictedAccZ - accZ, 2));
            float rmseGyroX = sqrt(pow(predictedGyroX - gyroX, 2));
            float rmseGyroY = sqrt(pow(predictedGyroY - gyroY, 2));
            float rmseGyroZ = sqrt(pow(predictedGyroZ - gyroZ, 2));
            float rmseMagX = sqrt(pow(predictedMagX - magX, 2));
            float rmseMagY = sqrt(pow(predictedMagY - magY, 2));
            float rmseMagZ = sqrt(pow(predictedMagZ - magZ, 2));

            float totalRmse = rmseAccX + rmseAccY + rmseAccZ + 
                            rmseGyroX + rmseGyroY + rmseGyroZ + 
                            rmseMagX + rmseMagY + rmseMagZ;
                            
            // cumulativeTotalRmse += totalRmse;
            // inferenceCount++;
            if (inferenceCount >= 45) {
                cumulativeTotalRmse -= rmseValues[rmseIndex];
            }
            cumulativeTotalRmse += totalRmse;
            rmseValues[rmseIndex] = totalRmse;
            rmseIndex = (rmseIndex + 1) % 45;
            inferenceCount++;

            // BLE Printing Tasks.
            accPrintTask();
            gyroPrintTask();
            magPrintTask();
            anomalyStatusPrintTask();
            connectionStatusPrintTask();

            if (sizeof(floatValues) == input->bytes) {
                memcpy(input->data.f, floatValues, input->bytes);
                TfLiteStatus invoke_status = interpreter->Invoke();
                if (invoke_status != kTfLiteOk) {
                    Serial.println("Invoke failed!");
                } else {
                    // Correctly copy and transform the model's output
                    for (int i = 0; i < outputSize; i++) {
                        // First, copy the raw output
                        modelOutput[i] = output->data.f[i];
                        // Then, apply mean and standard deviation adjustments
                        modelOutput[i] = (modelOutput[i] * outputStdDevs[i]) + outputMeans[i];
                    }
                    // Update predicted values
                    predictedAccX = modelOutput[0];
                    predictedAccY = modelOutput[1];
                    predictedAccZ = modelOutput[2];
                    predictedGyroX = modelOutput[3];
                    predictedGyroY = modelOutput[4];
                    predictedGyroZ = modelOutput[5];
                    predictedMagX = modelOutput[6];
                    predictedMagY = modelOutput[7];
                    predictedMagZ = modelOutput[8];
                }
            } else {
                Serial.println("Error: Size mismatch between floatValues and the model's input tensor.");
            }
        }
    }
}