/**
 * Nicla Sense Specs:
 * 512KB Flash / 64KB RAM,
 * 2MB SPI Flash for storage,
 * 2MB QSPI dedicated for BHI260AP.
*/

#include <math.h>
#include "model.h"
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

//----------------------------------------------------------------------------------------------------------------------
// APP & I/O
//----------------------------------------------------------------------------------------------------------------------

//#define NUMBER_OF_SENSORS 3

#define ACC_SENSOR_UPDATE_INTERVAL                (50) // node-red-dashboard can't handle 1 ms, can handle 100 ms.
#define MAG_SENSOR_UPDATE_INTERVAL                (50)
#define GYRO_SENSOR_UPDATE_INTERVAL               (50)


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

//----------------------------------------------------------------------------------------------------------------------
// BLE
//----------------------------------------------------------------------------------------------------------------------

#define BLE_DEVICE_NAME                           "Nicla"
#define BLE_LOCAL_NAME                            "Nicla"

BLEService IMUService(BLE_UUID_IMU_SERVICE);
BLECharacteristic accCharacteristic(BLE_UUID_ACC_CHAR, BLERead | BLENotify, sizeof accData.bytes);
BLECharacteristic gyroCharacteristic(BLE_UUID_GYRO_CHAR, BLERead | BLENotify, sizeof gyroData.bytes);
BLECharacteristic magCharacteristic(BLE_UUID_MAG_CHAR, BLERead | BLENotify, sizeof magData.bytes);

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
  static long previousMillis2 = 0;
  unsigned long currentMillis2 = millis();
  if (currentMillis2 - previousMillis2 < ACC_SENSOR_UPDATE_INTERVAL) {
    return false;
  }
  previousMillis2 = currentMillis2;
  
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
  static long previousMillis2 = 0;
  unsigned long currentMillis2 = millis();
  if (currentMillis2 - previousMillis2 < GYRO_SENSOR_UPDATE_INTERVAL) {
    return false;
  }
  previousMillis2 = currentMillis2;
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
  static long previousMillis3 = 0;
  unsigned long currentMillis3 = millis();
  if (currentMillis3 - previousMillis3 < MAG_SENSOR_UPDATE_INTERVAL) {
    return false;
  }
  previousMillis3 = currentMillis3;
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
//   Serial.print("AccX = ");
//   Serial.print(accData.values[0], 10);
//   Serial.println(" G");

//   Serial.print("AccY = ");
//   Serial.print(accData.values[1], 10);
//   Serial.println(" G");

//   Serial.print("AccZ = ");
//   Serial.print(accData.values[2], 10);
//   Serial.println(" G");

//   Serial.print("Acc. Subscription Status: ");
//   Serial.println(accCharacteristic.subscribed());
}

void gyroPrintTask() {
//   Serial.print("gyroX = ");
//   Serial.print(gyroData.values[0], 10);
//   Serial.println(" dps");

//   Serial.print("gyroY = ");
//   Serial.print(gyroData.values[1], 10);
//   Serial.println(" dps");

//   Serial.print("gyroZ = ");
//   Serial.print(gyroData.values[2], 10);
//   Serial.println(" dps");

//   Serial.print("Gyro. Subscription Status: ");
//   Serial.println(gyroCharacteristic.subscribed());
}

void magPrintTask() {
//   Serial.print("magX = ");
//   Serial.print(magData.values[0], 10);
//   Serial.println(" uT");

//   Serial.print("magY = ");
//   Serial.print(magData.values[1], 10);
//   Serial.println(" uT");

//   Serial.print("magZ = ");
//   Serial.print(magData.values[2], 10);
//   Serial.println(" uT");

//   Serial.print("Mag. Subscription Status: ");
//   Serial.println(magCharacteristic.subscribed());
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

  // add service
  BLE.addService(IMUService);

  // set the initial value for the characteristic:
  accCharacteristic.writeValue(accData.bytes, sizeof accData.bytes);
  gyroCharacteristic.writeValue(gyroData.bytes, sizeof gyroData.bytes);
  magCharacteristic.writeValue(magData.bytes, sizeof magData.bytes);

  // set BLE event handlers
  BLE.setEventHandler(BLEConnected, blePeripheralConnectHandler);
  BLE.setEventHandler(BLEDisconnected, blePeripheralDisconnectHandler);

  // start advertising
  BLE.advertise();
  return true;
}

void bleTask()
{
  const uint32_t BLE_UPDATE_INTERVAL = 10;
  static uint32_t previousMillis = 0;
  uint32_t currentMillis = millis();
  if (currentMillis - previousMillis >= BLE_UPDATE_INTERVAL) {
    previousMillis = currentMillis;
    BLE.poll();
  }
  if (accData.updated) {
    // Bluetooth does not define accelerometer
    accCharacteristic.writeValue(accData.bytes, sizeof accData.bytes);
    accData.updated = false;
  }

  if (gyroData.updated) {
    // Bluetooth does not define gyroscope
    gyroCharacteristic.writeValue(gyroData.bytes, sizeof gyroData.bytes);
    gyroData.updated = false;
  }

  if (magData.updated) {
    // Bluetooth does not define magnetometer
    magCharacteristic.writeValue(magData.bytes, sizeof magData.bytes);
    magData.updated = false;
  }
}


//----------------------------------------------------------------------------------------------------------------------
// Initializers
//----------------------------------------------------------------------------------------------------------------------

void initializeSensors() {
  if (!accelerometer.begin()) {
    Serial.println("Failed to initialize accelerometer!");
    while (1);
  }
  if (!gyro.begin()) {
    Serial.println("Failed to initialize gyroscope!");
    while (1);
  }
  if (!mag.begin()) {
    Serial.println("Failed to initialize magnetometer!");
    while (1);
  }
  if (!setupBleMode()) {
    while (1);
  } else {
    Serial.println("Characteristics are set. Waiting for clients to connect.");
  }
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
constexpr int kTensorArenaSize = 6000; // in bytes;
// Keep aligned to 16 bytes for CMSIS
alignas(16) uint8_t tensor_arena[kTensorArenaSize];
}  // namespace


// Create required variables
const int numRows = 45;
const int numFloatsPerRow = 9;
float floatValues[numRows * numFloatsPerRow]; // Array to store floats generated by sensors
float predictedValues[numFloatsPerRow]; // Array to hold predictions per inference.
float totalRMSE = 0.0; // Array to store floats predicted by ML model 

void setup() {
  
  // Assign the initial value of variables
  floatValues[0] = 0.0;
  predictedValues[0] = 0.0;
  Serial.begin(115200);
  
  // For debugging purposes only.
  while(!Serial);

  // Block until BHY2 sensor is initialized
  while(!BHY2.begin(NICLA_STANDALONE)) {;}
  // Block until BLE module is initialized
  while(!BLE.begin()) {;}
  initializeSensors();
  delay(1000);
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
  static tflite::MicroMutableOpResolver<4> resolver;
   
   // Needed for 1D-CNN
//    resolver.AddConv2D();
//    resolver.AddExpandDims();
//    resolver.AddMaxPool2D();
//    resolver.AddPack();
//    resolver.AddReshape();
//    resolver.AddShape();
//    resolver.AddStridedSlice();
//    resolver.AddFullyConnected();
//    resolver.AddRelu();
  
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

void loop() {
  
    BHY2.update();
    bleTask();
    if (accSensorTask()) {
        accPrintTask();
    }
    if (gyroSensorTask()){
        gyroPrintTask();
    }
    if (magSensorTask()){
        magPrintTask();
    }
    
    // Predefined example data
    float example_input_data[45 * 9] = {
        9.8076075, 0.5755661, 0.2580949, -21.5625, -1.05, -2.0625, -38.5439995, -17.0880001, 40.7230774,
        9.9249233, 0.6133945, 0.2149993, -13.4375, -0.65, -1.3, -38.4959991, -18.3120003, 39.492308,
        9.8076075, 0.5755661, 0.2580949, -21.5625, -1.05, -2.0625, -38.5439995, -17.0880001, 40.7230774,
        9.9249233, 0.6133945, 0.2149993, -13.4375, -0.65, -1.3, -38.4959991, -18.3120003, 39.492308,
        9.8076075, 0.5755661, 0.2580949, -21.5625, -1.05, -2.0625, -38.5439995, -17.0880001, 40.7230774,
        9.9249233, 0.6133945, 0.2149993, -13.4375, -0.65, -1.3, -38.4959991, -18.3120003, 39.492308,
        9.8076075, 0.5755661, 0.2580949, -21.5625, -1.05, -2.0625, -38.5439995, -17.0880001, 40.7230774,
        9.9249233, 0.6133945, 0.2149993, -13.4375, -0.65, -1.3, -38.4959991, -18.3120003, 39.492308,
        9.8076075, 0.5755661, 0.2580949, -21.5625, -1.05, -2.0625, -38.5439995, -17.0880001, 40.7230774,
        9.9249233, 0.6133945, 0.2149993, -13.4375, -0.65, -1.3, -38.4959991, -18.3120003, 39.492308,
        9.8076075, 0.5755661, 0.2580949, -21.5625, -1.05, -2.0625, -38.5439995, -17.0880001, 40.7230774,
        9.9249233, 0.6133945, 0.2149993, -13.4375, -0.65, -1.3, -38.4959991, -18.3120003, 39.492308,
        9.8076075, 0.5755661, 0.2580949, -21.5625, -1.05, -2.0625, -38.5439995, -17.0880001, 40.7230774,
        9.9249233, 0.6133945, 0.2149993, -13.4375, -0.65, -1.3, -38.4959991, -18.3120003, 39.492308,
        9.8076075, 0.5755661, 0.2580949, -21.5625, -1.05, -2.0625, -38.5439995, -17.0880001, 40.7230774,
        9.9249233, 0.6133945, 0.2149993, -13.4375, -0.65, -1.3, -38.4959991, -18.3120003, 39.492308,
        9.8076075, 0.5755661, 0.2580949, -21.5625, -1.05, -2.0625, -38.5439995, -17.0880001, 40.7230774,
        9.9249233, 0.6133945, 0.2149993, -13.4375, -0.65, -1.3, -38.4959991, -18.3120003, 39.492308,
        9.8076075, 0.5755661, 0.2580949, -21.5625, -1.05, -2.0625, -38.5439995, -17.0880001, 40.7230774,
        9.9249233, 0.6133945, 0.2149993, -13.4375, -0.65, -1.3, -38.4959991, -18.3120003, 39.492308,
        9.8076075, 0.5755661, 0.2580949, -21.5625, -1.05, -2.0625, -38.5439995, -17.0880001, 40.7230774,
        9.9249233, 0.6133945, 0.2149993, -13.4375, -0.65, -1.3, -38.4959991, -18.3120003, 39.492308,
        9.8076075, 0.5755661, 0.2580949, -21.5625, -1.05, -2.0625, -38.5439995, -17.0880001, 40.7230774,
        9.9249233, 0.6133945, 0.2149993, -13.4375, -0.65, -1.3, -38.4959991, -18.3120003, 39.492308,
        9.8076075, 0.5755661, 0.2580949, -21.5625, -1.05, -2.0625, -38.5439995, -17.0880001, 40.7230774,
        9.9249233, 0.6133945, 0.2149993, -13.4375, -0.65, -1.3, -38.4959991, -18.3120003, 39.492308,
        9.8076075, 0.5755661, 0.2580949, -21.5625, -1.05, -2.0625, -38.5439995, -17.0880001, 40.7230774,
        9.9249233, 0.6133945, 0.2149993, -13.4375, -0.65, -1.3, -38.4959991, -18.3120003, 39.492308,
        9.8076075, 0.5755661, 0.2580949, -21.5625, -1.05, -2.0625, -38.5439995, -17.0880001, 40.7230774,
        9.9249233, 0.6133945, 0.2149993, -13.4375, -0.65, -1.3, -38.4959991, -18.3120003, 39.492308,
        9.8076075, 0.5755661, 0.2580949, -21.5625, -1.05, -2.0625, -38.5439995, -17.0880001, 40.7230774,
        9.9249233, 0.6133945, 0.2149993, -13.4375, -0.65, -1.3, -38.4959991, -18.3120003, 39.492308,
        9.8076075, 0.5755661, 0.2580949, -21.5625, -1.05, -2.0625, -38.5439995, -17.0880001, 40.7230774,
        9.9249233, 0.6133945, 0.2149993, -13.4375, -0.65, -1.3, -38.4959991, -18.3120003, 39.492308,
        9.8076075, 0.5755661, 0.2580949, -21.5625, -1.05, -2.0625, -38.5439995, -17.0880001, 40.7230774,
        9.9249233, 0.6133945, 0.2149993, -13.4375, -0.65, -1.3, -38.4959991, -18.3120003, 39.492308,
        9.8076075, 0.5755661, 0.2580949, -21.5625, -1.05, -2.0625, -38.5439995, -17.0880001, 40.7230774,
        9.9249233, 0.6133945, 0.2149993, -13.4375, -0.65, -1.3, -38.4959991, -18.3120003, 39.492308,
        9.8076075, 0.5755661, 0.2580949, -21.5625, -1.05, -2.0625, -38.5439995, -17.0880001, 40.7230774,
        9.9249233, 0.6133945, 0.2149993, -13.4375, -0.65, -1.3, -38.4959991, -18.3120003, 39.492308,
        9.8076075, 0.5755661, 0.2580949, -21.5625, -1.05, -2.0625, -38.5439995, -17.0880001, 40.7230774,
        9.9249233, 0.6133945, 0.2149993, -13.4375, -0.65, -1.3, -38.4959991, -18.3120003, 39.492308,
        9.8076075, 0.5755661, 0.2580949, -21.5625, -1.05, -2.0625, -38.5439995, -17.0880001, 40.7230774,
        9.9249233, 0.6133945, 0.2149993, -13.4375, -0.65, -1.3, -38.4959991, -18.3120003, 39.492308,
        9.8076075, 0.5755661, 0.2580949, -21.5625, -1.05, -2.0625, -38.5439995, -17.0880001, 40.7230774,
    };

    // Load the example data into the model's input tensor
    for (int i = 0; i < 45 * 9; i++) {
        input->data.f[i] = example_input_data[i];
    }
    // Record the start time
    unsigned long start_time = micros();

    // Invoke the model
    TfLiteStatus invoke_status = interpreter->Invoke();

    // Record the end time
    unsigned long end_time = micros();

    // Calculate the inference latency
    unsigned long inference_latency = end_time - start_time;

    
    if (invoke_status != kTfLiteOk) {
        Serial.println("Invoke failed!");
        return; // Exit if invocation fails
    }

        // Print the inference latency
    Serial.print("Inference latency: ");
    Serial.print(inference_latency);
    Serial.println(" microseconds");

    // // Get the output size and print the output data
    //   const int N = output->dims->data[2];
    //   for (int i = 0; i < N; i++) {
    //     float output_value = output->data.f[i];
    //     Serial.print("Output ");
    //     Serial.print(i);
    //     Serial.print(": ");
    //     Serial.println(output_value);
    //   }

    // Add a delay before the next loop iteration
    delay(1); // Adjust this delay as needed
}
