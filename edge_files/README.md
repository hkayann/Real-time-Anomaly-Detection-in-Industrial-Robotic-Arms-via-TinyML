## File Explanations

| File Name        | Description                                                                                      |
|------------------|--------------------------------------------------------------------------------------------------|
| `simulate.cpp` | Run with `simulate.py` to send data over serial run inference and write the output to text file.     |
| `ram_test.cpp`      | Run to see how much RAM is available after loading libraries.     |
| `one_inference.cpp`    | Runs inference only once to test if TensorFlow is working.                                 |
| `blink_red.cpp`  | Blinks red LED, use to test OTA.                 |
| `blink_blue.cpp`       | Blinks blue LED, use to test OTA with the other blink sketch.                       |
| `empty.cpp`          | Does nothing, used in power consumption analysis. |
| `sleep.cpp`     | Forces board to enter sleep mode via introducing delay.    |
| `ota.cpp`     | Enables OTA, use bin files from blink sketchs to test OTA feature.    |
| `led.cpp`     | Enables LED, used in power consumption analysis.    |
| `IMU.cpp`     | Generates IMU data via accelerometer, gyroscope, and magnetomer and prints to serial. |
| `main.cpp`     | Generates IMU data, run inference, detects anomaly, and advertise IMU data and anomaly status over BLE, while having OTA enabled.  |
| `simulate.py`     | Run with `simulate.cpp`. Send IMU data over serial, fetch output from `cpp` and writes to a `txt` file.|
| `platformio.ini`     | The configuration file to be used within `PlatformIO`.|
| `model.h`     | The header file where we declare model and standardization parameters.|

The `main.cpp` file can be used to run both **LSTM** and **1D-CNN models** if generated via the same way we have done in `main.ipynb`. Edit the file accordingly to run any of the models.

You need to edit the file path and board port in `simulate.py`.

## Header File

The `model.h` file must stay exactly the same unless you prefer different naming conventions. In addition to the model declaration, the `mean` and `standard deviation` of each feature are defined as we need them to standardize our features before the **inference**. This file should be included in `tflm-lib` folder.

## CC File

The `model.cc` file provides the actual data and generated via the `xxd` tool. However, the generated version does not contain the standardization parameters and the parameter format is also undesired, hence requires further editing where we do via the `Python` script. 