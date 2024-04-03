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
| `BLEHandler.cpp`     | The customized version for **RAM** optimization.|
| `BoschSensortec.h`     | The customized version for **RAM** optimization.|

The `main.cpp` file can be used to run both **LSTM** and **1D-CNN models** if generated via the same way we have done in `main.ipynb`. Edit the file accordingly to run any of the models.

You need to edit the file path and board port in `simulate.py`.

## Header File

The `model.h` file must stay exactly the same unless you prefer different naming conventions. In addition to the model declaration, the `mean` and `standard deviation` of each feature are defined as we need them to standardize our features before the **inference**. This file should be included in `tflm-lib` folder.

## CC File

The `model.cc` file, generated via the `xxd` tool, provides the actual model. However, the generated version does not contain the standardization parameters and the parameter format is also undesired, hence requires further editing where we do via the `Python` script.

## Bin Files

`Bin` files are provided so you can test if `OTA` is working. Just provide the correct file path in `ota.js`.

## RAM Optimization

Two source code files (`BLEHandler.cpp` and `BoschSensortec.h`) can be modified the save more RAM. The versions in the `lib` folder are the originals. These ones are the modified ones. Copy/paste accordingly to test.

In addition more RAM can be saved via recompilation of the few source files. Follow the below steps which are originally provided on the [Bosch forum](https://community.bosch-sensortec.com/t5/MEMS-sensors-forum/Nicla-Sense-ME-Large-Array-Crash/m-p/89087#M16803) after our discussions:

1. Clone the repos:
    1.1 `mkdir -p ~/Arduino/hardware/arduino`
    1.2 `cd ~/Arduino/hardware/arduino`
    1.3 `git clone https://github.com/arduino/ArduinoCore-mbed.git`
    1.4 `cd ArduinoCore-mbed`
    1.5 `mkdir -p ~/Arduino`
    1.6 `cd ~/Arduino`
    1.7 `git clone git@github.com:arduino/ArduinoCore-API`
    1.8 `cd ~/Arduino/hardware/arduino/ArduinoCore-mbed/cores/arduino`
    1.9 `ln -sf ~/Arduino/ArduinoCore-API/api .`
    1.10 `export mbed_os_repo=~/prj/mbed-os`
    1.11 `mkdir -p ~/prj`
    1.12 `cd ~/prj`
    1.13 `git clone https://github.com/ARMmbed/mbed-os.git`
2. Default MbedOS generation:
    2.1 `export arduino_core=~/Arduino/hardware/arduino/ArduinoCore-mbed`
    2.2 `cd $arduino_core`
    2.3 `./mbed-os-to-arduino -a -r $mbed_os_repo NICLA:NICLA` => If you are running this from macOS the command is `./mbed-os-to-arduino-macos`.
3. Customized MbedOS generation:
    3.1 At this step, the changes should be done to `./variants/NICLA/conf/mbed_app.json` to the value of `"rtos.main-thread-stack-size": 3072,`. We observed that, the app runs fine if set to `2072`. The default paratemeters are available at [ArduinoCore-mbed GitHub](https://github.com/arduino/ArduinoCore-mbed/blob/main/variants/NICLA/conf/mbed_app.json) repo.
    3.2 Rebuild via `./mbed-os-to-arduino -r $mbed_os_repo NICLA:NICLA`

The rebuilding process provides two customized files which you have to manually replace => `libmed.a` and `mbed_config.h`.

The BLE buffers can also be edited the same way, but we do not suggest as they introduce some inconsistencies to BLE process.