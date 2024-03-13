## Summary

The figures utilized in my thesis can be found here. Please provide reference in case of a use.

## File Explanations

| File Name        | Description                                                                                      |
|------------------|--------------------------------------------------------------------------------------------------|
| `absoluteMaxVelocities.pdf` | Run with `simulate.py` to send data over serial run inference and write the output to text file.     |
| `absoluteSumVelocities.pdf`      | Run to see how much RAM is available after loading libraries.     |
| `anomalousTransitions.pdf`    | Runs inference only once to test if TensorFlow is working.                                 |
| `armJoints.pdf`  | Blinks red LED, use to test OTA.                 |
| `attackerModel.pdf`       | Blinks blue LED, use to test OTA with the other blink sketch.                       |
| `baseline_windows.pdf`          | Does nothing, used in power consumption analysis. |
| `colouredTasks.pdf`     | Forces board to enter sleep mode via introducing delay.    |
| `corr_heatmap_IMU_use_case_2.pdf`     | Enables OTA, use bin files from blink sketchs to test OTA feature.    |
| `corr_heatmap_IMU.pdf`     | Enables LED, used in power consumption analysis.    |
| `corr_heatmap_madgwick.pdf`     | Generates IMU data via accelerometer, gyroscope, and magnetomer and prints to serial. |
| `corr_heatmap_mahony.pdf`     | Generates IMU data, run inference, detects anomaly, and advertise IMU data and anomaly status over BLE, while having OTA enabled.  |
| `cyberPhysicalDomain.pdf`     | Run with `simulate.cpp`. Send IMU data over serial, fetch output from `cpp` and writes to a `txt` file.|
| `data_breaches_by_industry.pdf`     | The configuration file to be used within `PlatformIO`.|

The `main.cpp` file can be used to run both **LSTM** and **1D-CNN models** if generated via the same way we have done in `main.ipynb`. Edit the file accordingly to run any of the models.

You need to edit the file path and board port in `simulate.py`.