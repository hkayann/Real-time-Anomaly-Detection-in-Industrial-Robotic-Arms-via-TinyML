## File Explanations

| File Name        | Description                                                                                      |
|------------------|--------------------------------------------------------------------------------------------------|
| `ota.js` | How to upload the new `firmware.bin` to Nicla over BLE. |
| `extra_script.py` | Required to generate `bin` files, otherwise PlatformIO API only generates `hex` file.|
| `get_model.py` | How to download the `model.cc` file from Google Drive.|


## How to do this via Node-RED?

To trigger these scripts, we use the [Exec node.](https://flowfuse.com/node-red/core-nodes/exec/).

## Google Drive

We utilize Google Drive to store/fetch `model.cc` which contains the updated model and scaling parameters.

The process is provided in `cloud_train.ipynb`. You have to set up a `Google Cloud` project.

Check the following page to learn how to set up a project: [Google's OAuth 2.0 protocols](https://developers.google.com/identity/protocols/oauth2).

## How to OTA via Nicla?

You have to manually set correct paths for each file.

1. The `cloud_train.ipynb` which is included in `notebooks` folder, demonstrate how to tweak/upload new `model.cc` to Google Drive. First step is uploading new file to a **Google Drive**.
2. The new `model.cc` will downloaded from **Google Drive** when it is available via the `get_model.py` file. 
3. Then the `pio run` command should be executed from the path where the platfromIO project is. This will generated the `.bin` file. 
4. Finally trigger `ota.js` and the new firmware will be uploaded to **Nicla**.

## How to OTA via Portenta?

This process is pretty straight forward and explained in [Arduino Docs](https://docs.arduino.cc/tutorials/portenta-h7/over-the-air-update/).