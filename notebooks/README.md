## File Explanations

| File Name        | Description                                                                                      |
|------------------|--------------------------------------------------------------------------------------------------|
| `main.ipynb` | Includes the whole task classification, and anomaly detection works.|
| `exalens.ipynb`      | The anomaly detection work on the dataset provided by the Exalens.     |
| `cloud_train.ipynb`    | An example notebook that shows how to generate/tweak `model.cc` file and upload to Google Drive to be used for OTA.|
| `r1.ipynb`    | Implements anomaly detection on a robot arm degradation dataset where we classify the task based on the spead and payload |

The same logic provided in `cloud_train.ipynb` can also be utilized to upload `InfluxDB` files to log data on cloud (Google Drive).