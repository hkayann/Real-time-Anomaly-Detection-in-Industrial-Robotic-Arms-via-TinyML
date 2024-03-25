import os
import numpy as np
import pandas as pd
import tensorflow as tf
from sklearn.preprocessing import MinMaxScaler
from urllib.request import urlopen

# Fetch dataset
url = "https://raw.githubusercontent.com/jbrownlee/Datasets/master/daily-min-temperatures.csv"
data = pd.read_csv(urlopen(url))

# Preprocess data
# For simplicity, let's predict the temperature based on the last 5 days.
def create_dataset(X, time_steps=1):
    Xs, ys = [], []
    for i in range(len(X) - time_steps):
        v = X.iloc[i:(i + time_steps)].values
        Xs.append(v)
        ys.append(X.iloc[i + time_steps])
    return np.array(Xs), np.array(ys)

time_steps = 5
scaler = MinMaxScaler(feature_range=(0, 1))
scaled_data = scaler.fit_transform(data[['Temp']].values)

# Create dataset for LSTM
X, y = create_dataset(pd.DataFrame(scaled_data), time_steps)

# Split dataset (let's use the first 80% of the data for training and the rest for testing)
train_size = int(len(X) * 0.8)
X_train, X_test = X[:train_size], X[train_size:]
y_train, y_test = y[:train_size], y[train_size:]

# Define LSTM model
model = tf.keras.Sequential([
    tf.keras.layers.LSTM(50, activation='relu', input_shape=(X_train.shape[1], X_train.shape[2])),
    tf.keras.layers.Dense(1)
])

model.compile(optimizer='adam', loss='mean_squared_error')

# Train the model
model.fit(X_train, y_train, epochs=5, batch_size=32, validation_split=0.1, verbose=1)

# Model directory to save the TensorFlow SavedModel
MODEL_DIR = "../models/lstm_ex"
model.save(MODEL_DIR, save_format="tf")

# Convert the SavedModel to a TensorFlow Lite model
converter = tf.lite.TFLiteConverter.from_saved_model(MODEL_DIR)

# Enable Select TensorFlow operations
converter.target_spec.supported_ops = [
    tf.lite.OpsSet.TFLITE_BUILTINS,  # Enable TensorFlow Lite ops.
    tf.lite.OpsSet.SELECT_TF_OPS     # Enable TensorFlow ops.
]

# Disable lowering tensor list operations
converter._experimental_lower_tensor_list_ops = False

tflite_model = converter.convert()

# Path to save the TFLite model
tflite_model_path = "../models/lstm_ex/lstm_ex.tflite"

# Save the TFLite model to the file
with open(tflite_model_path, 'wb') as f:
    f.write(tflite_model)

# Get the size of the TFLite model file
tflite_file_size = os.path.getsize(tflite_model_path)

print(f"Model saved to: {tflite_model_path}")
print(f"Size of the TFLite model: {tflite_file_size} bytes")