import struct
import time
import serial
import numpy as np
import pandas as pd

def read_dataset():
    # Remove columns by their names
    drops = ['name', 'time']
    data_path_imu = "./dataset/IMU_10Hz.csv"
    dataset_imu = pd.read_csv(data_path_imu, on_bad_lines='warn', skiprows=range(1, 100))
    df_imu = dataset_imu.drop(columns=drops)
    delay = 120
    data_arrays = [df_imu.iloc[delay:, i].reset_index(drop=True) for i in range(9)]
    np_acc_x, np_acc_y, np_acc_z, np_gyro_x, np_gyro_y, np_gyro_z, np_mag_x, np_mag_y, np_mag_z = data_arrays
    # Combine the features per modality.
    np_acc = np.stack([np_acc_x, np_acc_y, np_acc_z], axis=1)
    np_gyro = np.stack([np_gyro_x, np_gyro_y, np_gyro_z], axis=1)
    np_mag = np.stack([np_mag_x, np_mag_y, np_mag_z], axis=1)
    # Filter the features with rolling median.
    # Rolling mean function.
    def moving_average(arr, window):
        return np.apply_along_axis(lambda x:
                                np.convolve(x, np.ones(window),
                                'valid') / window, axis=0, arr=arr)

    # Set the window size.
    window_size = 5

    # Get moving average with given window size.
    # _f means filtered.
    np_acc_f = moving_average(np_acc, window_size)
    np_gyro_f = moving_average(np_gyro, window_size)
    np_mag_f = moving_average(np_mag, window_size)
    # Check the lengths.
    print(f"Lengths: acc-{len(np_acc_f)}, gyro-{len(np_gyro_f)}, mag-{len(np_mag_f)}")

    # Combine the features.
    df_filtered = np.concatenate((np_acc_f, np_gyro_f, np_mag_f), axis=1)
    # Split the dataset into 80%, 20%, 20%.
    # Calculate split indices.
    total_rows = df_filtered.shape[0]
    split_1 = int(total_rows * 0.8)
    split_2 = int(total_rows * 0.9)

    # Split the array
    df_train, df_validation, df_test = np.split(df_filtered, [split_1, split_2])

    # Convert arrays to dataframes
    # train_df = pd.DataFrame(df_train)
    # val_df = pd.DataFrame(df_validation)
    test_df = pd.DataFrame(df_test)

    print(f"Train shape: {df_train.shape}")
    print(f"Validation shape: {df_validation.shape}")
    print(f"Test shape: {df_test.shape}")
    # Convert to numpy array
    np_test = test_df.values
    # Return that array.
    return np_test

def create_sliding_windows(data_test, window_size=45):
    """
    Create sliding windows of size (45, 9) from input data and slide by 1.
    
    Parameters:
        data (np.array): The input data of shape (87472, 9).
        window_size (int): The size of the sliding window.
    
    Returns:
        np.array: 3D array where each slice is a sliding window of the input data.
    """
    num_windows = data_test.shape[0] - window_size + 1
    new_data = np.zeros((num_windows, window_size, data_test.shape[1]))

    for i in range(num_windows):
        new_data[i] = data_test[i:i+window_size]

    return new_data

# Get the test dataset.
np_test = read_dataset()
np_test = np.round(np_test, 7)
windows = create_sliding_windows(np_test, 45)

# Setup
PORT = "/dev/cu.usbmodemAA7EAA932"  # Adjust the port accordingly
BAUDRATE = 115200

# Open serial port
ser = serial.Serial(PORT, BAUDRATE, timeout=1)
time.sleep(2)  # Wait for the connection to be established

# Assuming received_floats_storage is defined earlier in your code
received_floats_storage = []

x = 0
for window in windows:
    start_time = time.time()
    floats_to_send = window.flatten()
    # Send bytes in 9 turns of 45 floats each
    for i in range(9):
        # Select 45 floats for current turn
        current_floats = floats_to_send[i*45:(i+1)*45]

        # Convert the float list to bytes
        # Ensure current_floats is a list or tuple of 45 floats
        floats_bytes = struct.pack('<' + 'f'*45, *current_floats)
        try:
            ser.write(floats_bytes)
        except Exception as e:
            print(f"Error writing to serial: {e}")

        # We assume 50 ms is enough for Arduino to process the data.
        time.sleep(0.1) 

        # If it's the final turn, wait and read acknowledgment
        if i == 8:
            # Read 36 bytes (9 floats) from Arduino and unpack them
            try:
                while ser.inWaiting() < 36:
                    time.sleep(0.01)  # Short delay to wait for data
                ack = ser.read(36)
            except Exception as e:
                print(f"Error reading from serial: {e}")
                ack = None

            try:
                if len(ack) == 36:
                    final_floats = struct.unpack('f'*9, ack)
                    # Proceed as before with final_floats
                else:
                    print(f"Received incorrect number of bytes: {len(ack)}")
                    break
            except Exception as e:
                print(f"Error unpacking data: {e}")
            # Round the received floats for display
            rounded_floats = tuple(round(num, 7) for num in final_floats)

            print(f"Window {x+1}, Received floats: {rounded_floats}")

            # Store received floats
            received_floats_storage.append(rounded_floats)
    end_time = time.time()  # End timing
    elapsed_time = end_time - start_time  # Calculate elapsed time
    print(f"Time taken for window {x+1}: {elapsed_time:.5f} seconds")
    x += 1
    if x == 2:
        break

# Close port
ser.close()
# Check the stored received floats
# print("Stored received floats: ", received_floats_storage)

# Save the received floats to a file using numpy.save
np.save('received_floats.npy', received_floats_storage)

# Alternatively, you can save the data to a text file:
with open('received_floats.txt', 'w') as f:
    for float_set in received_floats_storage:
        line = ', '.join(map(str, float_set))  # Convert the floats to strings and join them
        f.write(line + '\n')