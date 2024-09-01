import csv
import logging
import sys
import time

from connector import RTDEConnect
from hashlib import new

sys.path.append('..')

ROBOT_HOST_RIGHT = '192.168.1.20'
ROBOT_HOST_LEFT = '192.168.1.10'

# 30004 is the port for RTDE
ROBOT_PORT = 30004
config_filename = '/home/hkayan/Documents/universalRobots/Python_code/Remote_Examples/control_loop_configuration_v1.xml'

# log errors
logging.getLogger().setLevel(logging.INFO)

# create object per arm
monitor_right = RTDEConnect(ROBOT_HOST_RIGHT, config_filename)
monitor_left = RTDEConnect(ROBOT_HOST_LEFT, config_filename)

# waypoints for the right arm
setp1_right = [-0.13311, -0.44332, 0.48116, 1.21324, 1.20813, -1.21244] 
setp2_right = [-0.13214, -0.29779, 0.16533, -2.22504, -2.21765, -0.00223]
setp3_right = [-0.13298, -0.47024, 0.09444, -2.22281, -2.21581, 0.00228]
setp4_right = [0.04682, -0.46214, 0.03844, 2.13656, 2.20663, 0.07478]
setp5_right = [-0.4433, 0.1331, 0.48119, -2.21997, 0.00461, 2.21779]
setp6_right = [-0.04436, 0.33159, 0.09818, 2.20805, -2.22283, -0.10647]

# rounded waypoints to compare
setp1_right_rounded = [round(num, 1) for num in setp1_right]
setp2_right_rounded = [round(num, 1) for num in setp2_right]
setp3_right_rounded = [round(num, 1) for num in setp3_right]
setp4_right_rounded = [round(num, 1) for num in setp4_right]
setp5_right_rounded = [round(num, 1) for num in setp5_right]
setp6_right_rounded = [round(num, 1) for num in setp6_right]

# grippers params in mm
release_width = 37.0
grip_width = 24.5

# arm speeds in rad/s
arm_speed_default = 1.05
arm_speed_10 = 1.155
arm_speed_35 = 1.4175
arm_speed_65 = 1.7325
arm_speed_100 = 2.10
arm_speed_slower_50 = 0.525
arm_speed_slower_5 = 0.9975
arm_speed_20 = 1.26
arm_speed_slower_25 = 0.7875

# anomaly state
normal = "0"
anomaly = "1"

# waypoints for the left arm
setp1_left = [-0.12895, -0.44462, 0.48003, 1.20892, 1.21131, -1.20398]
setp2_left = [-0.45662, 0.12889, 0.48, 2.22106, 0.00225, -2.21614]
setp3_left = [0.12847, 0.29873, 0.15235, 2.22446, -2.21307, -0.00101]
setp4_left = [-0.10457, 0.32664, 0.04779, 2.24321, -2.15347, -0.0295]
setp5_left = [0.08855, 0.36298, 0.34409, -1.32738, 2.32202, 1.04384]
setp6_left = [-0.42186, -0.2181, 0.47889, 1.78621, 0.70693, -1.77275]
setp7_left = [-0.05955, -0.46496, 0.11955, 2.17428, 2.26271, -0.02753]

# Rounded versions to compare with actual_TCP_pose
setp1_left_rounded = [round(num, 1) for num in setp1_left]
setp2_left_rounded = [round(num, 1) for num in setp2_left]
setp3_left_rounded = [round(num, 1) for num in setp3_left]
setp4_left_rounded = [round(num, 1) for num in setp4_left]
setp5_left_rounded = [round(num, 1) for num in setp5_left]
setp6_left_rounded = [round(num, 1) for num in setp6_left]
setp7_left_rounded = [round(num, 1) for num in setp7_left]

# we send the following for the left arm
# the left arm always work at default speed
parameters_left_1 = [-0.12895, -0.44462, 0.48003, 1.20892, 1.21131, -1.20398, release_width, arm_speed_default]
parameters_left_2 = [-0.45662, 0.12889, 0.48, 2.22106, 0.00225, -2.21614, release_width, arm_speed_default]
parameters_left_3 = [0.12847, 0.29873, 0.15235, 2.22446, -2.21307, -0.00101, release_width, arm_speed_default]
parameters_left_4 = [-0.10457, 0.32664, 0.04779, 2.24321, -2.15347, -0.0295, release_width, arm_speed_default]
parameters_left_4_grip = [-0.10457, 0.32664, 0.04779, 2.24321, -2.15347, -0.0295, grip_width, arm_speed_default]
parameters_left_5 = [0.08855, 0.36298, 0.34409, -1.32738, 2.32202, 1.04384, grip_width, arm_speed_default]
parameters_left_6 = [-0.42186, -0.2181, 0.47889, 1.78621, 0.70693, -1.77275, grip_width, arm_speed_default]
parameters_left_7 = [-0.05955, -0.46496, 0.11955, 2.17428, 2.26271, -0.02753, grip_width, arm_speed_default]
parameters_left_7_release = [-0.05955, -0.46496, 0.11955, 2.17428, 2.26271, -0.02753, release_width, arm_speed_default]

# This fixes 0 grip issue on left arm
monitor_left.sendall("sent_parameters", [-0.12895, -0.44462, 0.48003, 1.20892, 1.21131, -1.20398, release_width, arm_speed_default])
# This fixes movej error on right arm
monitor_right.sendall("sent_parameters", [-0.12895, -0.44462, 0.48003, 1.20892, 1.21131, -1.20398, release_width, arm_speed_default])

# dataset parameters
ts = "Timestamp" #Time elapsed since the controller was started
actual_q = "Actual Joint Positions"
actual_qd = "Actual Joint Velocities"
actual_current = "Actual Joint Currents"
joint_control_output = "Joint Control Currents"
actual_TCP_pose = "Actual Cartesian Coordinates"
actual_TCP_speed = "Actual Tool Speed" # m/s
actual_TCP_force = "Generalized Forces"
joint_temperatures = "Temperature of Each Joint" # Celcius
actual_execution_time = "Execution Time"
safety_status = "Safety Status"
actual_tool_accelerometer = "Tool Acceleration"
actual_momentum = "Norm of Cartesion Linear Momentum"
actual_main_voltage = "Main Voltage"
actual_robot_current = "Robot Current"
actual_joint_voltage = "Joint Voltages"
elbow_position = "Elbow Position"
elbow_velocity = "Elbow Velocity"
tool_mode = "Tool Mode"
tool_output_current = "Tool Current" # mA
tool_temperature = "Tool Temperature"
tcp_force_scalar = "TCP Force" # N
anomaly_state = "Anomaly State"

# open csv files for arms
with open('state_right_v3.csv', 'a', newline='') as state_right_csv, open('state_left_v3.csv', 'a', newline='') as state_left_csv:        
    field_names = [ts, actual_q, actual_qd, actual_current, 
                   actual_TCP_pose, actual_TCP_speed, actual_TCP_force, 
                   joint_temperatures, actual_execution_time, safety_status, 
                   actual_tool_accelerometer, actual_momentum, actual_robot_current, 
                   actual_joint_voltage, elbow_position, elbow_velocity,
                   tool_output_current, tool_temperature, tcp_force_scalar, anomaly_state]

    # write headers
    write_right = csv.DictWriter(state_right_csv, fieldnames=field_names)
    write_right.writeheader()
    write_left = csv.DictWriter(state_left_csv, fieldnames=field_names)
    write_left.writeheader()

    # start time in seconds
    initial_time = time.time()

    while True:
        try:

            # calculate the passed time
            time_diff = time.time() - initial_time
            print(f"The {time_diff} seconds passed.")

            # receive the current state
            state_right = monitor_right.receive()
            state_left = monitor_left.receive()

            if state_right is None:
                print("Check right arm!")
                raise KeyboardInterrupt
            if state_left is None:
                print("Check left arm!")
                raise KeyboardInterrupt

            """
            First 51840 secs (864 mins - 14.4 hours) we run everyting normal to generate training dataset (around 60% of total dataset)
            Then for 17280 secs (288 mins) we run normal + anomaly to generate validation dataset (around 20% of total dataset)
            Finaly at least for 17280 secs (288 mins) we run normal + anomaly to generate test dataset (around 20% of total dataset)
            Both test and validation datasets contain 50% of anomalies. They contain different types of anomalies to avoid overfitting, otherwise
            the results would be misleading.
            """
            # set the speed based on passed time
            if time_diff <= 51840 or time_diff > 86400:
                arm_speed_sent = arm_speed_default
                anomaly_state_sent = normal
            # First 36 mins of validation (normal data)
            elif 51840 < time_diff <= 54000:
                arm_speed_sent = arm_speed_default
                anomaly_state_sent = normal
            # Second 36 mins of validation data (10% increase in speed)
            elif 54000 < time_diff <= 56160:
                arm_speed_sent = arm_speed_10
                anomaly_state_sent = anomaly
            # Third 36 mins of validation data (normal)
            elif 56160 < time_diff <= 58320:
                arm_speed_sent = arm_speed_default
                anomaly_state_sent = normal
            # Fourth 36 mins of validation data (35% increase in speed)
            elif 58320 < time_diff <= 60480:
                arm_speed_sent = arm_speed_35
                anomaly_state_sent = anomaly
            # Fifth 36 mins of validation data (normal)
            elif 60480 < time_diff <= 62640:
                arm_speed_sent = arm_speed_default
                anomaly_state_sent = normal
            # Sixth 36 mins of validation data (65% increase in speed)    
            elif 62640 < time_diff <= 64800:
                arm_speed_sent = arm_speed_65
                anomaly_state_sent = anomaly
            # Seventh 36 mins of validation data (normal)
            elif 64800 < time_diff <= 66960:
                arm_speed_sent = arm_speed_default
                anomaly_state_sent = normal
            # Eighth (final) 36 mins of validation data (100% increase in speed)
            elif 66960 < time_diff <= 69120:
                arm_speed_sent = arm_speed_100
                anomaly_state_sent = anomaly
            # First 36 mins of test data (normal)
            elif 69120 < time_diff <= 71280:
                arm_speed_sent = arm_speed_default
                anomaly_state_sent = normal
            # Second 36 mins of test data (50% decrease in speed)
            elif 71280 < time_diff <= 73440:
                arm_speed_sent = arm_speed_slower_50
                anomaly_state_sent = anomaly
            # Third 36 mins of test data (normal)
            elif 73440 < time_diff <= 75600:
                arm_speed_sent = arm_speed_default
                anomaly_state_sent = normal
            # Fourth 36 mins of test data (5% decrease in speed) 
            elif 75600 < time_diff <= 77760:
                arm_speed_sent = arm_speed_slower_5
                anomaly_state_sent = anomaly
            # Fifth 36 mins of test data (normal)
            elif 77760 < time_diff <= 79920:
                arm_speed_sent = arm_speed_default
                anomaly_state_sent = normal
            # Sixth 36 mins of test data (20% increase in speed)
            elif 79920 < time_diff <= 82080:
                arm_speed_sent = arm_speed_20
                anomaly_state_sent = anomaly
            # Seventh 36 mins of test data (normal)
            elif 82080 < time_diff <= 84240:
                arm_speed_sent = arm_speed_default
                anomaly_state_sent = normal
            # Eight 36 mins of test data (25% decrease in speed)
            elif 84240 < time_diff <= 86400:
                arm_speed_sent = arm_speed_slower_25
                anomaly_state_sent = anomaly

            # we send the following for the right arm
            parameters_right_1 = [-0.13311, -0.44332, 0.48116, 1.21324, 1.20813, -1.21244, release_width, arm_speed_sent]
            parameters_right_2 = [-0.13214, -0.29779, 0.16533, -2.22504, -2.21765, -0.00223, release_width, arm_speed_sent]
            parameters_right_3 = [-0.13298, -0.47024, 0.09444, -2.22281, -2.21581, 0.00228, release_width, arm_speed_sent]
            parameters_right_4 = [0.04682, -0.46214, 0.03844, 2.13656, 2.20663, 0.07478, release_width, arm_speed_sent]
            parameters_right_4_grip= [0.04682, -0.46214, 0.03844, 2.13656, 2.20663, 0.07478, grip_width, arm_speed_sent]
            parameters_right_5 = [-0.4433, 0.1331, 0.48119, -2.21997, 0.00461, 2.21779, grip_width, arm_speed_sent]
            parameters_right_6 = [-0.04436, 0.33159, 0.09818, 2.20805, -2.22283, -0.10647, grip_width, arm_speed_sent]
            parameters_right_6_release = [-0.04436, 0.33159, 0.09818, 2.20805, -2.22283, -0.10647, release_width, arm_speed_sent]

            # Generate a dataset for right arm
            state_right_writer = csv.writer(state_right_csv, delimiter =',')
            state_right_writer.writerow([state_right.timestamp, state_right.actual_q, state_right.actual_qd, state_right.actual_current, 
                                state_right.actual_TCP_pose, state_right.actual_TCP_speed, state_right.actual_TCP_force, 
                                state_right.joint_temperatures, state_right.actual_execution_time, state_right.safety_status, 
                                state_right.actual_tool_accelerometer, state_right.actual_momentum, state_right.actual_robot_current, 
                                state_right.actual_joint_voltage, state_right.elbow_position, state_right.elbow_velocity, 
                                state_right.tool_output_current, state_right.tool_temperature, state_right.tcp_force_scalar, anomaly_state_sent])
            # Generate a dataset for left arm
            state_left_writer = csv.writer(state_left_csv, delimiter =',')
            state_left_writer.writerow([state_left.timestamp, state_left.actual_q, state_left.actual_qd, state_left.actual_current, 
                                state_left.actual_TCP_pose, state_left.actual_TCP_speed, state_left.actual_TCP_force, 
                                state_left.joint_temperatures, state_left.actual_execution_time, state_left.safety_status, 
                                state_left.actual_tool_accelerometer, state_left.actual_momentum, state_left.actual_robot_current, 
                                state_left.actual_joint_voltage, state_left.elbow_position, state_left.elbow_velocity, 
                                state_left.tool_output_current, state_left.tool_temperature, state_left.tcp_force_scalar, anomaly_state_sent])

            # send start signals to both arms, 1 for start, 0 for hold
            monitor_right.send("start", "input_int_register_0", 1)
            monitor_left.send("start", "input_int_register_0", 1)
            monitor_left.send("kick_left", "input_int_register_1", 0)
            
            print("The width value", state_left.output_double_register_1)

            if state_right.output_int_register_0 == 0:
                
                print("RIGHT ARM IS ACTIVE")
                
                # actual tcp position to compare
                actual_TCP_pose = [round(num, 1) for num in state_right.actual_TCP_pose] 
                actual_width_right = state_right.output_double_register_0
        
                # compare actual TCP to pre-determined points, define new waypoint based on the comparison
                if actual_TCP_pose == setp1_right_rounded and actual_width_right > 30:
                    new_setp_right = setp2_right
                    new_parameters = parameters_right_2
                    monitor_right.sendall("sent_parameters", new_parameters)
                elif actual_TCP_pose == setp2_right_rounded and actual_width_right > 30:
                    new_setp_right = setp3_right
                    new_parameters = parameters_right_3
                    monitor_right.sendall("sent_parameters", new_parameters)
                elif actual_TCP_pose == setp3_right_rounded and actual_width_right > 30:
                    new_setp_right = setp4_right
                    new_parameters = parameters_right_4
                    monitor_right.sendall("sent_parameters", new_parameters)
                elif actual_TCP_pose == setp4_right_rounded and actual_width_right > 30:
                    new_setp_right = setp4_right
                    new_parameters = parameters_right_4_grip
                    monitor_right.sendall("sent_parameters", new_parameters)
                elif actual_TCP_pose == setp4_right_rounded and actual_width_right < 30:
                    new_setp_right = setp5_right
                    new_parameters = parameters_right_5
                    monitor_right.sendall("sent_parameters", new_parameters)
                elif actual_TCP_pose == setp5_right_rounded and actual_width_right < 30:
                    new_setp_right = setp6_right
                    new_parameters = parameters_right_6
                    monitor_right.sendall("sent_parameters", new_parameters)
                elif actual_TCP_pose == setp6_right_rounded and actual_width_right < 30:
                    new_setp_right = setp6_right
                    new_parameters = parameters_right_6_release
                    monitor_right.sendall("sent_parameters", new_parameters)
                elif actual_TCP_pose == setp6_right_rounded and actual_width_right > 30:
                    new_setp_right = setp1_right
                    new_parameters = parameters_right_1
                    monitor_right.sendall("sent_parameters", new_parameters)        

            elif state_right.output_int_register_0 != 0:
                
                print("LEFT ARM IS ACTIVE")
                # enable robot program
                monitor_left.send("kick_left", "input_int_register_1", 1)
                # actual tcp position to compare
                actual_TCP_pose = [round(num, 1) for num in state_left.actual_TCP_pose]
                actual_width_left = state_left.output_double_register_0
                
                # compare actual TCP to pre-determined points, define new waypoint based on the comparison
                if actual_TCP_pose == setp1_left_rounded and actual_width_left > 30:
                    new_setp_left = setp2_left
                    new_parameters = parameters_left_2
                    monitor_left.sendall("sent_parameters", new_parameters)
                elif actual_TCP_pose == setp2_left_rounded and actual_width_left > 30:
                    new_setp_left = setp3_left
                    new_parameters = parameters_left_3
                    monitor_left.sendall("sent_parameters", new_parameters)
                elif actual_TCP_pose == setp3_left_rounded and actual_width_left > 30:
                    new_setp_left = setp4_left
                    new_parameters = parameters_left_4
                    monitor_left.sendall("sent_parameters", new_parameters)
                elif actual_TCP_pose == setp4_left_rounded and actual_width_left > 30:
                    new_setp_left = setp4_left
                    new_parameters = parameters_left_4_grip
                    monitor_left.sendall("sent_parameters", new_parameters)
                elif actual_TCP_pose == setp4_left_rounded and actual_width_left < 30:
                    new_setp_left = setp5_left
                    new_parameters = parameters_left_5
                    monitor_left.sendall("sent_parameters", new_parameters)
                elif actual_TCP_pose == setp5_left_rounded and actual_width_left < 30:
                    new_setp_left = setp6_left
                    new_parameters = parameters_left_6
                    monitor_left.sendall("sent_parameters", new_parameters)
                elif actual_TCP_pose == setp6_left_rounded and actual_width_left < 30:
                    new_setp_left = setp7_left
                    new_parameters = parameters_left_7
                    monitor_left.sendall("sent_parameters", new_parameters)
                elif actual_TCP_pose == setp7_left_rounded and actual_width_left < 30:
                    new_setp_left = setp7_left
                    new_parameters = parameters_left_7_release
                    monitor_left.sendall("sent_parameters", new_parameters)
                elif actual_TCP_pose == setp7_left_rounded and actual_width_left > 30:
                    new_setp_left = setp1_left
                    new_parameters = parameters_left_1
                    monitor_left.sendall("sent_parameters", new_parameters)                

        except KeyboardInterrupt:
            monitor_right.shutdown()
            monitor_left.shutdown()
