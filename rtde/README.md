## File Explanations

| File Name                         | Description                                                                                      |
|-----------------------------------|--------------------------------------------------------------------------------------------------|
| `connector.py`                    | Sets up the IP addresses and sampling rate for the RTDE interface, enabling communication between the Universal Robot and the external control system. |
| `control_loop_connector_v1.py`    | Configures and controls the robotic arm’s position, joint velocity, and joint acceleration values during the execution of the control loop. |

## How to Use RTDE with These Files

To trigger and use these RTDE scripts, follow these steps:

1. **Set Up the Connector:**
   - First, configure the IP addresses and sampling rate in `connector.py`. This step establishes the communication parameters necessary for the RTDE interface to connect with the robot.

2. **Control the Robot:**
   - Next, run `control_loop_connector_v1.py` using the command `python3 control_loop_connector_v1.py` to set and control the robotic arm's key parameters, such as position, joint velocity, and acceleration. This script is crucial for executing precise control over the robot during operations.

### **Notes:**
- These files are integral for establishing a reliable connection with the robot and controlling its movement in real-time. Ensure that both scripts are correctly configured before initiating any control processes.