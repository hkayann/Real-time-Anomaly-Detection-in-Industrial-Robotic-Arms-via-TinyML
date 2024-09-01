# Copyright (c) 2020, Universal Robots A/S,
# All rights reserved.
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#    * Neither the name of the Universal Robots A/S nor the names of its
#      contributors may be used to endorse or promote products derived
#      from this software without specific prior written permission.
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL UNIVERSAL ROBOTS A/S BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

import sys
import logging
import rtde.rtde as rtde
import rtde.rtde_config as rtde_config
import xml.etree.ElementTree as ET
import csv
sys.path.append('..')

ROBOT_HOST = '192.168.0.20'
# ROBOT_HOST = '192.168.56.101'
ROBOT_PORT = 30004
config_filename = 'rtdeIO.xml'
RTDE_inputs = '/home/hkayan/Documents/universalRobots/Python_code/Remote_Examples/RTDE_Inputs.csv'
RTDE_outputs = '/home/hkayan/Documents/universalRobots/Python_code/Remote_Examples/RTDE_Outputs.csv'

logging.getLogger().setLevel(logging.INFO)

class RTDEConnect:
    _inputlist = RTDE_inputs
    _outputlist = RTDE_outputs

    def __init__(self, robot_ip, filename, frequency=20):
        self.robotIP = robot_ip
        self.port = 30004
        self.con = rtde.RTDE(self.robotIP, self.port)
        self.keep_running = True
        self.config = filename
        self.frequency = frequency
        self._rtdein = {}
        self._rtdeout = {}
        self.inputDict = {}
        self.outputDict = {}
        self.inputKeys = {}
        self.controlVersion = None
        self._rtdein, self._rtdeout = RTDEConnect._create_dicts(self._rtdein, self._rtdeout)
        self.programState = {
            0: 'Stopping',
            1: 'Stopped',
            2: 'Playing',
            3: 'Pausing',
            4: 'Paused',
            5: 'Resuming',
            6: 'Retracting'
        }
        self._initialize()

    def _initialize(self):
        self.con.connect()
        self.controlVersion = self.con.get_controller_version()
        if not self.controlVersion[0] == 5:
            print("Robot connected is not an E-series. Exiting...")
            sys.exit()
        if self.controlVersion[1] < 10:
            print("Current Polyscope software is below 5.10. Please update robot and run again. Exiting...")
            sys.exit()
        x = rtde_config.Recipe
        tree = ET.parse(self.config)
        root = tree.getroot()
        recipes = [x.parse(r) for r in root.findall('recipe')]

        # Iterate through all the recipe keys.
        for i in range(len(recipes)):
            # Check if recipe key's variables all exist as RTDE Inputs. If so, send the key as an input setup.
            if all(item in list(self._rtdein.keys()) for item in recipes[i].names):
                self.inputDict[recipes[i].key] = self.con.send_input_setup(recipes[i].names, recipes[i].types)
                # Add all input fields to their respective keys. Used for sending inputs to RTDE.
                self.inputKeys[recipes[i].key] = recipes[i].names
            # Check if recipe key's variables all exist as RTDE Outputs. If so, send the key as an output setup.
            elif all(item in list(self._rtdeout.keys()) for item in recipes[i].names):
                self.outputDict[tuple(recipes[i].names)] = self.con.send_output_setup(recipes[i].names, recipes[i].types, frequency=self.frequency)
            else:
                print(f'Error: {recipes[i].key} has a mix of inputs and outputs or has a variable that does not '
                      f'exist\nExiting...')
                sys.exit()

        if not self.con.send_start():
            print('Could not connect. Exiting...')
            sys.exit()

    def receive(self):
        """
        Receive a packet of data from RTDE at the frequency specified in instantiation of the Connector.
        :return: Packet of output data
        """
        return self.con.receive()

    def send(self, key, field, value):
        """
        Send RTDE inputs to the robot with a given list of fields and values.
        All fields within a key must be sent with an associated value.
        :param str key: the key to pull the corresponding inputs from.
        :param field: A list of fields (RTDE inputs) to send updated values to. Can also be a string value when
        specifying a single field.
        :param value: A list of updated input values to send to RTDE. Must match the order and type of
        the field parameter. Can be a single value in the case of sending a single field.
        """
        if type(field) is not list:
            self.inputDict[key].__dict__[field] = value
        else:
            for i in range(len(field)):
                self.inputDict[key].__dict__[field[i]] = value[i]
        self.con.send(self.inputDict[key])
        # return self.con.send(cmd)

    def sendall(self, key, value):
        """
        Send RTDE inputs to the robot with a given list of values. The order of values matches the recipe XML file
        for a given key.
        :param str key: The key to pull the corresponding inputs from.
        :param value: A list of updated input values to send to RTDE.
        """
        if type(value) is not list:
            self.inputDict[key].__dict__[self.inputKeys[key][0]] = value
        else:
            for i in range(len(value)):
                self.inputDict[key].__dict__[self.inputKeys[key][i]] = value[i]
        self.con.send(self.inputDict[key])

    def shutdown(self):
        """
        Safely disconnects from RTDE and shuts down.
        :return: None
        """
        self.con.send_pause()
        self.con.disconnect()

    @staticmethod
    def _csvparse(csvlist, parsed_dict):
        empty_lines = 0
        with open(csvlist, newline='') as csvfile:
            reader = csv.reader(csvfile, delimiter=',')
            for row in reader:
                if not ''.join(row).strip():
                    empty_lines += 1
                    continue
                parsed_dict[row[0]] = row[1]
        return parsed_dict

    @classmethod
    def _create_dicts(cls, input_dict, output_dict):
        inputs = cls._csvparse(cls._inputlist, input_dict)
        outputs = cls._csvparse(cls._outputlist, output_dict)
        return inputs, outputs


if __name__ == "__main__":
    state_monitor = RTDEConnect(ROBOT_HOST, config_filename)
    fields = ["input_int_register_0", "input_int_register_1", "input_int_register_2"]
    vals = [6, 4, 101]
    state_monitor.send("input2", fields, vals)
    # state_monitor.sendall("input2", vals)
    runtime_old = -1
    while state_monitor.keep_running:
        state = state_monitor.receive()

        if state is None:
            break
        if state.runtime_state != runtime_old:
            print(f'Robot program is {state_monitor.programState.get(state.runtime_state)}')
            runtime_old = state.runtime_state
