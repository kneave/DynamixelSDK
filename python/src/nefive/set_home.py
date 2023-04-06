#!/usr/bin/env python
# -*- coding: utf-8 -*-

#*******************************************************************************
# Copyright 2017 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#*******************************************************************************


#*******************************************************************************
#***********************     Read and Write Example      ***********************
#  Required Environment to run this example :
#    - Protocol 2.0 supported DYNAMIXEL(X, P, PRO/PRO(A), MX 2.0 series)
#    - DYNAMIXEL Starter Set (U2D2, U2D2 PHB, 12V SMPS)
#  How to use the example :
#    - Select the DYNAMIXEL in use at the MY_DXL in the example code. 
#    - Build and Run from proper architecture subdirectory.
#    - For ARM based SBCs such as Raspberry Pi, use linux_sbc subdirectory to build and run.
#    - https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_sdk/overview/
#  Author: Ryu Woon Jung (Leon)
#  Maintainer : Zerom, Will Son
# *******************************************************************************

import os
from jointlist import servos
import time
from datetime import datetime

if os.name == 'nt':
    import msvcrt
    def getch():
        return msvcrt.getch().decode()
else:
    import sys, tty, termios
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    def getch():
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

from dynamixel_sdk import * # Uses Dynamixel SDK library

# Control table address
ADDR_HOMING_OFFSET          = 20
ADDR_TORQUE_ENABLE          = 64
ADDR_LED_RED                = 65
ADDR_ERROR_STATUS           = 70
ADDR_GOAL_POSITION          = 116
ADDR_PRESENT_POSITION       = 132
ADDR_PRESENT_TEMPERATURE    = 146
BAUDRATE                    = 1000000

# DYNAMIXEL Protocol Version (1.0 / 2.0)
# https://emanual.robotis.com/docs/en/dxl/protocol2/
PROTOCOL_VERSION            = 2.0

# Use the actual port assigned to the U2D2.
# ex) Windows: "COM*", Linux: "/dev/ttyUSB*", Mac: "/dev/tty.usbserial-*"
# DEVICENAME                  = 'COM4'
DEVICENAME                  = '/dev/ttyUSB0'

TORQUE_ENABLE               = 1     # Value for enabling the torque
TORQUE_DISABLE              = 0     # Value for disabling the torque
LED_ENABLE                  = 1     # Value for enabling the torque
LED_DISABLE                 = 0     # Value for disabling the torque

# home_positions =  {100: -11.35, 101: 295.33, 102: 110.53, 103: 79.9, 104: 69.78, 105: 110.7, 106: 371.89, 107: 74.27,
#                    108: 337.3, 109: 498.96, 110: -346.19, 111: 313.19, 112: 353.58, 113: 256.43, 114: 157.87, 115: -27.28}

home_positions = \
    {100: 177.67, 101: 256.52, 102: 111.94, 103: 55.97, 104: 15.05, 105: 97.24, 106: 347.34, 107: 97.59, 
     108: 342.76, 109: 134.64, 110: -10.47, 111: 295.5, 112: 354.29, 113: 266.64, 114: 194.39, 115: 15.58}

# Initialize PortHandler instance
# Set the port path
# Get methods and members of PortHandlerLinux or PortHandlerWindows
portHandler = PortHandler(DEVICENAME)

# Initialize PacketHandler instance
# Set the protocol version
# Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
packetHandler = PacketHandler(PROTOCOL_VERSION)

def setAllAngles(angles):
    positions = {}
    for servo_name, servo_id in servos.items():
        positions[servo_id] = angleToPosition(angles[servo_id])

    setAllPositions(positions)
def setAllPositions(positions):
    groupSyncWrite = GroupSyncWrite(portHandler, packetHandler, ADDR_GOAL_POSITION, 4)
    for servo_name, servo_id in servos.items():
        print(f"setting {servo_id} to {positions[servo_id]}")
        position_array = positions[servo_id].to_bytes(4, "little", signed=True)
        groupSyncWrite.addParam(servo_id, position_array)

    dxl_comm_result = groupSyncWrite.txPacket()
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))

    # Clear syncwrite parameter storage
    groupSyncWrite.clearParam()

def readAllAngles():
    angles = {}
    for servo_name, servo_id in servos.items():
        angles[servo_id] = getCurrentAngle(servo_id)
    return angles

def readAllPositions():
    groupSyncRead = GroupSyncRead(portHandler, packetHandler, ADDR_PRESENT_POSITION, 4)
    for servo_name, servo_id in servos.items():
        dxl_addparam_result = groupSyncRead.addParam(servo_id)

    dxl_comm_result = groupSyncRead.txRxPacket()
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))

    present_position = {}
    for servo_name, servo_id in servos.items():
        dxl_getdata_result = groupSyncRead.isAvailable(servo_id, ADDR_PRESENT_POSITION, 4)
        if dxl_getdata_result != True:
            print("[ID:%03d] groupSyncRead getdata failed" % servo_id)

        present_position[servo_id] = positionToAngle(groupSyncRead.getData(servo_id, ADDR_PRESENT_POSITION, 4))

    return present_position


def pingServos():
    # Try to broadcast ping the Dynamixel
    dxl_data_list, dxl_comm_result = packetHandler.broadcastPing(portHandler)
    if dxl_comm_result != COMM_SUCCESS:
        print("Error pinging: %s" % packetHandler.getTxRxResult(dxl_comm_result))

    detected = []
    for dxl_id in dxl_data_list:
        detected.append(dxl_id)

    return detected


# dxl_addparam_result = groupSyncWrite.addParam(DXL1_ID, param_goal_position)

# Open port
if portHandler.openPort():
    print("Succeeded to open the port")
else:
    print("Failed to open the port")
    print("Press any key to terminate...")
    getch()
    quit()


# Set port baudrate
if portHandler.setBaudRate(BAUDRATE):
    print("Succeeded to change the baudrate")
else:
    print("Failed to change the baudrate")
    print("Press any key to terminate...")
    getch()
    quit()

def setLED(id, value):
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, id, ADDR_LED_RED, value)
    if dxl_comm_result != COMM_SUCCESS:
        print(f"setLED for id {id}: {packetHandler.getTxRxResult(dxl_comm_result)}")
    elif dxl_error != 0:
        print(f"setLED for id {id}: {packetHandler.getRxPacketError(dxl_error)}")

def getCurrentPosition(servo_id):
    position, result, error = \
        packetHandler.readTxRx(portHandler, servo_id, ADDR_PRESENT_POSITION, 4)
    if result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(result))
        return
    elif error != 0:
        print("%s" % packetHandler.getRxPacketError(error))
        return
    data_array = bytes([position[0], position[1], position[2], position[3]])
    return int.from_bytes(data_array, "little", signed=True)


def getCurrentAngle(servo_id):
    position_signed = getCurrentPosition(servo_id)
    current_angle = position_signed * 0.088
    return round(current_angle, 2)


def readAllTemperatures():
    groupSyncRead = GroupSyncRead(portHandler, packetHandler, ADDR_PRESENT_TEMPERATURE, 1)
    for servo_name, servo_id in servos.items():
        dxl_addparam_result = groupSyncRead.addParam(servo_id)

    dxl_comm_result = groupSyncRead.txRxPacket()
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))

    present_temperature = {}
    for servo_name, servo_id in servos.items():
        dxl_getdata_result = groupSyncRead.isAvailable(servo_id, ADDR_PRESENT_TEMPERATURE, 1)
        if dxl_getdata_result != True:
            print("[ID:%03d] groupSyncRead getdata failed" % servo_id)
            present_temperature[servo_id] = None
        else:
            present_temperature[servo_id] = int(groupSyncRead.getData(servo_id, ADDR_PRESENT_TEMPERATURE, 1))

    return present_temperature

def readAllHardwareStatus():
    groupSyncRead = GroupSyncRead(portHandler, packetHandler, ADDR_ERROR_STATUS, 1)
    for servo_name, servo_id in servos.items():
        dxl_addparam_result = groupSyncRead.addParam(servo_id)

    dxl_comm_result = groupSyncRead.txRxPacket()
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))

    present_state = {}
    for servo_name, servo_id in servos.items():
        dxl_getdata_result = groupSyncRead.isAvailable(servo_id, ADDR_ERROR_STATUS, 1)
        if dxl_getdata_result != True:
            print("[ID:%03d] groupSyncRead getdata failed" % servo_id)
            present_state[servo_id] = None
        else:
            present_state[servo_id] = int(groupSyncRead.getData(servo_id, ADDR_ERROR_STATUS, 1))

    return present_state

def positionToAngle(position):
    current_angle = position * 0.088
    return round(current_angle, 2)

def angleToPosition(angle):
    # convert angle to position value, note this is float to int
    position = int(angle / 0.088)
    # print(f"{angle} angle converted to position {position}")
    return position


def setAngle(servo_id, angle):
    position = angleToPosition(angle);
    # print(position)

    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(
        portHandler, servo_id, ADDR_GOAL_POSITION, position)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))

def setHomingOffset(servo_id, angle):
    # convert angle to position value, note this is float to int
    position = int(angle / 0.088)

    # position to byte array
    position_array = position.to_bytes(4, "little", signed=True)

    # send to servo_id
    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, servo_id, ADDR_HOMING_OFFSET, position_array)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
def setServoTorque(servo_id, value):
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, servo_id, ADDR_TORQUE_ENABLE, value)
    if dxl_comm_result != COMM_SUCCESS:
        print("SetServo: %s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("SetServo: %s" % packetHandler.getRxPacketError(dxl_error))

# Disable all LEDs and servos
def disableAllServos():
    for servo_name, servo_id in servos.items():
        setServoTorque(servo_id, 0)
        setLED(servo_id, 0)

def enableAllServos():
    for servo_name, servo_id in servos.items():
        setServoTorque(servo_id, 1)
        setLED(servo_id, 1)



print(f"Press any key to move to home positions or ESC to quit")
if getch() == chr(0x1b):
    exit()

def lerp(start, end, ratio):
    return round((ratio * end) + ((1 - ratio) * start), 1)

def moveToHome():
    enableAllServos()

    start_angles = readAllAngles()

    print("start positions are:")
    print(start_angles)

    for i in range(100):
        angles = {}
        for servo_name, servo_id in servos.items():
            value = lerp(start_angles[servo_id], home_positions[servo_id], i / 100)
            if value == None:
                print(f"Unable to set position for {servo_name}")
                continue
            angles[servo_id] = value
        print(angles)
        setAllAngles(angles)
        print(i)
        time.sleep(0.005)

    start_angles = {}
    for servo_name, servo_id in servos.items():
        start_angles[servo_id] = getCurrentAngle(servo_id)

    print(start_angles)

    print("press any key to disable servos and exit")
    getch()

    readAllTemperatures()

    disableAllServos()

def getHomePositions():
    home_values = {}

    # Iterate through all servos, setting home location
    for servo_name, servo_id in servos.items():
        print(f"[{servo_name}] please move to home position and press any key to continue or ESC to quit")
        if getch() == chr(0x1b):
            break

        # Read present position
        angle = getCurrentAngle(servo_id)
        setServoTorque(servo_id, 1)
        setLED(servo_id, 1)
        home_values[servo_id] = angle

    print(f"Home positions: {home_values}")


    print("press any key to disable servos and exit")
    getch()

    disableAllServos()
    # Close port

def loopReadTemperature():
    log_file = open("temp_log.txt", "at")

    # Just print all the positions/temperatures
    while 1:
        current_time = datetime.now()
        line = "[{0}:{1}:{2}] : {3}\n".format(current_time.hour,
                                              current_time.minute,
                                              current_time.second,
                                              readAllTemperatures())
        print(line)
        log_file.write(line)

        time.sleep(4)
        # print(readAllPositions())

    log_file.close()


def get_key():
    first_char = getch()
    if first_char == '\x1b':
        return {'[A': 'up', '[B': 'down', '[C': 'right', '[D': 'left'}[getch() + getch()]
    else:
        return first_char

error_states = readAllHardwareStatus()
for servo_name, servo_id in servos.items():
    if not error_states[servo_id] == 0:
        print(f"[{servo_id}]: {error_states[servo_id]}")

        dxl_comm_result, dxl_error = packetHandler.reboot(portHandler, servo_id)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))

        print("[ID:%03d] reboot Succeeded\n" % servo_id)

# disableAllServos()
# getHomePositions()
# moveToHome()

import _thread


def input_thread(a_list):
    key = get_key()

    if key == 'q':
        exit()
    elif key == 'd':
        a_list.append(True)


def loopUntilKeyPress(servo_id):
    a_list = []
    current_position = None
    _thread.start_new_thread(input_thread, (a_list,))

    while not a_list:
        current_position = getCurrentPosition(servo_id)
        print(f"Value: {current_position}", end="\r")

    return current_position


detected = pingServos()
servos_present = 0
for servo_name, servo_id in servos.items():
    if servo_id not in detected:
        print(f"Servo {servo_id} was not detected")
    else:
        servos_present += 1

if servos_present == len(servos.items()):
    print("All servos detected")
else:
    print("Servos missing, check log and resolve.")
    print("Exiting")
    exit()

disableAllServos()
portHandler.closePort()
