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

import os, ctypes
from jointlist import servos
from wake_up import wakeup_angles
import time
from datetime import datetime
import signal


def exit_handler(signum, frame):
    print("ctrl-c pressed, exiting")
    disableAllServos()
    sys.exit()
    
signal.signal(signal.SIGINT, exit_handler)

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
ADDR_OPERATING_MODE         = 11
ADDR_HOMING_OFFSET          = 20
ADDR_TORQUE_ENABLE          = 64
ADDR_LED_RED                = 65
ADDR_ERROR_STATUS           = 70
ADDR_GOAL_CURRENT           = 102
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
DEVICENAME                  = '/dev/ttyUSB1'

TORQUE_ENABLE               = 1     # Value for enabling the torque
TORQUE_DISABLE              = 0     # Value for disabling the torque
LED_ENABLE                  = 1     # Value for enabling the torque
LED_DISABLE                 = 0     # Value for disabling the torque

# home_positions =  {100: -11.35, 101: 295.33, 102: 110.53, 103: 79.9, 104: 69.78, 105: 110.7, 106: 371.89, 107: 74.27,
#                    108: 337.3, 109: 498.96, 110: -346.19, 111: 313.19, 112: 353.58, 113: 256.43, 114: 157.87, 115: -27.28}

home_positions = {  
    100: 182.51, 101: 116.69, 102: 190.43, 103: 220.88, 104: 189.9, 105: 193.42, 106: 177.23, 107: 141.77, 
    108: 177.85, 109: 89.58, 110: 173.54, 111: 138.16, 112: 179.26, 113: 175.91, 114: 196.86, 115: 226.78,
    116: 173.8, 117: 179.17, 118: 178.29}

current_position_current_limits = {  
    100: 100, 101: 50, 102: 75, 103: 150, 104: 70, 105: 40, 106: 50, 107: 50, 
    108: 100, 109: 50, 110: 75, 111: 150, 112: 70, 113: 40, 114: 50, 115: 50,
    116: 100, 117: 50, 118: 60}

position_mode_current_limits = {  
    100: 910, 101: 910, 102: 910, 103: 910, 104: 1700, 105: 1700, 106: 1700, 107: 1700, 
    108: 910, 109: 910, 110: 910, 111: 910, 112: 1700, 113: 1700, 114: 1700, 115: 1700,
    116: 1700, 117: 1700, 118: 1700}

    # {100: 186.38, 101: 257.58, 102: 101.29, 103: 59.4, 104: -2.73, 105: 104.63, 106: 348.13, 107: 76.82, 
    #  108: 345.58, 109: 147.84, 110: -17.34, 111: 294.89, 112: 348.92, 113: 265.32, 114: 192.81, 115: 7.57}


servo_limits = {100: [1512, 3652], 101: [-967, 4917], 102: [-610, 4350], 103: [-276, 2618], 104: [-824, 5978], 105: [918, 3203], 106: [1087, 3179], 107: [1380, 2497], 108: [1040, 3685], 109: [-1254, 5105], 110: [-39, 4860], 111: [1565, 4156], 112: [-1172, 6545], 113: [999, 3103], 114: [1036, 3085], 115: [1587, 2673]}

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
        # print(f"setting {servo_id} to {positions[servo_id]}")
        position_array = positions[servo_id].to_bytes(4, "little", signed=True)
        groupSyncWrite.addParam(servo_id, position_array)

    dxl_comm_result = groupSyncWrite.txPacket()
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))

    # Clear syncwrite parameter storage
    groupSyncWrite.clearParam()

def readAllAngles():
    angles = {}
    positions = readAllPositions()
    for servo_name, servo_id in servos.items():
        angles[servo_id] = positionToAngle(positions[servo_id])
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


        position = groupSyncRead.getData(servo_id, ADDR_PRESENT_POSITION, 4)
        present_position[servo_id] = positionToSigned(position)

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


def setOperatingMode(id, value):
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, id, ADDR_OPERATING_MODE, value)
    if dxl_comm_result != COMM_SUCCESS:
        print(f"setOpMode for id {id}: {packetHandler.getTxRxResult(dxl_comm_result)}")
    elif dxl_error != 0:
        print(f"setOpMode for id {id}: {packetHandler.getRxPacketError(dxl_error)}")


def setCurrentGoal(id, value):
    dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, id, ADDR_GOAL_CURRENT, value)
    if dxl_comm_result != COMM_SUCCESS:
        print(f"setCurrentGoal for id {id}: {packetHandler.getTxRxResult(dxl_comm_result)}")
    elif dxl_error != 0:
        print(f"setCurrentGoal for id {id}: {packetHandler.getRxPacketError(dxl_error)}")


def setLED(id, value):
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, id, ADDR_LED_RED, value)
    if dxl_comm_result != COMM_SUCCESS:
        print(f"setLED for id {id}: {packetHandler.getTxRxResult(dxl_comm_result)}")
    elif dxl_error != 0:
        print(f"setLED for id {id}: {packetHandler.getRxPacketError(dxl_error)}")


def positionToSigned(position):
    if position > 2147483647:
        position = position - 4294967296

    return position


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
    position = angleToPosition(angle)
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

# set all servos to current-position mode
def setOperatingModes(value):
    for servo_name, servo_id in servos.items():
        setServoTorque(servo_id, 0)
        setOperatingMode(servo_id, value)

        if value == 5:
            setCurrentGoal(servo_id, current_position_current_limits[servo_id])

        if value == 3:
            setCurrentGoal(servo_id, position_mode_current_limits[servo_id])

        setServoTorque(servo_id, 1)


# Disable all LEDs and servos
def disableAllServos():
    for servo_name, servo_id in servos.items():
        setServoTorque(servo_id, 0)
        setLED(servo_id, 0)


def enableAllServos():
    for servo_name, servo_id in servos.items():
        setServoTorque(servo_id, 1)
        setLED(servo_id, 1)


def lerp(start, end, ratio):
    return round((ratio * end) + ((1 - ratio) * start), 1)


def lerpToAngles(angles, lerp_time):
    start_angles = readAllAngles()
    interval = lerp_time / 100
    print(f"Lerp interval: {interval}")
    for i in range(100):
        new_angles = {}
        for servo_name, servo_id in servos.items():
            value = lerp(start_angles[servo_id], angles[servo_id], i / 100)
            if value == None:
                print(f"Unable to set position for {servo_name}")
                continue
            new_angles[servo_id] = value
        setAllAngles(new_angles)
        
        time.sleep(interval)


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


def learnPositions():
    positions_in_order = []

    while True:
        print(f"[{servo_name}] please move to new position and press any key to continue or ESC to quit")
        if getch() == chr(0x1b):
            break

        # Read present positions
        currentPositions = readAllAngles()
        setAllAngles(currentPositions)
        positions_in_order.append(currentPositions)

    print(f"Home positions: {positions_in_order}")

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


def getLimits():
    servo_limits = {}
    min_value = 0
    max_value = 1

    for servo_name, servo_id in servos.items():
        # each tuple is {min_value, max_value}
        servo_limits[servo_id] = [ 100000, -100000 ]

    while 1:
        current_positions = readAllPositions()
        for servo_name, servo_id in servos.items():
            if current_positions[servo_id] >= servo_limits[servo_id][max_value]:
                servo_limits[servo_id][max_value] = current_positions[servo_id]
                
            if current_positions[servo_id] <= servo_limits[servo_id][min_value]:
                servo_limits[servo_id][min_value] = current_positions[servo_id]

        print(f"{servo_limits}")


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

disableAllServos()
# getHomePositions()

# moveToHome()

# getLimits()


# loopReadTemperature()

# print(f"Angles: {readAllAngles()}")
# print(f"Positions: {readAllPositions()}")
# print(f"Errors: {readAllHardwareStatus()}")
def learnSomeMoves():
    setOperatingModes(5)
    learnPositions()

setOperatingModes(3)
for angles in wakeup_angles:
    print(angles)
    lerpToAngles(angles, 1)
    # time.sleep(1)

getch()

disableAllServos()
portHandler.closePort()
