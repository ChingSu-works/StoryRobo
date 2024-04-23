import RPi.GPIO as GPIO
import threading
from threading import Event, Timer
import time
import pyaudio
import wave
#=====================================================================================================================
#For playsound
from playsound import playsound

#=====================================================================================================================
#For dynamixel
import json
import os

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
ADDR_TORQUE_ENABLE          = 64
ADDR_LED_ENABLE             = 65
ADDR_GOAL_POSITION          = 116
ADDR_GOAL_VELOCITY          = 104
ADDR_PRESENT_VELOCITY       = 128
ADDR_PRESENT_POSITION       = 132
DXL_MINIMUM_POSITION_VALUE  = 1743         # Refer to the Minimum Position Limit of product eManual
DXL_MIDDLE_POSITION_VALUE   = 2048      # Refer to the Maximum Position Limit of product eManual
DXL_MAXIMUM_POSITION_VALUE  = 2442      # Refer to the Maximum Position Limit of product eManual

BAUDRATE                    = 57600

PROTOCOL_VERSION            = 2.0

DXL_ID                      = 1

DEVICENAME                  = '/dev/ttyUSB0'

ENABLE               = 1
DISABLE              = 0
DXL_MOVING_STATUS_THRESHOLD = 20
VELOCITY_CONTROL_MODE = 1
POSITION_CONTROL_MODE = 3

index = 0
dxl_goal_position = [DXL_MINIMUM_POSITION_VALUE, DXL_MAXIMUM_POSITION_VALUE]
dxl_goal_velocity = 100

portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(PROTOCOL_VERSION)

#=====================================================================================================================
replay_pin = 11
record_pin = 13

GPIO.setmode(GPIO.BOARD)
GPIO.setup(replay_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(record_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)

system_state = "NORMAL"
recording = False
replaying = False
back_to_origin = True

ButtonConter = 0

# # Audio paramater setup for pyaudio
FRAMES_PER_BUFFER = 3200
FORMAT = pyaudio.paInt16
CHANNELS = 1
RATE = 48000
p = pyaudio.PyAudio()
seconds = 10
frames = []

#=====================================================================================================================
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

# Enable LED
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_LED_ENABLE, ENABLE)

if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_OPERATING_MODE, POSITION_CONTROL_MODE)
if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))
    
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_TORQUE_ENABLE, ENABLE)
if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))

print("LED Enabled! ")
print("Dynamixel has been successfully connected")
print("ALL MOTOR SET TO POSITION MODE! ")

dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL_ID, ADDR_GOAL_POSITION, 2443)
if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))
else:
    print("Write success~~")