import RPi.GPIO as GPIO
import threading
from threading import Event, Timer
import time
import pyaudio
import wave
#=====================================================================================================================
#For audio
import sounddevice as sd
import soundfile as sf
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
DXL_MINIMUM_POSITION_VALUE  = 0         # Refer to the Minimum Position Limit of product eManual
DXL_MIDDLE_POSITION_VALUE   = 2048      # Refer to the Maximum Position Limit of product eManual
DXL_MAXIMUM_POSITION_VALUE  = 4095      # Refer to the Maximum Position Limit of product eManual

BAUDRATE                    = 57600

PROTOCOL_VERSION            = 2.0

DXL_ID                      = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11]

Motor_Position_Data         = {}
Motor_Position_Data_List    = []
Motor_read_list             = []

DEVICENAME                  = '/dev/ttyUSB0'

ENABLE               = 1
DISABLE              = 0
DXL_MOVING_STATUS_THRESHOLD = 20
VELOCITY_CONTROL_MODE = 1
POSITION_CONTROL_MODE = 3
REPLAY_COUNTER        = 0

index = 0
dxl_goal_position = [DXL_MINIMUM_POSITION_VALUE, DXL_MAXIMUM_POSITION_VALUE]
dxl_goal_velocity = 100

portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(PROTOCOL_VERSION)
#=====================================================================================================================
#For Button setup
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
for i in DXL_ID:
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, i, ADDR_LED_ENABLE, ENABLE)
    
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))

    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, i, ADDR_OPERATING_MODE, POSITION_CONTROL_MODE)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))

print("LED Enabled! ")
print("Dynamixel has been successfully connected")
print("ALL MOTOR SET TO VELOCITY MODE! ")

# Timer to manage the repeat thread
class RepeatingTimer(Timer): 
    def run(self):
        self.finished.wait(self.interval)
        while not self.finished.is_set():
            self.function(*self.args, **self.kwargs)
            self.finished.wait(self.interval)

def record_movement():
    Motor_Position_Data = {}
    for i in DXL_ID:
        Motor_Position_Data[f'M{i}'], dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, i, ADDR_PRESENT_POSITION)
        
        if (Motor_Position_Data[f'M{i}'] > 4095) and (Motor_Position_Data[f'M{i}'] < 4200000000):
           Motor_Position_Data[f'M{i}'] = Motor_Position_Data[f'M{i}'] - 4095
        if Motor_Position_Data[f'M{i}'] > 4200000000:
            Motor_Position_Data[f'M{i}'] = Motor_Position_Data[f'M{i}'] - 4294967295 + 4095
    Motor_Position_Data_List.append(Motor_Position_Data)
    

def back_origin():
    print("Back to origin...")
    with open("Motor_Position.json") as openfile:
        Motor_move_back_list = json.load(openfile)
        
    for j in DXL_ID:
        packetHandler.write1ByteTxRx(portHandler, j, ADDR_TORQUE_ENABLE, ENABLE)
        packetHandler.write4ByteTxRx(portHandler, j, ADDR_GOAL_POSITION, Motor_move_back_list[0][f"M{j}"])
        print(Motor_move_back_list[0][f"M{j}"])
        
        if dxl_comm_result != COMM_SUCCESS:
            print(f"M{j}: ")
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print(f"M{j}: ")
            print("%s" % packetHandler.getRxPacketError(dxl_error))
        else:
            print(f"M{j}: Write success~~")

def replay_movement():
    global replaying
    for item in Motor_read_list:
        for i in DXL_ID:
            packetHandler.write4ByteTxRx(portHandler, i, ADDR_GOAL_POSITION, item[f'M{i}'])
            time.sleep(0.02)
    
    replaying = False

    for i in DXL_ID:
        dxl_comm_result, dxl_error = packetHandler.reboot(portHandler, i)
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, i, ADDR_TORQUE_ENABLE, DISABLE)
    print("End Replaying")

def record_audio(audio_path, input_device_index = None):
    global recording
    FRAMES_PER_BUFFER = 4096
    FORMAT = pyaudio.paInt16
    CHANNELS = 1
    RATE = 48000
    p = pyaudio.PyAudio()
    
    stream = p.open(
                        format              =   FORMAT,
                        channels            =   CHANNELS,
                        rate                =   RATE,
                        input               =   True,
                        frames_per_buffer   =   FRAMES_PER_BUFFER,
                    )

    print("Start recording audio...")
    frames = []
    
    while recording:
        data = stream.read(FRAMES_PER_BUFFER)
        frames.append(data)
        
        # Check for event to stop recording
        if not recording:
            break

    print("Stop recording audio!")
    
    stream.stop_stream()
    stream.close()
    p.terminate()

    obj = wave.open(audio_path, "wb")
    obj.setnchannels(CHANNELS)
    obj.setsampwidth(p.get_sample_size(FORMAT))
    obj.setframerate(RATE)
    obj.writeframes(b"".join(frames))
    obj.close()
        
    print("Audio stored...")
    return

def increase_volume(audio, factor):
    return audio * factor

def replay_audio(filename, volume_factor):
    sd.default.device = 3

    data, fs = sf.read(filename, dtype='float32')
    data = increase_volume(data, volume_factor)

    sd.play(data, fs)
    status = sd.wait()

try:
    while True:
        record_btn_state = GPIO.input(11)
        replay_btn_state = GPIO.input(13)
        
        if record_btn_state == False:
            print('Record Button Pressed')
            
            if recording == False:
                recording = True
                
                #Clear motor position data list to rewrite
                Motor_Position_Data_List.clear()
                
                #Start to recording movement
                recording_movement = RepeatingTimer(0.00001, record_movement)
                recording_movement.start()
                print("Recording Movement...")
                
                #Start to recording audio
                Record_Audio_Thread = threading.Thread(target=record_audio, args=('output.wav', 1))
                Record_Audio_Thread.start()
                print("Recording Audio...")
                
            elif recording == True:
                #Ending Record
                recording = False
                recording_movement.cancel()
                print(f"Thread is alive: {recording_movement.is_alive()}")
                
                #Store motor position data into json
                json_position_data = json.dumps(Motor_Position_Data_List, indent=4)
                with open("Motor_Position.json", "w") as outfile:
                    outfile.write(json_position_data)
                
                print("End Recording...")
                
            time.sleep(0.5)            

        if replay_btn_state == False:
            print('Replay Button Pressed')
            
            if replaying == False:
                replaying = True
                REPLAY_COUNTER = 0
                back_origin()
                print("Replaying Movement...")
                
                with open("Motor_Position.json") as openfile:
                    Motor_read_list = json.load(openfile)

                Replay_Movement_Thread = threading.Thread(target=replay_movement,)
                Replay_Movement_Thread.start()
                
                Replay_Audio_Thread = threading.Thread(target=replay_audio, args=('output.wav', 7.0))
                Replay_Audio_Thread.start()
                Replay_Audio_Thread.join()
                
                
            elif replaying == True:
                #Ending Replay
                replaying = False
                Replay_Movement_Thread.cancel()
                
                for i in DXL_ID:
                    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, i, ADDR_TORQUE_ENABLE, DISABLE)
                
                print("End Replaying")
    
            time.sleep(0.5)

except KeyboardInterrupt:
    # Disable Dynamixel
    for i in DXL_ID:
        dxl_comm_result, dxl_error = packetHandler.reboot(portHandler, i)
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, i, ADDR_TORQUE_ENABLE, DISABLE)
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, i, ADDR_LED_ENABLE, DISABLE)

    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    # Close port
    portHandler.closePort()