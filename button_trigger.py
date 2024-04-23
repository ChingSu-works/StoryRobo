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
DXL_MINIMUM_POSITION_VALUE  = 0         # Refer to the Minimum Position Limit of product eManual
DXL_MIDDLE_POSITION_VALUE   = 2048      # Refer to the Maximum Position Limit of product eManual
DXL_MAXIMUM_POSITION_VALUE  = 4095      # Refer to the Maximum Position Limit of product eManual

BAUDRATE                    = 57600

PROTOCOL_VERSION            = 2.0

DXL_ID                      = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11]

Motor_Position_Data  = {}
Motor_Position_Data_List             = []

DEVICENAME                  = '/dev/ttyUSB0'

ENABLE               = 1
DISABLE              = 0
DXL_MOVING_STATUS_THRESHOLD = 20
VELOCITY_CONTROL_MODE = 1
POSITION_CONTROL_MODE = 3
Replay_Counter = 0

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
for i in DXL_ID:
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, i, ADDR_LED_ENABLE, ENABLE)
    
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))

    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, i, ADDR_OPERATING_MODE, VELOCITY_CONTROL_MODE)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))

print("LED Enabled! ")
print("Dynamixel has been successfully connected")
print("ALL MOTOR SET TO VELOCITY MODE! ")
#=====================================================================================================================
class RepeatingTimer(Timer): 
    def run(self):
        self.finished.wait(self.interval)
        while not self.finished.is_set():
            self.function(*self.args, **self.kwargs)
            self.finished.wait(self.interval)
            
def print_hello():
     print("hello world")

# Record the movement of robot
def record_movement_through_time():
    global Motor_Position_Data
    global Motor_Position_Data_List

    for i in DXL_ID:
        Motor_Position_Data[f'M{i}'], dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, i, ADDR_PRESENT_POSITION)
        
        if (Motor_Position_Data[f'M{i}'] > 4095) and (Motor_Position_Data[f'M{i}'] < 4200000000):
           Motor_Position_Data[f'M{i}'] = Motor_Position_Data[f'M{i}'] - 4095
        if Motor_Position_Data[f'M{i}'] > 4200000000:
            Motor_Position_Data[f'M{i}'] = Motor_Position_Data[f'M{i}'] - 4294967295 + 4095
    
    Motor_Position_Data_List.append(Motor_Position_Data)
    # json_position_data = json.dumps(Motor_Position_Data_List, indent=4)
        
    # with open("Motor_Position.json", "w") as outfile:
    #     outfile.write(json_position_data)

def record_movement():
    Motor_Position_Data = {}
    Motor_Position_Data_List.clear()
    
    # Change to Position control mode
    for i in DXL_ID:
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, i, ADDR_OPERATING_MODE, VELOCITY_CONTROL_MODE)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
    print("ALL MOTOR SET TO VELOCITY MODE! ")
        
    
    while recording == True:
        print("Start recording movement.....",end='\r')
        ## Read present position
        Motor_Position_Data  = {}
        
        for i in DXL_ID:
            Motor_Position_Data[f'M{i}'], dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, i, ADDR_PRESENT_POSITION)
            
            if (Motor_Position_Data[f'M{i}'] > 4095) and (Motor_Position_Data[f'M{i}'] < 4200000000):
               Motor_Position_Data[f'M{i}'] = Motor_Position_Data[f'M{i}'] - 4095
            if Motor_Position_Data[f'M{i}'] > 4200000000:
                Motor_Position_Data[f'M{i}'] = Motor_Position_Data[f'M{i}'] - 4294967295 + 4095
        time.sleep(0.01)
        
        Motor_Position_Data_List.append(Motor_Position_Data)
        json_position_data = json.dumps(Motor_Position_Data_List, indent=4)
        
    with open("Motor_Position.json", "w") as outfile:
        outfile.write(json_position_data)
        
    print("Interrupt recording movement")
    print("Movement stored")
    sys.exit()

def back_origin():
    with open("Motor_Position.json") as openfile:
        Motor_move_back_list = json.load(openfile)
        
    Motor_checklist = {"M0": False, "M1": False, "M2": False, "M3": False,
                       "M4": False, "M5": False, "M6": False, "M7": False, 
                       "M8": False, "M9": False, "M10": False, "M11": False}
    
    # try:
    while True:
        print("Motor Now value: " + str(Motor_checklist))
        
        if all(value == True for value in Motor_checklist.values()):
            for i in DXL_ID:
                packetHandler.write1ByteTxRx(portHandler, i, ADDR_TORQUE_ENABLE, DISABLE)
                packetHandler.write1ByteTxRx(portHandler, i, ADDR_OPERATING_MODE, POSITION_CONTROL_MODE)
                packetHandler.write1ByteTxRx(portHandler, i, ADDR_TORQUE_ENABLE, ENABLE)
            print("All arrived, Exit back_to_origin function")
            sys.exit()
        else:
            print("Part of motor still not prepared~")
            for j in DXL_ID:
                dxl_present_position, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, j, ADDR_PRESENT_POSITION) 
                
                # To accurate the value while reading the present position from motor
                if (dxl_present_position > 4095) and (dxl_present_position < 4200000000):
                    dxl_present_position = dxl_present_position - 4095
                elif dxl_present_position > 4200000000:
                    dxl_present_position = dxl_present_position - 4294967295 + 4095
                    
                if((0 < Motor_move_back_list[0][f"M{j}"] < 2048) and (0 < dxl_present_position < 2048)) or ((2048 < Motor_move_back_list[0][f"M{j}"] < 4095) and (2048 < dxl_present_position < 4095)):
                    print("Same side!")
                    if abs(dxl_present_position - Motor_move_back_list[0][f"M{j}"]) < 60:
                        dxl_goal_velocity = 0
                        Motor_checklist[f"M{j}"] = True
                        print(f"M{j} is set to " + str(Motor_checklist[f"M{j}"]))
                    else:
                        if dxl_present_position > Motor_move_back_list[0][f"M{j}"]:
                            dxl_goal_velocity = -15
                        elif dxl_present_position < Motor_move_back_list[0][f"M{j}"]:
                            dxl_goal_velocity = 15
                else:
                    if (0 < Motor_move_back_list[0][f"M{j}"] < 2048) and (2048 < dxl_present_position < 4095):
                        dxl_goal_velocity = -15
                        print("Positive!")
                    elif (0 < dxl_present_position < 2048) and (2048 < Motor_move_back_list[0][f"M{j}"] < 4095):
                        dxl_goal_velocity = 15
                        print("Negative!")
                        
                packetHandler.write1ByteTxRx(portHandler, j, ADDR_TORQUE_ENABLE, ENABLE)
                packetHandler.write4ByteTxRx(portHandler, j, ADDR_GOAL_VELOCITY, dxl_goal_velocity)
                print("Writing Goal Velocity")

# Replay the movement of robot
def replay_movement():
    with open("Motor_Position.json") as openfile:
        Motor_read_list = json.load(openfile)
        
    print("Start replaying movement...")
    
    for item in Motor_read_list:
        for i in DXL_ID:
            packetHandler.write4ByteTxRx(portHandler, i, ADDR_GOAL_POSITION, item[f'M{i}'])
            Motor_Pos_in_Replaying, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, i, ADDR_PRESENT_POSITION)
            print(f"M{i} Replay  Pos: {item[f'M{i}']}")
            print(f"M{i} Present Pos: {Motor_Pos_in_Replaying}")
        time.sleep(0.01)
    
    for i in DXL_ID:
        packetHandler.write1ByteTxRx(portHandler, i, ADDR_TORQUE_ENABLE, DISABLE)
        if replaying == False:
            print("Interrupt replaying movement")
            sys.exit()
    print("Replay successfull!")

# Recording by pyaudio
def record_audio(audio_path):
    stream = p.open(
                    format=FORMAT,
                    channels=CHANNELS,
                    rate=RATE,
                    input=True,
                    frames_per_buffer=FRAMES_PER_BUFFER
                    )

    print("Start recording audio...")
    
    while recording:
        data = stream.read(FRAMES_PER_BUFFER)
        frames.append(data)
        
        # Check for event to stop recording
        if recording == False:
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
    # sys.exit()
    return

# Replaying by pyaudio
def replay_audio(audio_file):
    playsound(audio_file)
    return
#=====================================================================================================================

# Main program
try:
    while True:
        record_btn_state = GPIO.input(11)
        replay_btn_state = GPIO.input(13)
        
        if record_btn_state == False:
            print('Record Button Pressed')
            
            if recording == False:
                recording = True
                Motor_Position_Data.clear()
                recording_Movement = RepeatingTimer(1, record_movement_through_time)
                recording_Movement.start()    
                print("Recording Movement...")
                
                # # Setup threading
                # Record_Movement_Thread = threading.Thread(target=record_movement, )
                # Record_Movement_Thread.start()
                
                Record_Audio_Thread = threading.Thread(target=record_audio, args=('output000.mp3', ))
                Record_Audio_Thread.start()
                print("Recording...")
                
            elif recording == True:
                #Stop recording movement
                recording_Movement.cancel()
                
                #Store motor position data into json
                json_position_data = json.dumps(Motor_Position_Data_List, indent=4)
                with open("Motor_Position.json", "w") as outfile:
                    outfile.write(json_position_data)
                
                #Ending Record
                print("End Record...")
                recording = False
                
            time.sleep(0.5)            

        if replay_btn_state == False:
            print('Replay Button Pressed')
            if replaying == False:
                replaying = True
                
                Back_Origin_Thread = threading.Thread(target=back_origin, )
                Back_Origin_Thread.start()
                Back_Origin_Thread.join()
                
                Replay_Movement_Thread = threading.Thread(target=replay_movement, )
                Replay_Movement_Thread.start()
                
                Replay_Audio_Thread = threading.Thread(target=replay_audio, args=('output000.mp3', ))
                Replay_Audio_Thread.start()

            elif replaying == True:
                replaying = False
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