import RPi.GPIO as GPIO
import time
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
#=====================================================================================================================
            
def print_hello():
     print("hello world")

# Main program
while True:
    record_btn_state = GPIO.input(11)
    replay_btn_state = GPIO.input(13)
    
    if record_btn_state == False:
        print('Record Button Pressed')
        
        if recording == False:
            recording = True
            print("Recording Movement...")
            print("Recording...")
            
        elif recording == True:
            print("End Record...")
            recording = False
            
        time.sleep(0.5)            
    if replay_btn_state == False:
        print('Replay Button Pressed')
        if replaying == False:
            replaying = True
        elif replaying == True:
            replaying = False
        time.sleep(0.5)