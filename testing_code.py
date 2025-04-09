import RPi.GPIO as GPIO
from time import sleep, time, perf_counter
import threading
import math
import board
import busio
from adafruit_pca9685 import PCA9685
from pynput import keyboard

class Motor: #Making a Motor class so we can access everything in one place
    def __init__(self,name,I2C_Channel,Motor_Type,DIR_Pin,ENC_Pin,Pulse_Count):
        self.name = name
        self.I2C_Channel = I2C_Channel
        self.Motor_Type = Motor_Type
        self.DIR_Pin = DIR_Pin
        self.ENC_Pin = ENC_Pin
        self.Pulse_Count = Pulse_Count

# ========= CONSTANTS & PIN SETUP =========
#DIRECTION PINS
FML_DIR_Pin = 11
BML_DIR_Pin = 13
FMR_DIR_Pin = 15
TML_DIR_Pin = 10
TMR_DIR_Pin = 12
BMR_DIR_Pin = 31
DIR_Pins = {0: FML_DIR_Pin, 1: FMR_DIR_Pin, 2: BML_DIR_Pin, 3: BMR_DIR_Pin, 4: TML_DIR_Pin, 5: TMR_DIR_Pin}
#ENCODDER PINS
FML_ENC_Pin = 32
FMR_ENC_Pin = 33
BML_ENC_Pin = 18
BMR_ENC_Pin = 27
TML_ENC_Pin = 35
TMR_ENC_Pin = 37
ENC_Pins = {0: FML_ENC_Pin, 1: FMR_ENC_Pin, 2: BML_ENC_Pin, 3: BMR_ENC_Pin, 4: TML_ENC_Pin, 5: TMR_ENC_Pin}
ENC_Pins_Flipped = {FML_ENC_Pin: 0, FMR_ENC_Pin: 1, BML_ENC_Pin: 2, BMR_ENC_Pin: 3, TML_ENC_Pin: 4, TMR_ENC_Pin: 5}
#PULSE COUNTS
FML_Pulse_Count = 0
BML_Pulse_Count = 0
FMR_Pulse_Count = 0
TML_Pulse_Count = 0
TMR_Pulse_Count = 0
BMR_Pulse_Count = 0
Pulse_Counts = {0: FML_Pulse_Count, 1: FMR_Pulse_Count, 2: BML_Pulse_Count, 3: BMR_Pulse_Count, 4: TML_Pulse_Count, 5: TMR_Pulse_Count}

#FML = Motor("FML",0,"Turning", FML_DIR_Pin, FML_ENC_Pin, FML_Pulse_Count)
Turning_PPR = 854         # Pulses per revolution
Straight_PPR = 500
gear_ratio = 7.2  # Gear ratio if needed
motor_channel = 0 # PCA9685 output channel for PWM to your motor driver
frequency = 1600

GPIO.cleanup()
GPIO.setmode(GPIO.BOARD)


# ========= PCA9685 SETUP =========
i2c = busio.I2C(board.SCL, board.SDA)
pca9685 = PCA9685(i2c)
pca9685.frequency = frequency  # e.g. 1 kHz

# ========= ENCODER GLOBALS =========
def update_position(GPIO_channel):
    """Encoder callback function to track the motor’s rotation."""
    #Channel # : Motor = {0 : FML, 1 : FMR, 2 : BML, 3 : BMR, 4 : TML, 5 : TMR}
    # On a rising or falling edge, increment if the pin is HIGH
    # or you could just do 'pulse_count += 1' every edge:
    #print(f"channel: {channel} + ENC_Pins: {ENC_Pins[channel]}")
    #print("GPIO enc level:", GPIO.input(GPIO_channel))
    if GPIO.input(GPIO_channel) == GPIO.LOW:
        if GPIO.input(DIR_Pins[ENC_Pins_Flipped[GPIO_channel]]) == 0: #assuming negative
            Pulse_Counts[ENC_Pins_Flipped[GPIO_channel]] -= 1
        else:
            Pulse_Counts[ENC_Pins_Flipped[GPIO_channel]] += 1
        print (f"pulse count for channel {ENC_Pins_Flipped[GPIO_channel]}: {Pulse_Counts[ENC_Pins_Flipped[GPIO_channel]]}")
       
#GPIO.add_event_detect(32, GPIO.RISING, callback=update_position) #to increase effeicency we can only call it during rising, 
#GPIO.add_event_detect(ENC_Pins[channel], GPIO.BOTH, callback=update_position)
for channel in range(0, 6):
    GPIO.setup(DIR_Pins[channel], GPIO.OUT)
    if(ENC_Pins[channel] != 27):
        GPIO.setup(ENC_Pins[channel], GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.add_event_detect(ENC_Pins[channel], GPIO.BOTH, callback=update_position)
# ========= HELPER FUNCTIONS =========
def set_duty_cycle(percent, channel):
    """
    Sets the duty cycle on the given PCA9685 channel.
    - percent: 0 to 100
    - channel: which channel on the PCA9685 to use
    """
    # PCA9685 (Adafruit lib) expects a 16-bit value (0–65535) for duty cycle.
    # Convert from 0–100%:
    duty_16bit = int((percent / 100.0) * 65535)
    pca9685.channels[channel].duty_cycle = duty_16bit
    #duty_16bit = int((percent / 100.0) * 65535)
    #pca9685.channels[FML.I2C_Channel].duty_cycle = duty_16bit
#set_duty_cycle(0,0)

def rotate_motor(degrees, duty_cycle, channel):
    """
    Rotate the motor ‘degrees’ at ‘duty_cycle’ % speed (0–100).
    Uses the global pulse_count and PPR to stop accurately.
    """
    pulse_count = Pulse_Counts[channel]
    # PPR = Turning_PPR if (channel == 4 or channel == 5) else Straight_PPR
    PPR = Turning_PPR
    #pulse_count = 0  # reset before starting
    # Start motor
    set_duty_cycle(duty_cycle, channel)
    start_time = time()
    if GPIO.input(DIR_Pins[channel]) == 0: #If CCW goes negative
        
        target_pulses = pulse_count - (degrees / 360.0) * PPR
        print(target_pulses, pulse_count)
        while Pulse_Counts[channel] > target_pulses:
            sleep(1/frequency)
    else:
        target_pulses = (degrees / 360.0) * PPR + pulse_count
        print(target_pulses, pulse_count)
        while Pulse_Counts[channel] < target_pulses:
            #print(Pulse_Counts[channel])
            sleep(1/frequency)
    end_time = time()
    # Stop motor
    set_duty_cycle(0, channel)
    sleep(1)

    # Approx average velocity in rev/s:
    seconds = end_time - start_time
    revs = degrees / 360.0
    if seconds > 0:
        avg_vel = revs / seconds
    else:
        avg_vel = 0
    print(f"Rotated {degrees} deg at {duty_cycle}% => time={seconds:.3f}s, avg_vel={avg_vel:.3f} rev/s")

def turn_wheel(degrees, duty_cycle, direction, channel):
    """
    Rotates the *wheel* a certain number of degrees,
    factoring in a gear ratio by rotating the motor degrees * gear_ratio.
    Channel # : Motor = {4 : TML, 5 : TMR}
    """
    # Set direction pin on Pi
    if direction == "CCW":
        GPIO.output(DIR_Pins[channel], GPIO.LOW)
    elif direction == "CW":
        GPIO.output(DIR_Pins[channel], GPIO.HIGH)
    else:
        raise ValueError('Direction must be "CW" or "CCW".')

    motor_degs = degrees * gear_ratio
    rotate_motor(motor_degs, duty_cycle, channel)

#turn_wheel(90,99,'CW',0)
#Pulse_Counts[0] = 0
#GPIO.output(DIR_Pins[0], GPIO.HIGH)
#rotate_motor(360,100, 0)


 
def drive_straight(duty_cycle, direction, channel, max_time):
    """
    Drives the stright-line motors forward for 'max_time' (in seconds)
    Channel # : Motor = {0 : FML, 1 : FMR, 2 : BML, 3 : BMR}
    """
    if direction == "CCW":
        GPIO.output(DIR_Pins[channel], GPIO.LOW)
    elif direction == "CW":
        GPIO.output(DIR_Pins[channel], GPIO.HIGH)
    else:
        raise ValueError('Direction must be "CW" or "CCW".') 
        
    #pulse_count = 0  # reset before starting
    # Start motor
    set_duty_cycle(duty_cycle, channel)
    start_time = perf_counter()
            
    while (perf_counter() - start_time) < max_time:
        sleep(1/frequency)
        
    # Stop motor once enough time
    set_duty_cycle(0, channel)
    sleep(1)

    print(f"Drove {max_time} seconds") # idk how to get the encoder distance lol
    
#drive_straight(99, 'CW', 0, 3)
    

#WORK ON STRAIGHT LINE FUNCTION
#WIRE UP EVERYTHING TOGETHER IN PARALLEL 


def turn_simultaneously(angle_left, duty_cycle_left, direction_left, angle_right, duty_cycle_right, direction_right):
    """
    Threading to turn both turning drivers at the same time
    """
    thread_left = threading.Thread(target=turn_wheel, args=(angle_left, duty_cycle_left, direction_left, 4)) # TML
    thread_right = threading.Thread(target=turn_wheel, args=(angle_right, duty_cycle_right, direction_right, 5)) # TMR
    
    thread_left.start()
    thread_right.start()
    
    thread_left.join()
    thread_right.join()

def drive_simultaneously(duty_cycle, direction, time):
    """
    Threading to move both the front and back straight-line drivers at the same time
    """
    thread_front_left = threading.Thread(target=drive_straight, args=(duty_cycle, direction, 0, time)) # replace with correct channel
    thread_front_right = threading.Thread(target=drive_straight, args=(duty_cycle, direction, 1, time))
    thread_back_left = threading.Thread(target=drive_straight, args=(duty_cycle, direction, 2, time))
    thread_back_right = threading.Thread(target=drive_straight, args=(duty_cycle, direction, 3, time))

    thread_front_left.start()
    thread_front_right.start()
    thread_back_left.start()
    thread_back_right.start()
    
    thread_front_left.join()
    thread_front_right.join()
    thread_back_left.join()
    thread_back_right.join()
    
    
def turn_simultaneously_time(duty_cycle_left, direction_left, duty_cycle_right, direction_right, time):
    """
    Threading to turn both turning drivers at the same time, for a given amount of time
    """
    thread_left = threading.Thread(target=drive_straight, args=(duty_cycle_left, direction_left, 4, time)) # TML
    thread_right = threading.Thread(target=drive_straight, args=(duty_cycle_right, direction_right, 5, time)) # TMR
    
    thread_left.start()
    thread_right.start()
    
    thread_left.join()
    thread_right.join()

def turn_and_drive_forward(velocity, time, turn_direction, duty_cycle_left, duty_cycle_right):
    """
    Threading to call both the steering and turning code at the same time
    """
    thread_steering = threading.Thread(target=drive_simultaneously, args=(velocity, 'CW', time)) # CHANGE WITH CORRECT DIRECTION
    # thread_turning = threading.Thread(target=turn_simultaneously, args=(angle_left, duty_cycle_left, turn_direction, angle_right, duty_cycle_right, turn_direction))
    thread_turning = threading.Thread(target=turn_simultaneously_time, args=(duty_cycle_left, turn_direction, duty_cycle_right, turn_direction, time))  
    
    thread_steering.start()
    thread_turning.start()
   
    thread_steering.join()
    thread_turning.join()
    
    print(f"Drove for {time} seconds and turned {turn_direction}")



######################################################################
########################## REMOTE CONTROL ############################
######################################################################

def turn_on(duty_cycle, channel, direction):
    if direction == "CCW":
        GPIO.output(DIR_Pins[channel], GPIO.LOW)
    elif direction == "CW":
        GPIO.output(DIR_Pins[channel], GPIO.HIGH)
    else:
        raise ValueError('Direction must be "CW" or "CCW".') 
    
    set_duty_cycle(duty_cycle, channel)

def turn_off(channel):
    set_duty_cycle(0, channel)

# Motor state
motor_state = {
    'up': False,
    'down': False,
    'left': False,
    'right': False
}

def on_press(key):
    try:
        if key == keyboard.Key.up:
            motor_state['up'] = True
        elif key == keyboard.Key.down:
            motor_state['down'] = True
        elif key == keyboard.Key.left:
            motor_state['left'] = True
        elif key == keyboard.Key.right:
            motor_state['right'] = True
        elif key.char == 'q':
            print("Bye Honu")
            return False  # Stop the listener
    except AttributeError:
        pass

def on_release(key):
    if key == keyboard.Key.up:
        motor_state['up'] = False
    elif key == keyboard.Key.down:
        motor_state['down'] = False
    elif key == keyboard.Key.left:
        motor_state['left'] = False
    elif key == keyboard.Key.right:
        motor_state['right'] = False

def motor_loop():
    while True:
        if motor_state['up'] and not motor_state['down']:
            print("Moving forward")
            turn_on(70, 0, 'CW')
            turn_on(70, 1, 'CW')
            turn_on(70, 2, 'CW')
            turn_on(70, 3, 'CW')

        elif motor_state['down'] and not motor_state['up']:
            print("Moving backward")
            turn_on(70, 0, 'CCW')
            turn_on(70, 1, 'CCW')
            turn_on(70, 2, 'CCW')
            turn_on(70, 3, 'CCW')

        else:
            turn_off(0)
            turn_off(1)
            turn_off(2)
            turn_off(3)

        if motor_state['left'] and not motor_state['right']:
            print("Turning left")
            turn_on(70, 4, 'CCW')
            turn_on(70, 5, 'CCW')
            
        elif motor_state['right'] and not motor_state['left']:
            print("Turning right")
            turn_on(70, 4, 'CW')
            turn_on(70, 5, 'CW')
        else:
            turn_off(4)
            turn_off(5)
        time.sleep(0.05)

def keyboardControl():
    motor_thread = threading.Thread(target=motor_loop, daemon=True)
    motor_thread.start()

    try:
        with keyboard.Listener(on_press=on_press, on_release=on_release) as listener:
            listener.join()
    finally:
        print("Cleaning up motors and GPIO")
        for ch in range(6):
            turn_off(ch)
        GPIO.cleanup()

'''
# ========= TEST CODE / MAIN =========
if __name__ == "__main__":
    try:
        print("Starting PCA9685-based PWM test with encoder feedback ...")

        num_trials = 5
        average_times = []

        # Example: do 360 deg rotation from 100% down to 50%, step -10
        start_duty = 100
        end_duty   = 10
        for channel in range(0,6):
            for duty_cycle in range(start_duty, end_duty - 1, -10):
                time_sum = 0.0
                for i in range(num_trials):
                    print(i)
                    t1 = time()
                    rotate_wheel(360, duty_cycle, 'CW', channel)
                    t2 = time()


                    rotation_time = t2 - t1
                    time_sum += rotation_time

                avg_time = time_sum / num_trials
                average_times.append((duty_cycle, avg_time))

        # Print results
        for duty, avg_t in average_times:
            print(f"Avg time for 360° at {duty}% duty = {avg_t:.2f} s")

    except KeyboardInterrupt:
        print("Interrupted by user!")
    finally:
        # Always stop PWM and clean up
        for channel in range(0, 6):
            set_duty_cycle(0, channel)
        pca9685.deinit()
        GPIO.cleanup()
'''
