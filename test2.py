import RPi.GPIO as GPIO
from time import sleep, time, perf_counter
import threading
import math
import board
import busio
from adafruit_pca9685 import PCA9685
from pynsshkeyboardput import listen_keyboard, stop_listening

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


def motor_loop():
    while True:
        if motor_state['up'] and not motor_state['down']:
            print("Moving forward")
            turn_on(50, 0, 'CW')
            turn_on(50, 1, 'CW')
            turn_on(50, 2, 'CW')
            turn_on(50, 3, 'CW')

        elif motor_state['down'] and not motor_state['up']:
            print("Moving backward")
            turn_on(50, 0, 'CCW')
            turn_on(50, 1, 'CCW')
            turn_on(50, 2, 'CCW')
            turn_on(50, 3, 'CCW')

        else:
            turn_off(0)
            turn_off(1)
            turn_off(2)
            turn_off(3)

        if motor_state['left'] and not motor_state['right']:
            print("Turning left")
            turn_on(20, 4, 'CCW')
            turn_on(20, 5, 'CCW')
            
        elif motor_state['right'] and not motor_state['left']:
            print("Turning right")
            turn_on(20, 4, 'CW')
            turn_on(20, 5, 'CW')
        else:
            turn_off(4)
            turn_off(5)
        time.sleep(0.05)


def press(key):
  global switch, switch_vals
  if key in ["q","esc"]:
    stop_listening()
  if key == "up" and not motor_state["down"]:
    motor_state["up"] = not motor_state["up"]
  elif key == "down" and not motor_state["up"]:
    motor_state["down"] = not motor_state["down"]
  elif key == "right" and not motor_state["left"]:
      motor_state["right"] = not motor_state["right"]
  elif key == "left" and not motor_state["right"]:
      motor_state["left"] = not motor_state["left"]

  motor_loop()


def keyboardControl():
    print("Listening to keybord input (over SSH). Press \"space\" to toggle, and \"ESC\" or \"q\" to quit")

    try:
        listen_keyboard(
        on_press=press,
        )

    except Exception as e:
        print(f"Error occurred: {e}")
    
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
