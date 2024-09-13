import time
import board
import usb_hid
from analogio import AnalogIn
from digitalio import DigitalInOut, Direction
from hid_gamepad import Gamepad

# Setup ADC inputs
adc_pitch = AnalogIn(board.A1)  # ADC1 for pitch
adc_roll = AnalogIn(board.A2)   # ADC2 for roll
adc_shared = AnalogIn(board.A0)  # ADC0 shared for throttle and yaw

# Use correct GPIO pin numbers for the Raspberry Pi Pico (e.g., GP3 and GP6)
mux_select_pin_1 = board.GP3  # GPIO3 to control multiplexer
mux_select_pin_2 = board.GP6  # GPIO6 to control multiplexer

# Setup multiplexer control pins as outputs
mux1 = DigitalInOut(mux_select_pin_1)
mux1.direction = Direction.OUTPUT
mux2 = DigitalInOut(mux_select_pin_2)
mux2.direction = Direction.OUTPUT

# Initialize arrays to store raw ADC values
adc_throttle_array = [0] * 20
adc_yaw_array = [0] * 20
adc_pitch_array = [0] * 20
adc_roll_array = [0] * 20

averaging_sample_size = 20
averaging_sample_array_index = 0
axis_index = 0
deadzone = 5.0

# Read ADC values and switch between throttle and yaw on ADC0
def joystick_read_callback():
    global axis_index, averaging_sample_array_index

    # Multiplexed ADC0: Throttle and Yaw
    if axis_index == 0:
        mux1.value = False
        mux2.value = False  # Switch: ADC0 -> throttle
        adc_throttle_array[averaging_sample_array_index] = adc_shared.value
    elif axis_index == 1:
        mux1.value = False
        mux2.value = True  # Switch: ADC0 -> yaw
        adc_yaw_array[averaging_sample_array_index] = adc_shared.value
    elif axis_index == 2:
        adc_pitch_array[averaging_sample_array_index] = adc_pitch.value
    elif axis_index == 3:
        adc_roll_array[averaging_sample_array_index] = adc_roll.value

    axis_index += 1
    if axis_index >= 4:
        axis_index = 0
        averaging_sample_array_index += 1
        averaging_sample_array_index %= averaging_sample_size

# Function to get the averaged ADC value
def get_average_value(adc_array):
    return sum(adc_array) // averaging_sample_size

# Conversion functions based on the updated 16-bit ADC ranges
def joystick_get_throttle_percent():
    average_throttle = get_average_value(adc_throttle_array)
  #  print(f"Raw Throttle ADC Value: {average_throttle}")  # Debugging print
    percent_value = ((average_throttle - 23505.0) * 100.0) / (46986.0 - 23505.0)
    return max(0.0, min(100.0, percent_value))

def joystick_get_yaw_percent():
    average_yaw = get_average_value(adc_yaw_array)
 #   print(f"Raw Yaw ADC Value: {average_yaw}")  # Debugging print
    percent_value = ((average_yaw - 24538.0 - 1712.0) * 100.0) / (51891.0 - 24538.0)
    if abs(percent_value - 50.0) < deadzone:
        percent_value = 50.0
    elif percent_value > 50.0 + deadzone:
        percent_value = 50.0 + ((percent_value - (50.0 + deadzone)) * 100.0) / ((50.0 - deadzone) * 2.0)
    else:
        percent_value = (percent_value * 100.0) / ((50.0 - deadzone) * 2.0)
    return max(0.0, min(100.0, percent_value))

def joystick_get_pitch_percent():
    average_pitch = get_average_value(adc_pitch_array)
  #  print(f"Raw Pitch ADC Value: {average_pitch}")  # Debugging print
    percent_value = ((average_pitch - 20469.0 - 1072.0) * 100.0) / (45920.0 - 20469.0)
    if abs(percent_value - 50.0) < deadzone:
        percent_value = 50.0
    elif percent_value > 50.0 + deadzone:
        percent_value = 50.0 + ((percent_value - (50.0 + deadzone)) * 100.0) / ((50.0 - deadzone) * 2.0)
    else:
        percent_value = (percent_value * 100.0) / ((50.0 - deadzone) * 2.0)
    return max(0.0, min(100.0, percent_value))

def joystick_get_roll_percent():
    average_roll = get_average_value(adc_roll_array)
 #   print(f"Raw Roll ADC Value: {average_roll}")  # Debugging print
    percent_value = ((average_roll - 23313.0 + 48.0) * 100.0) / (47101.0 - 23313.0)
    if abs(percent_value - 50.0) < deadzone:
        percent_value = 50.0
    elif percent_value > 50.0 + deadzone:
        percent_value = 50.0 + ((percent_value - (50.0 + deadzone)) * 100.0) / ((50.0 - deadzone) * 2.0)
    else:
        percent_value = (percent_value * 100.0) / ((50.0 - deadzone) * 2.0)
    return max(0.0, min(100.0, percent_value))

def percent_to_hid(percent):
    return int((percent - 50.0) * (127 / 50.0))  # Map 0-100% to -127 to 127


gp = Gamepad(usb_hid.devices)

loop_iterations_limit = 10
loop_iterations = 0

# Main loop to read the ADC values and print the percentages
while True:
    joystick_read_callback()  # Read ADC values in a round-robin fashion

    # Print raw values for debugging
    throttle_percent = joystick_get_throttle_percent()
    yaw_percent = joystick_get_yaw_percent()
    pitch_percent = joystick_get_pitch_percent()
    roll_percent = joystick_get_roll_percent()
    
    if loop_iterations == loop_iterations_limit:
        # Convert percent to HID joystick range (-127 to 127)
        throttle_hid = percent_to_hid(throttle_percent)
        yaw_hid = percent_to_hid(yaw_percent)
        pitch_hid = percent_to_hid(pitch_percent)
        roll_hid = percent_to_hid(roll_percent)

        #print(f"Throttle: {throttle_percent:.2f}%, Yaw: {yaw_percent:.2f}%, Pitch: {pitch_percent:.2f}%, Roll: {roll_percent:.2f}%")
        
        gp.move_joysticks(
            x=-yaw_hid,
            y=-throttle_hid,
            z=pitch_hid,
            r_z=roll_hid,
        )
        
        loop_iterations = 0
    
    
    #print(throttle_hid)
    time.sleep(0.0005)  # Delay for testing, adjust as needed
    
    loop_iterations = loop_iterations + 1
