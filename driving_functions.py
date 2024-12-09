import serial
import time
from config import *

zeros_array = [0, 0, 0, 0, 0, 0, 0, 0]

#arduino pinout: FRA, FLA, RRA, RLA, FRB, FLB, RRB, RLB

# define all the driving functions
def drive_forward(speed_drive, serial_port):
    data_array = [speed_drive, speed_drive, speed_drive, speed_drive, 0, 0, 0, 0]
    serial_port.write(bytes(data_array))
    print("driving forward")
    
def drive_reverse(speed_drive, serial_port):
    data_array = [0, 0, 0, 0, speed_drive, speed_drive, speed_drive, speed_drive]
    serial_port.write(bytes(data_array))
    print("driving reverse")

def turn(line_angle, offset, serial_port):
    speed_FR = int(MAX_SPEED * (SPEED_COEF - offset*OFFSET_COEF - line_angle*ANGLE_COEF))
    speed_FL = int(MAX_SPEED * (SPEED_COEF + offset*OFFSET_COEF + line_angle*ANGLE_COEF))
    speed_RR = int(MAX_SPEED * (SPEED_COEF - offset*OFFSET_COEF - line_angle*ANGLE_COEF))
    speed_RL = int(MAX_SPEED * (SPEED_COEF + offset*OFFSET_COEF + line_angle*ANGLE_COEF))
    data_array = [speed_FR, speed_FL, speed_RR, speed_RL, 0, 0, 0, 0]
    serial_port.write(bytes(data_array))
    print("turning")

def strafe_right(speed_strafe, serial_port):
    data_array = [0, speed_strafe, speed_strafe, 0, speed_strafe, 0, 0, speed_strafe]
    serial_port.write(bytes(data_array))
    print("strafing R")

def strafe_left(speed_strafe, serial_port):
    data_array = [speed_strafe, 0, 0, speed_strafe, 0, speed_strafe, speed_strafe, 0]
    serial_port.write(bytes(data_array))
    print("strafing L")

def drift_left(speed_drift, serial_port):
    data_array = [0, 0, 0, speed_drift, 0, 0, speed_drift, 0]
    serial_port.write(bytes(data_array))
    print("drift l")

def drift_right(speed_drift, serial_port):
    data_array = [0, 0, speed_drift, 0, 0, 0, 0, speed_drift]
    serial_port.write(bytes(data_array))
    print("drift r")

def spin_left(speed_spin, serial_port):
    data_array = [speed_spin, 0, speed_spin, 0, 0, speed_spin, 0, speed_spin]
    serial_port.write(bytes(data_array))
    print("spin l")

def spin_right(speed_spin, serial_port):
    data_array = [0, speed_spin, 0, speed_spin, speed_spin, 0, speed_spin, 0]
    serial_port.write(bytes(data_array))
    print("spin r")

def stop_all(serial_port):
    serial_port.write(bytes(zeros_array))
    print("stop")

# for testing purposes
if __name__ == '__main__':
    arduino_port = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
    arduino_port.reset_input_buffer()
    time.sleep(2)
    for PWM_dutycycle in range(20):
        print(PWM_dutycycle)
        drive_forward(PWM_dutycycle, arduino_port)
        time.sleep(1)


    time.sleep(1)
    stop_all(arduino_port)
    quit()
