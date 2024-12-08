import serial
import time
from config import *

zeros_array = [0, 0, 0, 0, 0, 0, 0, 0]

#arduino pinout: FRA, FLA, RRA, RLA, FRB, FLB, RRB, RLB

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

def turn_left(speed_turn, serial_port):
    data_array = [speed_turn, 0, speed_turn, 0, 0, 0, 0, 0]
    serial_port.write(bytes(data_array))
    print("turning left")

def turn_right(speed_turn, serial_port):
    data_array = [0, speed_turn, 0, speed_turn, 0, 0, 0, 0]
    serial_port.write(bytes(data_array))
    print("turning right")

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


def align_center(centre_x, speed_strafe, serial_port):
    if centre_x < CENTRE_OF_FRAME[0] - MARGIN_OF_CENTER_MISALIGNMENT:
        aligned_center = False
        last_marker_position = -1
        strafe_left(speed_strafe, serial_port)
        
    elif centre_x > CENTRE_OF_FRAME[0] + MARGIN_OF_CENTER_MISALIGNMENT:
        aligned_center = False
        last_marker_position = 1
        strafe_right(speed_strafe, serial_port)
        
    else:
        last_marker_position = 0
        aligned_center = True
        print("translation aligned")

    return aligned_center, last_marker_position


def align_distance(transform_translation_z, speed_drive, serial_port):
    if transform_translation_z > DISTANCE_FROM_MARKER + MARGIN_OF_DISTANCE:
        aligned_distance = False
        drive_forward(speed_drive, serial_port)
        
    elif transform_translation_z < DISTANCE_FROM_MARKER - MARGIN_OF_DISTANCE:
        aligned_distance = False
        drive_reverse(speed_drive, serial_port)
    else:
        aligned_distance = True
        print("distance aligned")
    return aligned_distance


def align_rotation(yaw_deg, speed_drift, serial_port):

    if MARGIN_OF_ANGLE < yaw_deg < 90:
        aligned_rotation = False
        drift_left(speed_drift, serial_port)
        
    elif -90 < yaw_deg < -MARGIN_OF_ANGLE:
        aligned_rotation = False
        drift_right(speed_drift, serial_port)
       
    else:
        aligned_rotation = True
        print("rotation aligned")
    return aligned_rotation



speed_all = 16
delay = 1

SPEED_COEF = 0.7
offset = -0.5
angle = 0.5

if __name__ == '__main__':
    arduino_port = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
    arduino_port.reset_input_buffer()
    time.sleep(2)
    #speed_FR = int(MAX_SPEED + offset - angle)
    speed_FR = int(SPEED_COEF - offset - angle)
    speed_FL = int(SPEED_COEF + offset + angle)
    speed_RR = int(SPEED_COEF - offset - angle)
    speed_RL = int(SPEED_COEF + offset + angle)
    turn(speed_FR, speed_FL, speed_RR, speed_RL, arduino_port)
    
    time.sleep(1)
    stop_all(arduino_port)
    quit()    
    
