import serial
import time

zeros_array = [0, 0, 0, 0, 0, 0, 0, 0]

#FRA, FLA, RRA, RLA, FRB, FLB, RRB, RLB

def drive_straight(speed_drive, serial_port):
    data_array = [speed_drive, speed_drive, speed_drive, speed_drive, 0, 0, 0, 0]
    serial_port.write(bytes(data_array))
    
def drive_reverse(speed_drive, serial_port):
    data_array = [0, 0, 0, 0, speed_drive, speed_drive, speed_drive, speed_drive]
    serial_port.write(bytes(data_array))
    
def turn_left(speed_turn, serial_port):
    data_array = [speed_turn, 0, speed_turn, 0, 0, 0, 0, 0]
    serial_port.write(bytes(data_array))

def turn_right(speed_turn, serial_port):
    data_array = [0, speed_turn, 0, speed_turn, 0, 0, 0, 0]
    serial_port.write(bytes(data_array))

def slide_right(speed_slide, serial_port):
    data_array = [0, speed_slide, speed_slide, 0, speed_slide, 0, 0, speed_slide]
    serial_port.write(bytes(data_array))

def slide_left(speed_slide, serial_port):
    data_array = [speed_slide, 0, 0, speed_slide, 0, speed_slide, speed_slide, 0]
    serial_port.write(bytes(data_array))

def roll_left(speed_roll, serial_port):
    data_array = [0, 0, 0, speed_roll, 0, 0, speed_roll, 0]
    serial_port.write(bytes(data_array))
    
def roll_right(speed_roll, serial_port):
    data_array = [0, 0, speed_roll, 0, 0, 0, 0, speed_roll]
    serial_port.write(bytes(data_array))

def spin_left(speed_spin, serial_port):
    data_array = [speed_spin, 0, speed_spin, 0, 0, speed_spin, 0, speed_spin]
    serial_port.write(bytes(data_array))

def spin_right(speed_spin, serial_port):
    data_array = [0, speed_spin, 0, speed_spin, speed_spin, 0, speed_spin, 0]
    serial_port.write(bytes(data_array))

def stop_all(serial_port):
    serial_port.write(bytes(zeros_array))



if __name__ == '__main__':
    ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
    ser.reset_input_buffer()
    time.sleep(2)

    
    spin_right(12, ser)   
    time.sleep(3)
           # num_read = ser.read()
           # print(int.from_bytes(num_read, byteorder = 'big'))

    stop_all(ser)
    quit()    
    
