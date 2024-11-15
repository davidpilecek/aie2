#!/usr/bin/env python3
import serial
import time


data_array = [1, 10, 20, 30]

if __name__ == '__main__':
    ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
    ser.reset_input_buffer()
    time.sleep(2)

    ser.write(bytes(data_array))
    while True:
   # print(str(led_number).encode('utf-8'))
      #  ser.write(data)
        num_read = ser.read()
        print(int.from_bytes(num_read, byteorder = 'big'))
   # print(int.from_bytes(num_read, byteorder = 'big'))
        
