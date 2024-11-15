import serial
import time

data_array = [1, 10, 20, 30]

if __name__ == '__main__':
    ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
    ser.reset_input_buffer()
    time.sleep(2)

    while True:
        #print(str(led_number).encode('utf-8'))
        ser.write(bytes(data_array))
        num_read = ser.read()
        print(int.from_bytes(num_read, byteorder = 'big'))
