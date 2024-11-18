import serial
import time

data_array = []
zeros_array = [0, 0, 0, 0]
if __name__ == '__main__':
    ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
    ser.reset_input_buffer()
    time.sleep(2)

    while True:
        #print(str(led_number).encode('utf-8'))
        for i in range(100):
            data_array = [i, i, i, i]
            print(data_array)
            time.sleep(0.02)
            ser.write(bytes(data_array))
            num_read = ser.read()
            print(int.from_bytes(num_read, byteorder = 'big'))
    
    ser.write(bytes(zeros_array))
