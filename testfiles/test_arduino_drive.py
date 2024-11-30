import serial
import time
import driving_functions as dfu

zeros_array = [0, 0, 0, 0, 0, 0, 0, 0]
if __name__ == '__main__':
    ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
    ser.reset_input_buffer()
    time.sleep(2)
    
    dfu.slide_left(15, ser)
    print(i)
    time.sleep(1)
           # num_read = ser.read()
           # print(int.from_bytes(num_read, byteorder = 'big'))

    ser.write(bytes(zeros_array))
    quit()


