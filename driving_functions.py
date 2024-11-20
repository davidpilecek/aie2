def drive_straight(speed_drive, serial_port):
    data_array = [speed_drive, speed_drive, speed_drive, speed_drive, 0, 0, 0, 0]
    serial_port.write(bytes(data_array))
    
def drive_reverse(speed_drive, serial_port):
    data_array = [0, 0, 0, 0, speed_drive, speed_drive, speed_drive, speed_drive]
    serial_port.write(bytes(data_array))

def slide_right(speed_drive, serial_port):
    data_array = [255, 0, 255, 0]
    serial_port.write(bytes(data_array))

def slide_left(speed_drive, serial_port):
    data_array = [0, 255, 0, 255]
    serial_port.write(bytes(data_array))

def stop_all(serial_port):
    data_array = [0, 0, 0, 0]
    serial_port.write(bytes(data_array))