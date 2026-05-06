import serial
from serial.tools.list_ports import comports
import numpy as np

def initialize_esp(baudrate=921600):
    #standard Espressif vendor id's
    ESP_VIDS = {0x10C4, 0x1A86}

    #find the port the ESP is connected to
    esp_ports = [p for p in comports() if p.vid in ESP_VIDS]

    if len(esp_ports) == 0:
        raise RuntimeError("No ESP connection found")
    # if len(esp_ports) > 1:
    #     raise RuntimeError(f"Multiple ESPs connected ({len(esp_ports)} found), expected exactly one")
    for p in esp_ports:
        #connect to the correct device name (e.g. /dev/ttyUSB0)
        if p.device == "/dev/ttyUSB0":
            print(f"Connecting to ESP on {p.device} with baudrate {baudrate}")
            ser = serial.Serial(p.device, baudrate=baudrate)
            ser.read()  # wait for connection
            return ser

    raise RuntimeError("No ESP connection found on /dev/ttyUSB0")

def get_sensor_data(ser: serial.Serial, data_array):
    """
    return array structure:
    [heading, gyro, lin_acc_x, lin_acc_y, US_1_distance, US_2_distance,  OFS_1_X, OFS_1_Y, OFS_2_X, OFS_2_Y, elapsed time]
    """
    ser.reset_input_buffer()
    data = ser.readline()
    data_list = data.split()
    print(data_list)
    if len(data_list) != 11:
        print("Error reading line, expected 11 elements but got ", len(data_list))
        return

    #data_list = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0, 0, 0, 0, 1]
    for i in range(6):
        data_array[i] = float(data_list[i])
    for i in range(6,11):
        data_array[i] = int(data_list[i])
    return
# data_array = np.zeros(12)
# ser = initialize_esp()
# get_sensor_data(ser, data_array)
# print(data_array)
