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
    if len(esp_ports) > 1:
        raise RuntimeError(f"Multiple ESPs connected ({len(esp_ports)} found), expected exactly one")

    ser = serial.Serial(esp_ports[0].device, baudrate=baudrate)
    ser.read()  # wait for connection
    return ser

def get_sensor_data(ser: serial.Serial, data_array):
    """
    return array structure:
    [heading, gyro, lin_acc_x, lin_acc_y, US_1_distance, OFS_1_X, OFS_1_Y, OFS_2_X, OFS_2_Y, elapsed time]
    """
    ser.reset_input_buffer()
    data = ser.readline()
    data_list = data.split()
    for i in range(5):
        data_array[i] = float(data_list[i])      
    for i in range(5,10):
        data_array[i] = int(data_list[i])
    return 