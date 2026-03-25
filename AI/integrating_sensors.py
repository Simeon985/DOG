import serial

def initialize_esp(buadrate = 921600 ):
    #find the port that the ESP is connected to
    port_list = serial.tools.list_ports.comports()
    print(port_list)
    return
    port = None
    ser = serial.Serial(port, baudrate=buadrate)
    connected = False
    while not connected:
        serin = ser.read()
        connected = True
    return ser

def get_sensor_data(ser):
    return

initialize_esp()
