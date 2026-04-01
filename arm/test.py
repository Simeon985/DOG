import serial
import serial.serialutil as serialutil
from serial.serialwin32 import Serial

# Work around environment where top-level 'serial' package is a namespace
# without pyserial's attributes (Serial, SerialBase, constants) populated.
for name in dir(serialutil):
    if not name.startswith("_"):
        setattr(serial, name, getattr(serialutil, name))

ser = Serial("COM6", 115200)

while True:
    cmd = input("Command (w/a/s/d/stop): ")

    if cmd == "w":
        ser.write(b"forward\n")
    elif cmd == "s":
        ser.write(b"backward\n")
    elif cmd == "a":
        ser.write(b"left\n")
    elif cmd == "d":
        ser.write(b"right\n")
    elif cmd == "stop":
        ser.write(b"stop\n")