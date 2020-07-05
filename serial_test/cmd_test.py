import serial 
import math
import time
from datetime import datetime

ser = serial.Serial('/dev/serial0', 9600)
test_string = "0,0,0,0"
g = raw_input("Enter command: ")

while True:
    ser.write(g)
    ser.flush()
    print(g)
    time.sleep(1)
