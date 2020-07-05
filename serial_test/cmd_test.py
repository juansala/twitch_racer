import serial 
import math
import time
from datetime import datetime

ser = serial.Serial('/dev/serial0', 9600)
test_string = "1,1,1,1"

while True:
    ser.write(test_string)
    ser.flush()
    print(test_string)
    time.sleep(1)
