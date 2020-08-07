#!/usr/bin/env python
import serial
from time import time, sleep

import rospy
import roslib
from std_msgs.msg import String
from sensor_msgs.msg import Image
#from geometry_msgs.msg import PoseStamped

class SerialBridge():
    #MEASURE_TOPIC = "measurements"
    #COMMAND_TOPIC = "command"

    def __init__(self, teensyPort='/dev/serial0', teensyBaud = 9600, teensyUpdateInterval=1.25):
        #rospy.loginfo("Serial node started.")
        print('Serial node started.')
        
        self.pub = rospy.Publisher("measurements", String, queue_size=1)
        rospy.Subscriber("command", String, self.callback)
        self._teensySerial = serial.Serial(teensyPort, baudrate=teensyBaud, timeout=None, bytesize=serial.EIGHTBITS, parity = serial.PARITY_NONE, stopbits = serial.STOPBITS_ONE)
        self._teensyUpdateInterval = teensyUpdateInterval

    def listen(self):
        while not rospy.is_shutdown():
            if self._teensySerial.inWaiting():
                bytesToRead = self._teensySerial.inWaiting()
                x = self._teensySerial.read(bytesToRead)
                readings = String()
                readings.data = x
                self.pub.publish(readings)
                print(x),
                #rospy.loginfo(x)

    def callback(self, msg):
        cmd = msg.data
        self._teensySerial.flushInput()
        self._teensySerial.flushOutput()
        self._teensySerial.write(cmd)
        print(cmd)
        #rospy.loginfo(cmd)

if __name__ == '__main__':
    import sys
    # update_hz = 30
    rospy.init_node('serial', anonymous=True)
    piSerial = SerialBridge()
    piSerial.listen()
    rospy.spin()
