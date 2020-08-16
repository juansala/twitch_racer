#!/usr/bin/env python
import serial
from time import time, sleep

import rospy
import roslib
from std_msgs.msg import String
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
#from geometry_msgs.msg import PoseStamped

class SerialBridge():
    #MEASURE_TOPIC = "measurements"
    #COMMAND_TOPIC = "command"

    def __init__(self, teensyPort='/dev/serial0', teensyBaud = 9600, teensyUpdateInterval=1.25):
        #rospy.loginfo("Serial node started.")
        print('Serial node started.')
        
        self.raw_pub = rospy.Publisher("measurements", String, queue_size=1)
        rospy.Subscriber("command", String, self.cmd_cb)
        rospy.Subscriber("key_vel", Twist, self.key_cb)
        self._teensySerial = serial.Serial(teensyPort, baudrate=teensyBaud, timeout=None, bytesize=serial.EIGHTBITS, parity = serial.PARITY_NONE, stopbits = serial.STOPBITS_ONE)
        self._teensyUpdateInterval = teensyUpdateInterval

        self.buf = []
        self.incomingStringLength = 31 # placeholder value for teensy serial message length

    def convert(self, s):
        str1 = ""
        return(str1.join(s))

    def parseSensorData(self, data):
        data = data.replace(" ", "")
        arr = data.split(",")
        values = [float(i) for i in arr]
        return values

    def listen(self):
        while not rospy.is_shutdown():
            if self._teensySerial.inWaiting():
                x = self._teensySerial.read_until()
                self.buf.append(x)
                if len(self.buf) >  self.incomingStringLength:
                    self._teensySerial.flush()
                    msg = self.convert(self.buf)
                    data = self.parseSensorData(msg)
                    rospy.loginfo(data)
                    self.raw_pub.publish(msg)
                    self.buf = []

    def cmd_cb(self, msg):
        cmd = msg.data
        self._teensySerial.flushInput()
        self._teensySerial.flushOutput()
        self._teensySerial.write(cmd)
        print(cmd)
        #rospy.loginfo(cmd)

    def key_cb(self, msg):
        pass

if __name__ == '__main__':
    import sys
    # update_hz = 30
    rospy.init_node('serial', anonymous=True)
    piSerial = SerialBridge()
    piSerial.listen()
    rospy.spin()
