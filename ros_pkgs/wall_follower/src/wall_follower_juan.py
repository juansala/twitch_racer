#!/usr/bin/env python2

import numpy as np
import math

import rospy
from rospy.numpy_msg import numpy_msg
from sensor_msgs.msg import LaserScan
#from ackermann_msgs.msg import AckermannDriveStamped
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

class WallFollower:
    # Import ROS parameters from the "params.yaml" file.
    # Access these variables in class functions with self:
    # i.e. self.CONSTANT
    SCAN_TOPIC = rospy.get_param("wall_follower/scan_topic")
    DRIVE_TOPIC = rospy.get_param("wall_follower/drive_topic")
    SIDE = rospy.get_param("wall_follower/side")
    VELOCITY = rospy.get_param("wall_follower/velocity")
    DESIRED_DISTANCE = rospy.get_param("wall_follower/desired_distance")

    def __init__(self):

        #self.pub = rospy.Publisher(self.DRIVE_TOPIC, AckermannDriveStamped, queue_size=10)
	self.line_pub = rospy.Publisher("my_special_line", Marker, queue_size=10)
	rospy.Subscriber(self.SCAN_TOPIC, LaserScan, self.callback)
	self.min_side_angle = -math.pi/12 if self.SIDE == 1 else -3*math.pi/4
	self.max_side_angle = math.pi/3 if self.SIDE == 1 else  0.0
	self.distance = 0
	self.error = 0
	self.error_rate = 0
	self.error_sum = 0
	self.prev_error = 0
	self.Kp = 2.2
	self.Kd = 0.6 #0.3
	self.Ki = 0


    def callback(self, data):
        # Parse LaserScan into slices (side and front)
	slice, angles = self.slice_data(data, self.min_side_angle, self.max_side_angle)
        #rospy.loginfo(angles)
	x, y = self.polar2cartesian(slice, angles)
        #rospy.loginfo(y)

	# Find the wall using numpy.lstsq and compute distance 
        A = np.vstack([x, np.ones(len(x))]).T
        #rospy.loginfo(A)
        m, c = np.linalg.lstsq(A, y, rcond=-1)[0]
        self.distance = c/math.sqrt(m**2 + 1)

	# Visualize line
	line_msg = Marker()
	line_msg.type = Marker.LINE_STRIP
	line_msg.header.frame_id = 'laser'
	line_msg.header.stamp = rospy.get_rostime()
	line_msg.scale.x = 0.1
	line_msg.scale.y = 0.1
	line_msg.scale.z = 0.1
	line_msg.action = Marker.ADD
	line_msg.color.a = 1.0
	line_msg.color.r = 0.0
	line_msg.color.g = 1.0
	line_msg.color.b = 0.0
        line_msg.pose.orientation.x = 0.0
        line_msg.pose.orientation.y = 0.0
        line_msg.pose.orientation.z = 0.0
        line_msg.pose.orientation.w = 1.0
	
	vis_x = np.array([i for i in x])
	vis_y = vis_x * m + c
        #vis_y = y

        #rospy.loginfo(vis_x)
        #rospy.loginfo(vis_y)
        #rospy.loginfo(m)
        #rospy.loginfo(c)

        # Marker debugging
        #vis_x = np.array([1, 2, 3, 4, 5])
        #vis_y = np.array([2, 4, 6, 8, 10])

	points = []
	for i in range(len(x)):
	    point = Point()
            point.x = vis_x[i]
	    point.y = vis_y[i]
	    point.z = 0.0
	    points.append(point)
	line_msg.points = points
	self.line_pub.publish(line_msg)

	# Compute error and PID control effort
	input = self.PID(self.DESIRED_DISTANCE - abs(self.distance))

	# Prepare and publish command
	#command = AckermannDriveStamped()
	#command.drive.steering_angle = -self.SIDE * input
	#command.drive.steering_angle_velocity = 0.0
	#command.drive.speed = self.VELOCITY; # m/s
	#command.drive.acceleration = 0.0
	#command.drive.jerk =
	#self.pub.publish(command)
	#rospy.loginfo(rospy.get_caller_id() + 'Slope: %4.3f', m)
	#rospy.loginfo(rospy.get_caller_id() + 'Distance: %4.3f', self.distance)
	#rospy.loginfo(rospy.get_caller_id() + 'angle_max: %4.3f', data.angle_min)


    def slice_data(self, data, min_scangle, max_scangle):
	full_data = np.array(data.ranges) 
	slice_index_min = int((min_scangle - data.angle_min)/data.angle_increment) 
	slice_index_max = int((max_scangle - data.angle_min)/data.angle_increment)
	angles = np.array([min_scangle + i*data.angle_increment for i in range(slice_index_max - slice_index_min)])
	slice = full_data[slice_index_min: slice_index_max]
        angles = angles[(slice >= data.range_min) & (slice <= data.range_max)]
        slice = slice[(slice >= data.range_min) & (slice <= data.range_max)]
	return slice, angles

    def polar2cartesian(self, r, theta):
	x_points = r * np.cos(theta)
	y_points = r * np.sin(theta)
	return x_points, y_points

    def PID(self, error):
	self.error = error
	self.error_rate = (self.error - self.prev_error) / (0.05) 
	self.error_sum = self.error_sum + self.error * 0.05
	self.prev_error = self.error
	return self.Kp*self.error + self.Kd*self.error_rate + self.Ki*self.error_sum 

if __name__ == "__main__":
    rospy.init_node('wall_follower')
    wall_follower = WallFollower()
    rospy.spin()
