#!/usr/bin/env python2

import numpy as np
import math

import rospy
from rospy.numpy_msg import numpy_msg
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
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

        self.cmd_pub = rospy.Publisher(self.DRIVE_TOPIC, String, queue_size=10)
	self.line_pub = rospy.Publisher("my_special_line", Marker, queue_size=10)
	rospy.Subscriber(self.SCAN_TOPIC, LaserScan, self.cb)
        self.front_dist_threshold = 0.25
        self.front_angle_start = math.pi - math.pi/3
        self.front_angle_end = -math.pi + math.pi/3
	self.min_side_angle = math.pi - math.pi/3 if self.SIDE == 1 else -math.pi + math.pi/12 # min/max side angle ordering matters, use common sense, use Slamtech's provided frame image for reference
	self.max_side_angle = math.pi - math.pi/12 if self.SIDE == 1 else  -math.pi + math.pi/2 #+ math.pi/3
	self.distance = 0
	self.error = 0
	self.error_rate = 0
	self.error_sum = 0
	self.prev_error = 0
	self.Kp = 10.0 #2.2
	self.Kd = 0 #0.6 #0.3
	self.Ki = 0

        self.scan = None
        rate = rospy.Rate(10) # publish commands at 10 Hz
        while not rospy.is_shutdown():
            if self.scan is not None:
                self.output(self.scan)
            rate.sleep()

    def cb(self, msg):
        self.scan = msg

    def output(self, data):

        # Extract front left and front right data then concatenate
        front_slice_right, front_angles_right = self.slice_data(data, self.front_angle_start, math.pi)
        front_slice_left, front_angles_left = self.slice_data(data, -math.pi, self.front_angle_end)
        front_slice = np.concatenate(front_slice_right, front_slice_left)
        front_angles = np.concatenate(front_angles_right, front_angles_left)

        # Compute average distance to front wall, relative to negative x axis (subtract by pi/2)
        distance_to_front = np.mean(np.multiply(front_slice, np.cos(front_angles - math.pi/2)))

        # Parse LaserScan into slices (side and/or front)
	slice, angles = self.slice_data(data, self.min_side_angle, self.max_side_angle)
        #rospy.loginfo(angles)

        # Combine side and front scans before regression if close enough to front
        if distance_to_front < self.front_dist_threshold:
            # Account for repeats between front and side data (either fix with angle ranges or using numpy)
            slice = np.concatenate(front_slice, slice)
            angles = np.concatenate(front_angles, angles)

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
	command = String()
        left_speed = self.constrain(220 + 60*input, 0, 255)
        right_speed = self.constrain(220 - 60*input, 0, 255)
	command.data = str(int(left_speed)) + ",0," + str(int(right_speed)) + ",0"
        self.cmd_pub.publish(command)
	#rospy.loginfo(rospy.get_caller_id() + 'Slope: %4.3f', m)
	rospy.loginfo(rospy.get_caller_id() + 'Distance: %4.3f', self.distance)
	#rospy.loginfo(rospy.get_caller_id() + 'angle_max: %4.3f', data.angle_min)


    def slice_data(self, data, min_scangle, max_scangle):
	full_data = np.array(data.ranges)
	slice_index_min = int((min_scangle - data.angle_min)/data.angle_increment) 
	slice_index_max = int((max_scangle - data.angle_min)/data.angle_increment)
	angles = np.array([min_scangle + i*data.angle_increment for i in range(slice_index_max - slice_index_min)])
	slice = full_data[slice_index_min: slice_index_max]
        #angles = angles[(slice >= data.range_min) & (slice <= data.range_max)]
        #rospy.loginfo(data.angle_max)
        #rospy.loginfo(data.angle_min)
        angles = np.array([angles[i] for i in range(np.size(angles)) if (slice[i] >= data.range_min and slice[i] <= data.range_max)])
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

    def constrain(self, val, min_val, max_val):
        return min(max_val, max(min_val, val))

if __name__ == "__main__":
    rospy.init_node('wall_follower')
    wall_follower = WallFollower()
    rospy.spin()
