#!/usr/bin/env python2
import numpy as np
import rospy
from rospy.numpy_msg import numpy_msg
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped

class WallFollower:
    SCAN_TOPIC = rospy.get_param("wall_follower/scan_topic")
    DRIVE_TOPIC = rospy.get_param("wall_follower/drive_topic")
    SIDE = rospy.get_param("wall_follower/side")
    VELOCITY = rospy.get_param("wall_follower/velocity")
    DESIRED_DISTANCE = rospy.get_param("wall_follower/desired_distance")

    def __init__(self):
        rospy.loginfo("Node started")

        # Set up some variables
        self.scan = None
        self.min_index = 0
        self.max_index = 0

        # Set up publishers and subscribers
        self.pub = rospy.Publisher(self.DRIVE_TOPIC, AckermannDriveStamped, queue_size=1)
        self.pub_scan = rospy.Publisher("~relevant_scan_data", LaserScan, queue_size=1)
        rospy.Subscriber(self.SCAN_TOPIC, LaserScan, self.cb)

        # Publish at a constant rate
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.scan is not None:
                self.output()
            rate.sleep()

    def cb (self,msg):
        # Save the laserscan message
        self.scan = msg

    def output(self):
        '''
        Clips scan data to the relevant ranges, then filters for points that are less than twice the minimum 
        distance detected. Polyfit that data to find the current distance and orientation from wall. Sum the
        angle error and distance error, map it on a sigmoid function from (-1,1), and output a steering angle.
        '''
        # Set up output
        output = AckermannDriveStamped()
        output.drive.speed = self.VELOCITY
        output.drive.steering_angle = 0

        # Clip the data 
        relevant_scan_data = self.get_clipped_scan()

        # for each point, determine if close enough to be relevant, then convert to Cartesian and polyfit
        wall_pts_x = []
        wall_pts_y = []
        for i in range(len(relevant_scan_data)):
            r = relevant_scan_data[i]
            if r < min(relevant_scan_data)*2:
                theta = (i+self.min_index)*self.scan.angle_increment + self.scan.angle_min
                (x, y) = (r*np.cos(theta), r*np.sin(theta))
                wall_pts_x.append(x)
                wall_pts_y.append(y)

        # Get slope and offset of points relative to car
        m, b = np.polyfit(wall_pts_x,wall_pts_y, 1)

        # Use line from polyfit to get orientation and distance from wall
        wall_theta = np.arctan(m) # desired to be 0
        distance_from_wall = np.abs(b/np.sqrt(m**2+1)) # desired to be 1

        # PID error_angle and error_distance
        dist_error = distance_from_wall - self.DESIRED_DISTANCE
        angle_error = self.SIDE * wall_theta
        # rospy.loginfo("distance_from_wall: " + str(distance_from_wall))
        command = dist_error + angle_error

        # Map onto sigmoid function
        z = np.exp(command)
        output.drive.steering_angle = self.SIDE * (z-1)/(z+1) * np.pi/2

        self.pub.publish(output)

    def get_clipped_scan(self, visualize=False):
        ''' 
        Clips scan data to relevant side. Only adds front scan data if a wall is detected in the front
        '''
        # Set up indices to get front scan data
        front_angle_start = -np.pi/3
        front_angle_end = np.pi/3
        front_angle_index_start = int((front_angle_start - self.scan.angle_min)/self.scan.angle_increment)
        front_angle_index_end = int((front_angle_end - self.scan.angle_min)/self.scan.angle_increment)

        # Get the front scan data
        front_scan_data = np.array(self.scan.ranges[front_angle_index_start:front_angle_index_end])

        # Get average distance to the front wall
        front_thetas = np.array([i*self.scan.angle_increment+self.scan.angle_min for i in range(front_angle_index_start, front_angle_index_end)])
        distance_to_front = np.mean(np.multiply(front_scan_data, np.cos(front_thetas)))

        # Set up indices to get side scan data
        side_angle_min = -np.pi*2/3
        side_angle_max = -np.pi*1/12

        # Change indices to other direction if desired
        if self.SIDE == 1: # Left
            side_angle_min = np.pi*1/12
            side_angle_max = np.pi*2/3

        # Get the final indices for the laserscan range we're looking at
        self.min_index = int((side_angle_min - self.scan.angle_min)/self.scan.angle_increment)
        self.max_index = int((side_angle_max - self.scan.angle_min)/self.scan.angle_increment)

        # If there's a wall near front, include the front
        if np.mean(front_scan_data) < self.DESIRED_DISTANCE*self.VELOCITY:
            if self.SIDE == -1:
                self.max_index = front_angle_index_end

            elif self.SIDE == 1:
                self.min_index = front_angle_index_start

        # Publish clipped scan data if desired
        if visualize:
            scan_out = LaserScan()
            scan_out.header.frame_id = 'laser'
            scan_out.angle_min = self.min_index * self.scan.angle_increment + self.scan.angle_min
            scan_out.angle_max = self.max_index * self.scan.angle_increment + self.scan.angle_min
            scan_out.angle_increment = self.scan.angle_increment
            scan_out.range_min = self.scan.range_min
            scan_out.range_max = self.scan.range_max
            scan_out.ranges = relevant_scan_data
            self.pub_scan.publish(scan_out)

        # Return clipped scan data as np array
        return np.array(self.scan.ranges[self.min_index:self.max_index])


if __name__ == "__main__":
    rospy.init_node('wall_follower')
    wall_follower = WallFollower()
    rospy.spin()
