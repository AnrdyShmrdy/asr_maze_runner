#!/usr/bin/env python
from __future__ import absolute_import, division, print_function
import time
import rospy
import math
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Int32
from sensor_msgs.msg import LaserScan
from unitysim.msg import BoundingBox3d
def clamp(value, low=-1.0, high=1.0):
    return min(max(value, low), high)
class RobotController:
    def __init__(self):
        self.healthfinder_sub = rospy.Subscriber("/ponce/healthfinder", BoundingBox3d, self.healthfinder_callback)
        # self.odom_sub = rospy.Subscriber("/ponce/odom", Odometry, self.odom_callback)
        self.cmd_vel_pub = rospy.Publisher("/ponce/cmd_vel", Twist, queue_size=10)
        self.scan_sub = rospy.Subscriber("/ponce/scan", LaserScan, self.scan_callback)
        self.is_initialized = False
        self.rate = rospy.Rate(10)
        self.x_vec_sum = 0
        self.y_vec_sum = 0
        self.tau = math.radians(90) #90 degrees = 1.57 radians
        self.middle_angle = -self.tau
        self.angle_offset = None #will be set once node is initialized
        self.front_angle = None #will be set once node is initialized
        self.front_angle = None #will be set once node is initialized
        self.rear_angle = None #will be set once node is initialized
        self.middle_index = None #will be set once node is initialized
        self.front_index = None #will be set once node is initialized
        self.rear_index = None #will be set once node is initialized 
        self.guard_index = None #will be set once node is initialized
        self.window = None #will be set once node is initialized
        self.angle_increment = None #will be set once node is initialized
        self.angle_min = None #will be set once node is initialized
        self.range_max = None #will be set once node is initialized
        self.ranges = None #will be set once node is initialized
        self.healthfinder_msg = BoundingBox3d()
        self.center_pos_y = 0.0
        self.size_y = 0.0
        self.time = time.time()
    def initialize_runtime_variables(self, msg):
        #variables to set once the node is initialized
        #The reason these variables cannot be set in the constructor is because
        #their values can only be determined after the node is initialized
        self.angle_increment = msg.angle_increment
        self.angle_min = msg.angle_min
        self.range_max = msg.range_max
        self.ranges = msg.ranges
        self.angle_offset = abs(self.angle_min + self.tau)    # angle stuff
        self.front_angle = (-self.tau + self.angle_offset)
        self.rear_angle = (-self.tau - self.angle_offset)
        self.middle_index = self.get_index_from_angle(self.middle_angle) # convert angles to index
        self.front_index = self.get_index_from_angle(self.front_angle)
        self.rear_index = self.get_index_from_angle(self.rear_angle)
        self.guard_index = int(len(self.ranges) / 2)
        self.window = int((math.radians(45)) / self.angle_increment)
    def scan_callback(self, msg):
        if not self.is_initialized:
            self.initialize_runtime_variables(msg)
            self.is_initialized = True
        self.ranges = msg.ranges
    def healthfinder_callback(self, msg):
        self.time = time.time()
        self.healthfinder_msg = msg
    def get_angle_from_index(self, pos):
        #return the angle (in radians) for the laser at the specified index in laser_msg.ranges
        return pos * self.angle_increment + self.angle_min
    def get_index_from_angle(self, angle_in_radians):
        #return the index in laser_msg.ranges that has the given angle (in radians)
        return int((angle_in_radians - self.angle_min)/self.angle_increment)
    def get_front_laser(self):
        return self.ranges[int(len(self.ranges) / 2)]
        
    def get_vec_sums(self):
        ranges = self.ranges
        for i in range(len(self.ranges)):
            theta = self.get_angle_from_index(i)
            x_vec = ranges[i] * math.cos(float(theta)) / self.range_max
            y_vec = ranges[i] * math.sin(float(theta)) / self.range_max
            self.x_vec_sum += x_vec
            self.y_vec_sum += y_vec
        self.x_vec_sum = self.x_vec_sum / len(ranges)
        self.y_vec_sum = self.y_vec_sum / len(ranges)
    def safe_forward(self):
        self.get_vec_sums()
        final_angle = math.atan2(self.y_vec_sum,self.x_vec_sum)
        cmdTwist = Twist()
        cmdTwist.linear.x = self.get_front_laser() / self.range_max
        cmdTwist.angular.z = final_angle
        msg = self.healthfinder_msg
        self.center_pos_y = msg.center.position.y
        self.size_y = msg.size.y
        t0 = time.time()
        if self.center_pos_y < self.size_y and -(self.time - t0) < 1.0:
            print("center pos y: " + str(self.center_pos_y))
            print("size y: " + str(self.size_y))
            print(time.time() - t0)
            cmdTwist.angular.z = self.center_pos_y / 10 * -1
        self.cmd_vel_pub.publish(cmdTwist)
    def right_wall_follow(self):
        ranges = self.ranges    # quick reference to the ranges
        vector = [0, 0.8]       # the movement vector. it starts biased forward
        middle_range = ranges[self.middle_index]     # get the ranges at the indices
        front_range = ranges[self.front_index]
        rear_range = ranges[self.rear_index]
        guard_range = ranges[self.guard_index]
        for i in range(self.guard_index - self.window, self.guard_index + self.window):
            guard_range = min(ranges[i], guard_range)
        if min(guard_range, middle_range, front_range, rear_range) < 0.35:  # turn left and slow down if the wall is too close (use smallest sensor)
            vector[0] = 0.5 * 2	
            vector[1] /= 3.0
        elif min(guard_range, middle_range, front_range, rear_range) > 0.45:  # turn right and slow down if the wall is too far (use smallest sensor)
            vector[0] = -0.5 * 2
            vector[1] /= 2.0
        if rear_range > front_range:  # turn slightly if not straight with the wall
            vector[0] += 0.2 * 2
        else:
            vector[0] += -0.2 * 2
        if min(guard_range, front_range) < 0.35:  # reverse direction if the guard or front sensor see something
            vector[1] *= -1.0
        angle = clamp(vector[0]/1.0)  # limit speed and rotation
        speed = clamp(vector[1]/1.0)
        twist_cmd = Twist()
        twist_cmd.linear.x = speed
        twist_cmd.angular.z = max(min(angle, 1.0), -1.0)
        msg = self.healthfinder_msg
        self.center_pos_y = msg.center.position.y
        self.size_y = msg.size.y
        t0 = time.time()
        if self.center_pos_y < self.size_y and -(self.time - t0) < 1.0:
            print("center pos y: " + str(self.center_pos_y))
            print("size y: " + str(self.size_y))
            twist_cmd.angular.z = self.center_pos_y / 10 * -1
        self.cmd_vel_pub.publish(twist_cmd)
        # print("center pos y: " + str(self.center_pos_y))
        # print("size y: " + str(self.size_y))
        self.rate.sleep()
    def main_loop(self):
        for i in range(10):
            twist_cmd = Twist()
            twist_cmd.linear.x = -0.5
            self.cmd_vel_pub.publish(twist_cmd)
            self.rate.sleep()
        while not rospy.is_shutdown():
            ranges = self.ranges
            min_range = ranges[0]
            for i in range(0, int(len(ranges) / 2)):
                min_range = min(min_range, ranges[i])
            if min_range < 1.0:
                self.right_wall_follow()
            elif min_range > 2.0:
                self.safe_forward()
            
def main(args=None):
    rospy.init_node("Controller",anonymous=True)
    robot_controller = RobotController()
    time.sleep(2) #Sleep to allow time to initialize. Otherwise subscriber might recieve an empty message
    robot_controller.main_loop()

if __name__ == '__main__':
    main()