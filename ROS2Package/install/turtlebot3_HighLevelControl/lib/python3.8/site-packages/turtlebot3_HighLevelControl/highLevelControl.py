import rclpy
from rclpy.node import Node
import rclpy.qos

from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan, Imu
import numpy as np
import math

import time
import threading

class Turtlebot3HighLevelControl(Node):

    def __init__(self):
        super().__init__('turtlebot3_HighLevelControl_node')

        # definition of publisher and subscriber object to /cmd_vel and /scan 
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 1)
        self.subscription = self.create_subscription(LaserScan, '/scan', self.laser_callback, rclpy.qos.qos_profile_sensor_data)
        
        # initial state of FSM
        self.state_ = 0

        #initialization dict of lidar regions
        #key:value
        self.regions = {
            'front': 0,
            'right': 0,
            'left': 0,
        }
        # definition of dict with state of FSM
        self.state_dict_ = {
            0: 'find the wall',
            1: 'align left',
            2: 'follow the wall',
        }
        # velocity command
        self.msg = Twist()

        # distance threshold to the wall
        self.th = 0.12

        timer_period = 0.2  # seconds

        self.timer = self.create_timer(timer_period, self.control_loop)

        self.front_region_readings= []
        self.right_region_readings= []

    def debug_print(self, debug_input):
    # Check if the input is a list (iterable), a float, or a string
        if isinstance(debug_input, list):
            # Format each number in the list to 2 decimal places
            formatted_str = ', '.join([f'{float(r):.2f}' for r in debug_input])
        elif isinstance(debug_input, (float, int)):
            # If the input is a float or int, format it directly
            formatted_str = f'{debug_input:.2f}'
        else:
            # If it's a string, just print the string as is
            formatted_str = str(debug_input)
        # Log the formatted string
        self.get_logger().info(formatted_str)


    # loop each 0.1 seconds
    def control_loop(self):
    
        # actions for states 
        if self.state_ == 0:
            self.find_wall()
        elif self.state_ == 1:
            self.align_left()
        elif self.state_ == 2:
            self.follow_the_wall()
            pass
        else:
            self.get_logger().info("unknown state!")

        #self.debug_print(self.msg)
        
        self.publisher_.publish(self.msg)

            
    # laser scanner callback
    def laser_callback(self, laser_msg: LaserScan):

        angle_min= laser_msg.angle_min

        front_start_idx1= int((math.radians(315)-angle_min)/laser_msg.angle_increment)
        front_end_idx1= int((math.radians(360)-angle_min)/laser_msg.angle_increment)
        front_start_idx2= int((math.radians(0)-angle_min)/laser_msg.angle_increment)
        front_end_idx2= int((math.radians(45)-angle_min)/laser_msg.angle_increment)

        left_start_idx= int((math.radians(60)-angle_min)/laser_msg.angle_increment)
        left_end_idx= int((math.radians(120)-angle_min)/laser_msg.angle_increment)

        right_start_idx= int((math.radians(240)-angle_min)/laser_msg.angle_increment)
        right_end_idx= int((math.radians(300)-angle_min)/laser_msg.angle_increment)

        


        self.front_region_readings = laser_msg.ranges[front_start_idx1:front_end_idx1] + laser_msg.ranges[front_start_idx2:front_end_idx2]
        self.right_region_readings = laser_msg.ranges [right_start_idx:right_end_idx]

        # populate the lidar reagions with minimum distance 
        # where you find <ranges> you have to read from msg the desired interval
        # I suggesto to do parametric the ranges in this way you don't have to change the value for the real robot 
        #key,value
        self.regions = {
        'front': min(min(min(laser_msg.ranges[front_start_idx1:front_end_idx1]), min(laser_msg.ranges[front_start_idx2:front_end_idx2])),10),
        'left':  min(min(laser_msg.ranges[left_start_idx:left_end_idx]), 10),
        'right':  min(min(laser_msg.ranges[right_start_idx:right_end_idx]), 10),
        }

        self.get_logger().info("front ")
        self.debug_print(self.regions['front'])
        self.get_logger().info("left ")
        self.debug_print(self.regions['left'])
        self.get_logger().info("right ")
        self.debug_print(self.regions['right'])
        self.get_logger().info('\n')

        # function where are definied the rules for the change state
        self.take_action()


    def take_action(self):
 
        # you have to implement the if condition usign the lidar regions and threshold
        # you can add or remove statement if you prefer
        # call change_state function with the state index to enable the change state

        #E1 statement for state 1 2
        if  (self.state_ in [0,1,2]) and self.regions['right']>(self.th*1.1) and self.regions['front']>(self.th*1.1):
            self.change_state(0)

        
        #E2 statement for state 0 2
        elif (self.state_ in [0,1,2]) and (self.regions['front']<self.th and self.regions['left']>self.th and self.regions['right']<self.th) or \
                (self.regions['front']<self.th and self.regions['left']<self.th and self.regions['right']<self.th) or\
                              (self.regions['front']<self.th and self.regions['left']<self.th):
                self.change_state(1)

            #else:
             #   self.get_logger().info("\n NOT A WALL")

        #E3 statement for state 0 1
        elif  (self.state_ in [0,1,2]) and self.regions['front']>self.th and self.regions['left']>self.th and self.regions['right']<self.th:
            #if all(value < self.front_region_readings[45]+1 for value in self.front_region_readings):
             #   self.get_logger().info("wall readings found")
            self.change_state(2)
           
        else:
            self.get_logger().info("no case matched")
            self.change_state(self.state_)
            
    # function to update state
    # don't modify the function
    def change_state(self, state):
        if state is not self.state_:
            self.state_ = state

        print('Wall follower - [%s] - %s' % (state, self.state_dict_[state]))


    # action to find the wall, move forward and wall side to find the wall
    def find_wall(self):
                    
            self.msg.linear.x= 0.03
            self.msg.angular.z=-0.0
 
        # write velocity commands using the class variable self.msg
        
    # action to torn left, move forward and left side
    def align_left(self):
        self.msg.linear.x=0.0
        self.msg.angular.z=0.4
        # write velocity commands using the class variable self.msg
    
    # action to follow the wall, move forward 
    def follow_the_wall(self):

        error= 2.5*((self.th*0.85)-(sum(self.right_region_readings[50:60]))/10)
        self.msg.linear.x= np.clip(0.06+(0.3*error),0.02,0.06)

        self.msg.angular.z= error

        #print('%s,\n%s' % (self.msg.linear.x,self.msg.angular.z))

        # write velocity commands using the class variable self.msg
        



def main(args=None):
    rclpy.init(args=args)

    turtlebot3_HighLevelControl_node = Turtlebot3HighLevelControl()

    rclpy.spin(turtlebot3_HighLevelControl_node)


    turtlebot3_HighLevelControl_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
