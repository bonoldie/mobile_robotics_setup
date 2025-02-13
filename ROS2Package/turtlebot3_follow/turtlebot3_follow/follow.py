 #Single scan from a planar laser range-finder
#
# If you have another ranging device with different behavior (e.g. a sonar
# array), please find or create a different message, since applications
# will make fairly laser-specific assumptions about this data

#std_msgs/Header header # timestamp in the header is the acquisition time of
                             # the first ray in the scan.
                             #
                             # in frame frame_id, angles are measured around
                             # the positive Z axis (counterclockwise, if Z is up)
                             # with zero angle being forward along the x axis

#float32 angle_min            # start angle of the scan [rad]
#float32 angle_max            # end angle of the scan [rad]
#float32 angle_increment      # angular distance between measurements [rad]

#float32 time_increment       # time between measurements [seconds] - if your scanner
                             # is moving, this will be used in interpolating position
                             # of 3d points
#float32 scan_time            # time between scans [seconds]

#float32 range_min            # minimum range value [m]
#float32 range_max            # maximum range value [m]

#float32[] ranges             # range data [m]
                             # (Note: values < range_min or > range_max should be discarded)
#float32[] intensities        # intensity data [device-specific units].  If your
                             # device does not provide intensities, please leave
                             # the array empty.

import debugpy

debugpy.listen(('0.0.0.0', 5678))

import rclpy
from rclpy.node import Node
import rclpy.qos

from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan, Imu
import numpy as np

import time
import threading
import math

class Turtlebot3Follow(Node):

    def __init__(self):
        super().__init__('turtlebot3_follow_node')

        # definition of publisher and subscriber object to /cmd_vel and /scan 
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 1)
        self.subscription = self.create_subscription(LaserScan, '/scan', self.laser_callback, rclpy.qos.qos_profile_sensor_data)

        self.angle_min = 0
        #self.angle_increment = 0
        self.linear_vel = 0.0
        self.angular_vel = 0.0
        self.ranges = []
        # list of distance in angles range
        self.range_view = []
        # list of rays angles in angles range
        self.angle_view = []
        self.stop = False

        self.len_ranges=0
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.control_loop)
        self.get_logger().info("node started")

        #parameter for follow objects
        self.max_distance=0.60
        self.min_distance=0.20
        self.max_vel_linear=0.22
        self.max_vel_angular=1.80

    # loop each 0.1 seconds
    def control_loop(self):
        msg_pub = Twist()
        msg_pub.linear.x  = self.linear_vel
        msg_pub.angular.z = self.angular_vel
        self.publisher_.publish(msg_pub)

    def stop_robot(self):
        self.stop = True
        self.get_logger().info("robot stop")

    def debug_print(self, debug_list: list):
    # Format each number in the list to 2 decimal places, including negative numbers
        formatted_str = ', '.join([f'{float(r):.2f}' for r in debug_list])
    # Log the formatted string
        self.get_logger().info(formatted_str)


        
    # called each time new message is published
    def laser_callback(self, msg: LaserScan):
       
        # stop robot when shutdown node
        if self.stop:
            self.linear_vel = 0.0
            self.angular_vel = 0.0

            msg_pub = Twist()
            msg_pub.linear.x  = 0.0
            msg_pub.angular.z = 0.0
            self.publisher_.publish(msg_pub)
            return
        
        #self.range_view.clear()
        #self.angle_view.clear()
        
        # check sanity of ranges array
        #if self.len_ranges < 100:
         #   self.linear_vel = 0.0
          #  self.angular_vel = 0.0
           # return


        #range from 0 a 45° clockwise
        self.range_view= msg.ranges[(len(msg.ranges))-45:(len(msg.ranges)-1)]
        #range from -45 a 0° clockwise
        self.range_view.extend(msg.ranges[0:45])

        np.nan_to_num(self.range_view)
       
        self.angle_view = [0] * len(self.range_view)


        #this vector goes from -pi/4 -> 0 -> pi/4, according to range data disposition
        for index in range(len(self.range_view)//2+1):       
            self.angle_view[index]=self.angle_min+(index*msg.angle_increment)-math.pi/4
            self.angle_view[index+len(self.range_view)//2]=self.angle_min+(index*msg.angle_increment)

            #self.debug_print(self.angle_view)

        min_range_value=min(self.range_view)
        min_index=self.range_view.index(min_range_value)

        min_angle=self.angle_view[min_index]

        self.get_logger().info("min range")
        self.debug_print([min_range_value])
        
        self.get_logger().info("min angle")
        self.debug_print([min_angle])
        

        #calculate robot commands only if the range value falls between mi and max distance

        if(min_range_value>0.1 and min_range_value<0.5):
            self.linear_vel=min_range_value*0.3
            self.angular_vel=(min_angle)*2

            self.get_logger().info("\nvelocity command")
            self.debug_print([self.linear_vel])

            self.get_logger().info("\nangular command")
            self.debug_print([self.angular_vel])
        else:
            self.linear_vel=0.0
            self.angular_vel=0.0

      
        

def main(args=None):
    rclpy.init(args=args)

    turtlebot3_follow_node = Turtlebot3Follow()

    t = threading.Thread(target=rclpy.spin, args=[turtlebot3_follow_node])
    t.start()

    try:
        while rclpy.ok():
            time.sleep(5)
    except KeyboardInterrupt:
        turtlebot3_follow_node.stop_robot()
        
        time.sleep(0.5)

    # Destroy the node explicitly

    turtlebot3_follow_node.destroy_node()
    rclpy.shutdown()

    t.join()


if __name__ == '__main__':
    main()
