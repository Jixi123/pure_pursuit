#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import numpy as np
from nav_msgs.msg import Odometry 
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from geometry_msgs.msg import PoseStamped
# TODO CHECK: include needed ROS msg type headers and libraries

class PurePursuit(Node):
    """ 
    Implement Pure Pursuit on the car
    This is just a template, you are free to implement your own node!
    """
    def __init__(self):
        super().__init__('pure_pursuit_node')
    
        self.waypoints = np.loadtxt("./src/pure_pursuit/waypoints/path.csv", delimiter = ",")
        self.look_ahead = 0.75
        self.drive_pub = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        self.odo_sub = self.create_subscription(Odometry, '/ego_racecar/odom', self.pose_callback, 10)
        self.get_logger().info("pure pursuit started")
        
    def find_waypoint(self):  
        distances = []
        for waypoint in self.waypoints:
            distance = np.sqrt((waypoint[0]-self.cur_x)**2 + (waypoint[1]-self.cur_y)**2)
            distances.append(distance)
        
        min_index = np.argmin(distances)
        cur_index = min_index + 1
        while(True):
            if(cur_index>=len(distances)):
                cur_index = 0
            if(distances[cur_index] >= self.look_ahead):
                break
            cur_index+=1
        x = self.waypoints[cur_index][0]
        y = self.waypoints[cur_index][1]
        return x,y
        
    def transform(self, x, y, msg):
        o_x = msg.pose.pose.orientation.x 
        o_y = msg.pose.pose.orientation.y
        o_z = msg.pose.pose.orientation.z
        o_w = msg.pose.pose.orientation.w 
        yaw = euler_from_quaternion(o_x, o_y, o_z, o_w)
        
        
        
        return x-self.cur_x, y-self.cur_y
    
    def publish_drive(self):
        
        msg = AckermannDriveStamped()
        msg.drive.steering_angle = self.steering_angle 
        msg.drive.speed = 1.0
        if(abs(self.steering_angle)>0.25):
            msg.drive.speed = 0.5
        self.drive_pub.publish(msg)
        self.get_logger().info(str(msg.drive.steering_angle))

    def pose_callback(self, pose_msg):
        
        self.cur_x = pose_msg.pose.pose.position.x 
        self.cur_y = pose_msg.pose.pose.position.y 
        
        # TODO: find the current waypoint to track using methods mentioned in lecture
        temp_x, temp_y = self.find_waypoint()
        # TODO: transform goal point to vehicle frame of reference
        self.x, self.y = self.transform(temp_x, temp_y, pose_msg)
        # TODO: calculate curvature/steering angle
        self.steering_angle = np.clip(2 * self.y / (self.look_ahead ** 2), -0.35, 0.35)
        # TODO: publish drive message, don't forget to limit the steering angle.
        self.publish_drive()

def main(args=None):
    rclpy.init(args=args)
    print("PurePursuit Initialized")
    pure_pursuit_node = PurePursuit()
    rclpy.spin(pure_pursuit_node)
    pure_pursuit_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
