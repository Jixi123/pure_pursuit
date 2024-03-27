#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
import numpy as np

# TODO CHECK: include needed ROS msg type headers and libraries

class PurePursuit(Node):
    """ 
    Implement Pure Pursuit on the car
    This is just a template, you are free to implement your own node!
    """
    def __init__(self):
        super().__init__('pure_pursuit_node')
        # TODO: create ROS subscribers and publishers
        pf_odom_topic = '/pf/viz/inferred_pose' # change to particle filter for actual car
        drive_topic = '/drive'

        self.odom_sub = self.create_subscription(PoseStamped, pf_odom_topic, self.pose_callback, 10)
        self.drive_pub = self.create_publisher(AckermannDriveStamped, drive_topic, 10)
        self.path_pub = self.create_publisher(Marker,'/visualization_marker',10)
        self.waypoints = np.genfromtxt('/home/team5/f1tenth_ws/src/pure_pursuit/waypoints_test.csv', delimiter=',')
        self.waypoints = self.waypoints[:, 0 : 2]

        self.lookahead = 1.5
        self.curr_index = 0
        self.clamp_angle = 15.0

    def display_marker(self, current_waypoint, g, r):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.ns = "marker"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.id = 0
        marker.type = 2
        marker.action = 0
        marker.pose.position.x = current_waypoint[0]
        marker.pose.position.y = current_waypoint[1]
        marker.pose.position.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.25
        marker.scale.z = 0.1
        marker.scale.y = 0.25
        marker.color.a = 1.0
        marker.color.g = g
        marker.color.r = r
        self.path_pub.publish(marker)
        
    def get_current_waypoint(self, location):
        finished = False
        num_points = self.waypoints.shape[0]
        if (self.curr_index >= num_points):
            finished = True
            return None, finished
    
        distances = (self.waypoints[:, 0] - location[0]) ** 2 + (self.waypoints[:, 1] - location[1]) ** 2

        for i in range(num_points):
            if distances[i] < self.lookahead:
                self.curr_index = i

        waypoint_prev = self.waypoints[self.curr_index]
        waypoint_next = self.waypoints[min(self.curr_index + 1, num_points - 1)]

        current_waypoint = (waypoint_prev + waypoint_next) / 2
        self.display_marker(current_waypoint, 0.0, 1.0)

        return current_waypoint, finished


    def get_speed(self, angle, finished):
        if finished:
            return 0.0

        abs_angle = np.abs(angle)
        if abs_angle >= np.deg2rad(15):
            speed = 0.75
        elif abs_angle >= np.deg2rad(10):
            speed = 1.25
        else:
            speed = 1.75
        return speed
    

    def pose_callback(self, pose_msg):
        x = pose_msg.pose.orientation.x
        y = pose_msg.pose.orientation.y
        z = pose_msg.pose.orientation.z
        w = pose_msg.pose.orientation.w

        self.display_marker([x, y], 1.0, 0.0)

        rot_matrix = np.array([[1-2*y**2 - 2*z**2, 2*x*y - 2*z*w, 2*x*z + 2*y*w], 
                                [2*x*y + 2*z*w, 1-2*x**2 - 2*z**2, 2*y*z - 2*x*w],
                                [2*x*z - 2*y*w, 2*y*z + 2*x*w, 1-2*x**2 - 2*y**2]])
        
        
        location = [pose_msg.pose.position.x, pose_msg.pose.position.y]

        # find the current waypoint to track using methods mentioned in lecture
        target_waypoint, finished = self.get_current_waypoint(location)

        angle = 0.0

        # transform goal point to vehicle frame of reference
        if not finished:
            goal_point_wrt_world = target_waypoint - location
            goal_point_wrt_world = np.append(goal_point_wrt_world, 0.0)
            goal_point_wrt_body = np.matmul(np.linalg.inv(rot_matrix), goal_point_wrt_world)

            # calculate curvature/steering angle
            angle = (2 * goal_point_wrt_body[1]) / (self.lookahead ** 2)
            angle = -np.clip(angle, -np.deg2rad(self.clamp_angle), np.deg2rad(self.clamp_angle))

        # TODO: publish drive message, don't forget to limit the steering angle.
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.steering_angle = angle
        drive_msg.drive.speed = self.get_speed(angle, finished)

        self.drive_pub.publish(drive_msg)
        print("steering at angle: ", np.rad2deg(angle))


def main(args=None):
    rclpy.init(args=args)
    print("PurePursuit Initialized")
    pure_pursuit_node = PurePursuit()
    rclpy.spin(pure_pursuit_node)

    pure_pursuit_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
