#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from nav_msgs.msg import Odometry

from scipy.spatial.transform import Rotation as R
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
        pf_odom_topic = '/ego_racecar/odom' # change to particle filter for actual car
        drive_topic = '/drive'

        self.odom_sub = self.create_subscription(Odometry, pf_odom_topic, self.pose_callback, 10)
        self.drive_pub = self.create_publisher(AckermannDriveStamped, drive_topic, 10)

        self.waypoints = np.genfromtxt('/sim_ws/waypoints/interpolated_tepper.csv', delimiter=',')
        self.waypoints = self.waypoints[:, 0 : 2]

        self.lookahead = 1.2
        self.curr_index = 0
        self.clamp_angle = 20.0


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
    

    def get_rotation_matrix(self, quaternion):
        rotation = R.from_quat(quaternion)
        rotation_matrix = rotation.as_matrix()
        return rotation_matrix[0 : 3, 0 : 3]
    

    def pose_callback(self, pose_msg):
        location = [pose_msg.pose.pose.position.x, pose_msg.pose.pose.position.y]
        quaternion = [pose_msg.pose.pose.orientation.x, pose_msg.pose.pose.orientation.y, pose_msg.pose.pose.orientation.z, pose_msg.pose.pose.orientation.w]
        rot_matrix = self.get_rotation_matrix(quaternion)

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
            angle = np.clip(angle, -np.deg2rad(self.clamp_angle), np.deg2rad(self.clamp_angle))

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