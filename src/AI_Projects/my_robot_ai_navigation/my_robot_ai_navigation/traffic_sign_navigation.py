#!/usr/bin/env python3
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from nav2_simple_commander.robot_navigator import BasicNavigator
from tf_transformations import quaternion_from_euler


class TrafficSignNavigation(Node):
    """
    ROS2 node that navigates a robot through an environment,
    reacting dynamically to detected traffic signs using Nav2.
    """

    def __init__(self):
        super().__init__('traffic_sign_navigation')
        # Define spatial coordinates (x, y, yaw) and timing parameters
        self.declare_parameter('initial_pose', [0.0, 0.0, 0.0])
        self.declare_parameter('inspection_pose', [0.5, 0.0, 0.0])
        self.declare_parameter('left_waypoint', [1.0, 0.5, 0.0])
        self.declare_parameter('right_waypoint', [1.0, -0.5, 0.0])
        self.declare_parameter('final_pose', [3.0, 0.0, 1.57])
        self.declare_parameter('stop_duration', 3.0)

        self.initial_pose = self.get_parameter('initial_pose').value
        self.inspection_pose = self.get_parameter('inspection_pose').value
        self.left_waypoint = self.get_parameter('left_waypoint').value
        self.right_waypoint = self.get_parameter('right_waypoint').value
        self.final_pose = self.get_parameter('final_pose').value
        self.stop_duration = self.get_parameter('stop_duration').value

        self.navigator = BasicNavigator()
        self.detected_sign = None
        self.accept_sign = False

        # TODO 1: Create a String subscriber for /traffic_sign.
        # Arguments: message type, topic name, callback function, queue size.
        self.sign_subscriber = self.create_subscription(
            String,
            '/traffic_sign',
            self.sign_callback,
            10)

    def sign_callback(self, msg):
        # TODO 2: While accept_sign is True, store the sign in lower case.
        # Messages that arrive while the robot is still travelling to the
        # inspection pose are ignored.
        if self.accept_sign:
            self.detected_sign = msg.data.strip().lower()
            self.get_logger().info(f'Traffic sign received: {self.detected_sign}')

    def create_pose(self, values):
        x, y, yaw = values
        # Convert orientation from Euler angles (yaw) to quaternions
        qx, qy, qz, qw = quaternion_from_euler(0.0, 0.0, yaw)

        pose = PoseStamped()
        pose.header.frame_id = 'map'  # Reference coordinate frame
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        return pose

    def go_to(self, values, name):
        self.get_logger().info(f'Going to {name}')
        self.navigator.goToPose(self.create_pose(values))

        while not self.navigator.isTaskComplete():
            rclpy.spin_once(self, timeout_sec=0.1)

    def wait_for_sign(self):
        self.detected_sign = None
        self.accept_sign = True
        self.get_logger().info('Waiting for a traffic sign on /traffic_sign ...')

        # TODO 3: Process callbacks until detected_sign is not None.
        # spin_once lets ROS call sign_callback when a message arrives.
        
        # Keep spinning the node to process incoming callbacks until a sign is detected
        while self.detected_sign is None and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)

        self.accept_sign = False
        return self.detected_sign

    def run(self):
        self.navigator.setInitialPose(self.create_pose(self.initial_pose))
        self.navigator.waitUntilNav2Active()

        self.go_to(self.inspection_pose, 'inspection pose')
        sign = self.wait_for_sign()

        # TODO 4: Implement left, right, stop and unknown-sign behaviour.
        
        # Evaluate the detected sign and execute the corresponding maneuver
        # 'left'  = "no right turn" sign -> take the left route
        if sign == 'left':
            self.get_logger().info('Sign "left": taking the left route')
            self.go_to(self.left_waypoint, 'left waypoint')
            self.go_to(self.final_pose, 'final pose')
 
         # 'right' = "no left turn" sign  -> take the right route
        elif sign == 'right':
            self.get_logger().info('Sign "right": taking the right route')
            self.go_to(self.right_waypoint, 'right waypoint')
            self.go_to(self.final_pose, 'final pose')
 
        elif sign == 'stop':
            self.get_logger().info(
                f'Sign "stop": waiting {self.stop_duration} seconds')
            time.sleep(self.stop_duration)
            self.go_to(self.final_pose, 'final pose')
 
        else:
            self.get_logger().warn(
                f'Unknown sign "{sign}": staying at inspection pose 2')
 
 
def main(args=None):
    rclpy.init(args=args)
    node = TrafficSignNavigation()
 
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()
 
 
if __name__ == '__main__':
    main()