import rclpy
from rclpy.node import Node

import numpy as np
from scipy.linalg import expm

from std_msgs.msg import String, Float32MultiArray


class DiffDriveController(Node):
    def __init__(self):
        super().__init__("diff_drive_controller")

        self.WHEEL_RADIUS = 0.1
        self.TRACK_WIDTH = 0.2

        # State Variables
        self.T = np.eye(3)
        self.x_pos = 0
        self.y_pos = 0
        self.rot = self.T[:2,:2]

        self.w_left = 1
        self.w_right = 1.03

        self.omega = None
        
        self.create_subscription(Float32MultiArray, '/wheel_commands', self.update_position, 1)

        self.position_publisher = self.create_publisher(String, '/position', 1)


    def compute_twist(self):
        x_dot = (self.WHEEL_RADIUS/2) * (self.w_right + self.w_left)
        theta_dot = (self.WHEEL_RADIUS/self.TRACK_WIDTH) * (self.w_right - self.w_left)

        omega = np.zeros((3,3))
        omega[0,1] = -theta_dot
        omega[0,2] = x_dot
        omega[1,0] = theta_dot

        self.omega = omega

        return omega

    def update_position(self, msg:Float32MultiArray):

        dt = 0.1

        self.w_left = msg.data[0]
        self.w_right = msg.data[1]

        self.compute_twist() 

        self.T = self.T @ expm(dt * self.omega)

        # Update State
        self.x_pos = self.T[0,2]
        self.y_pos = self.T[1,2]
        self.rot = self.T[:2,:2]

        msg = String()
        msg.data = f'X Pos: {self.x_pos}, Y Pos: {self.y_pos}'

        self.position_publisher.publish(msg)

        return self.x_pos, self.y_pos, self.rot

    def update_wheel_speed(self, w_left, w_right):
        self.w_left = w_left
        self.w_right = w_right


def main(args=None):
    rclpy.init(args=args)
    node = DiffDriveController()
    rclpy.spin(node)
    rclpy.shutdown()



if __name__ == "__main__":
    main()