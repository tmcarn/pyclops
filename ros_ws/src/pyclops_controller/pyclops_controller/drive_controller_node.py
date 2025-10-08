import rclpy
from rclpy.node import Node

import numpy as np
from scipy.linalg import expm

from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster


class DiffDriveController(Node):
    def __init__(self):
        super().__init__("diff_drive_controller")

        self.WHEEL_RADIUS = 0.1
        self.TRACK_WIDTH = 0.1

        # State Variables
        self.T = np.eye(3)
        self.x_pos = 0
        self.y_pos = 0
        self.rot = self.T[:2,:2]

        self.w_left_vel = 0
        self.w_left_pos = 0

        self.w_right_vel = 0
        self.w_right_pos = 0
        
        self.omega = None
        
        self.create_subscription(Float32MultiArray, '/wheel_commands', self.update_position, 1)
        self.joint_state_publisher = self.create_publisher(JointState, '/joint_states', 1)
        self.tf_broadcaster = TransformBroadcaster(self)


    def compute_twist(self):
        x_dot = (self.WHEEL_RADIUS/2) * (self.w_right_vel + self.w_left_vel)
        theta_dot = (self.WHEEL_RADIUS/self.TRACK_WIDTH) * (self.w_right_vel - self.w_left_vel)

        omega = np.zeros((3,3))
        omega[0,1] = -theta_dot
        omega[0,2] = x_dot
        omega[1,0] = theta_dot

        self.omega = omega

        return omega

    def update_position(self, msg:Float32MultiArray):

        dt = 0.1

        self.w_left_vel = msg.data[0]
        self.w_right_vel = msg.data[1]

        self.compute_twist() 

        self.T = self.T @ expm(dt * self.omega)

        # Update State
        self.x_pos = self.T[0,2]
        self.y_pos = self.T[1,2]
        self.rot = self.T[:2,:2]

        self.publish_wheel_position()
        self.publish_tf()

    def publish_tf(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'body'
        
        t.transform.translation.x = self.x_pos
        t.transform.translation.y = self.y_pos
        t.transform.translation.z = 0.0

        qx, qy, qz, qw = self.rotation_matrix_to_quaternion_2d(self.rot)
        
        # Convert theta to quaternion
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        
        self.tf_broadcaster.sendTransform(t) 

    def publish_wheel_position(self):
        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.name = ['left_wheel', 'right_wheel']

        # TODO: Update wheel positions by integrating velocity
        joint_state.position = [self.w_left_pos, self.w_right_pos]
        
        self.joint_state_publisher.publish(joint_state)

    def rotation_matrix_to_quaternion_2d(self, rot):
        """Convert 2x2 rotation matrix to quaternion"""
        theta = np.arctan2(rot[1, 0], rot[0, 0])
        qx = 0.0
        qy = 0.0
        qz = np.sin(theta / 2.0)
        qw = np.cos(theta / 2.0)
        return qx, qy, qz, qw



def main(args=None):
    rclpy.init(args=args)
    node = DiffDriveController()
    rclpy.spin(node)
    rclpy.shutdown()



if __name__ == "__main__":
    main()