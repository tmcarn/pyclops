import rclpy
from rclpy.node import Node

import numpy as np
from scipy.linalg import expm

from std_msgs.msg import Float32MultiArray, Float64
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster


class DiffDriveController(Node):
    def __init__(self):
        super().__init__("diff_drive_controller")

        self.WHEEL_RADIUS = 0.0375
        self.TRACK_WIDTH = 0.1

        self.TORQUE_SCALE = 0.5 # TODO: Update dynamics when adding in motor controller

        # State Variables
        self.T = np.eye(3)
        self.x_pos = 0
        self.y_pos = 0
        self.rot = self.T[:2,:2]

        self.w_left_vel = 0
        self.w_left_pos = 0
        self.w_right_vel = 0
        self.w_right_pos = 0

        # Cmd Variables
        self.left_cmd_vel = 0
        self.right_cmd_vel = 0
        
        self.omega = None

        # Time Variables
        self.prev_time = None
        self.curr_time = None
        
        self.create_subscription(Float32MultiArray, '/cmd_wheel_vel', self.cmd_wheel_vel_callback, 1)
        self.create_subscription(JointState, '/joint_states', self.joint_states_callback, 1)

        self.left_cmd_vel_publisher = self.create_publisher(Float64, '/model/pyclops_robot/joint/left_wheel_joint/cmd_vel', 1)
        self.right_cmd_vel_publisher = self.create_publisher(Float64, '/model/pyclops_robot/joint/right_wheel_joint/cmd_vel', 1)
        
        self.tf_broadcaster = TransformBroadcaster(self)

        self.create_timer(0.1, self.publish_cmd_wheel_vel)


    def compute_twist(self):
        x_dot = (self.WHEEL_RADIUS/2) * (self.w_right_vel + self.w_left_vel)
        theta_dot = (self.WHEEL_RADIUS/self.TRACK_WIDTH) * (self.w_right_vel - self.w_left_vel)

        omega = np.zeros((3,3))
        omega[0,1] = -theta_dot
        # omega[0,2] = x_dot
        omega[1,0] = theta_dot
        omega[1,2] = -x_dot

        self.omega = omega

        return omega

    def cmd_wheel_vel_callback(self, msg:Float32MultiArray):
        self.left_cmd_vel = msg.data[0]
        self.right_cmd_vel = msg.data[1]
        
    def publish_cmd_wheel_vel(self):
        # Publish cmd_wheel_vels
        left_msg = Float64()
        left_msg.data = self.left_cmd_vel
        self.left_cmd_vel_publisher.publish(left_msg)
        
        right_msg = Float64()
        right_msg.data = self.right_cmd_vel
        self.right_cmd_vel_publisher.publish(right_msg)
        
    def joint_states_callback(self, msg:JointState):
        left_idx = msg.name.index('left_wheel_joint')
        right_idx = msg.name.index('right_wheel_joint')

        # Update Joint States
        self.w_left_pos = msg.position[left_idx]    
        self.w_left_vel = msg.velocity[left_idx]
        
        self.w_right_pos = msg.position[right_idx]
        self.w_right_vel = msg.velocity[right_idx]

        self.compute_twist()

        # Calculate dt
        self.curr_time = rclpy.time.Time.from_msg(msg.header.stamp)
        if self.prev_time is None:
            # First callback -> skip integration
            self.prev_time = self.curr_time
            return
        
        dt = (self.curr_time - self.prev_time).nanoseconds / 1e9  # Convert to seconds
    
        self.T = self.T @ expm(dt * self.omega)

        # Update Odometry
        self.x_pos = self.T[0,2]
        self.y_pos = self.T[1,2]
        self.rot = self.T[:2,:2]

        # Publish Odometry
        self.publish_tf()

        self.prev_time = self.curr_time

    def publish_tf(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base_link'
        t.child_frame_id = 'body'
        
        t.transform.translation.x = self.x_pos
        t.transform.translation.y = self.y_pos
        t.transform.translation.z = self.WHEEL_RADIUS

        qx, qy, qz, qw = self.rotation_matrix_to_quaternion_2d(self.rot)
        
        # Convert theta to quaternion
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        
        self.tf_broadcaster.sendTransform(t) 


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