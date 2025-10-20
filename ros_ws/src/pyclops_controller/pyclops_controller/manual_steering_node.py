import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist

class ManualSteering(Node):
    def __init__(self):
        super().__init__("manual_steering_controller")

        self.TRACK_WIDTH = 0.1

        # Control parameters
        self.lin_vel = 0  # m/s
        self.ang_vel = 0  # rad/s

        self.lin_scale_factor = 10
        self.ang_scale_factor = 50
        
        # Current wheel velocities
        self.left_vel = 0.0
        self.right_vel = 0.0

        self.create_subscription(Twist, "/cmd_vel", self.update_wheel_commands, 1)
        self.wheel_command_publisher = self.create_publisher(Float32MultiArray, '/cmd_wheel_vel', 1)
        
    def inverse_kinematics(self, lin_vel, ang_vel):
        left_vel = (lin_vel - (ang_vel * self.TRACK_WIDTH / 2.0)) 
        right_vel = (lin_vel + (ang_vel * self.TRACK_WIDTH / 2.0))

        return left_vel, right_vel

    def update_wheel_commands(self, msg:Twist):
        lin_vel = msg.linear.x * self.lin_scale_factor
        ang_vel = msg.angular.z * self.ang_scale_factor

        self.left_vel, self.right_vel = self.inverse_kinematics(lin_vel, ang_vel)

        msg = Float32MultiArray()
        msg.data = [self.left_vel, self.right_vel]
        
        self.wheel_command_publisher.publish(msg)



def main(args=None):
    rclpy.init(args=args)
    node = ManualSteering()
    rclpy.spin(node)
    rclpy.shutdown()



if __name__ == "__main__":
    main()