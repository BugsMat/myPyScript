import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import time
import math

class TurtleController(Node):
    def __init__(self):
        super().__init__('turtle_controller')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscriber_ = self.create_subscription(Odometry, '/odom', self.update_pose, 10)
        self.timer = self.create_timer(0.1, self.control_loop)
        self.pose = None  # To store Odometry position data
        self.wall_threshold = 0.5
        self.rotation_angle = 80.0  # Angle to rotate (in degrees)
        self.rotation_speed = 1.0  # Speed of rotation (radians per second)
        self.forward_speed = 0.2  # Adjusted speed for real robot
        self.backward_speed = -0.1  # Adjusted speed for real robot
        self.minimum_y = -2.0  # Boundary min y
        self.maximum_y = 2.0  # Boundary max y
        self.minimum_x = -2.0  # Boundary min x
        self.maximum_x = 2.0  # Boundary max x
        self.state = 'move_forward'
        self.state_start_time = None
        self.rotation_direction = 1.0  # 1.0 for left, -1.0 for right
        self.get_logger().info("Node for turtle_controller has started")

    def update_pose(self, msg):
        # Update the robot's pose from Odometry data
        self.pose = msg.pose.pose

    def control_loop(self):
        if self.pose is None:
            return  # No pose data yet

        twist = Twist()

        if self.state == 'move_forward':
            twist.linear.x = self.forward_speed
            twist.angular.z = 0.0

            if self.pose.position.x < self.minimum_x + self.wall_threshold:
                self.get_logger().info(f"Approaching left wall at x={self.pose.position.x}")
                self.state = 'move_backward'
                self.state_start_time = time.time()
                twist.linear.x = self.backward_speed

            elif self.pose.position.x > self.maximum_x - self.wall_threshold:
                self.get_logger().info(f"Approaching right wall at x={self.pose.position.x}")
                self.state = 'move_backward'
                self.state_start_time = time.time()
                twist.linear.x = self.backward_speed

            elif self.pose.position.y < self.minimum_y + self.wall_threshold:
                self.get_logger().info(f"Approaching bottom wall at y={self.pose.position.y}")
                self.state = 'move_backward'
                self.state_start_time = time.time()
                twist.linear.x = self.backward_speed

            elif self.pose.position.y > self.maximum_y - self.wall_threshold:
                self.get_logger().info(f"Approaching top wall at y={self.pose.position.y}")
                self.state = 'move_backward'
                self.state_start_time = time.time()
                twist.linear.x = self.backward_speed

        elif self.state == 'move_backward':
            twist.linear.x = self.backward_speed
            twist.angular.z = 0.0

            # Move backward for a short duration
            if time.time() - self.state_start_time > 1.0:
                self.state = 'rotate'
                self.state_start_time = time.time()
                twist.linear.x = 0.0
                twist.angular.z = self.rotation_speed * self.rotation_direction

        elif self.state == 'rotate':
            twist.linear.x = 0.0
            twist.angular.z = self.rotation_speed * self.rotation_direction

            # Calculate the required time for rotation
            elapsed_time = time.time() - self.state_start_time
            rotation_angle_radians = math.radians(self.rotation_angle)
            required_time = rotation_angle_radians / abs(self.rotation_speed)

            if elapsed_time > required_time:
                self.get_logger().info("Rotation complete. Moving forward.")
                self.state = 'move_forward'

        self.publisher_.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    turtle_controller = TurtleController()
    
    try:
        rclpy.spin(turtle_controller)
    except KeyboardInterrupt:
        pass  # Handle interrupt gracefully
    finally:
        turtle_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
