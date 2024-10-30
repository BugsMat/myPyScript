import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
import time
import math

class TurtleController(Node):
    def __init__(self):
        super().__init__('turtle_controller')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_subscriber_ = self.create_subscription(Odometry, '/odom', self.update_pose, 10)

        # Set QoS profile for LiDAR sensor to BEST_EFFORT
        qos_profile = QoSProfile(depth=10)
        qos_profile.reliability = QoSReliabilityPolicy.BEST_EFFORT

        # Subscribe to LD-08 LiDAR data
        self.lidar_subscriber_ = self.create_subscription(LaserScan, '/ld_08_scan', self.update_scan, qos_profile)

        self.pose = None  # To store Odometry position data
        self.lidar_data = None  # To store latest LiDAR data
        self.wall_threshold = 0.5  # Distance threshold to detect walls
        self.rotation_angle = 80.0  # Angle to rotate (in degrees)
        self.rotation_speed = 1.0  # Speed of rotation (radians per second)
        self.forward_speed = 0.2  # Adjusted speed for real robot
        self.backward_speed = -0.1  # Adjusted speed for real robot
        self.state = 'move_forward'
        self.state_start_time = None
        self.rotation_direction = 1.0  # 1.0 for left, -1.0 for right
        self.get_logger().info("Node for turtle_controller has started")

    def update_pose(self, msg):
        # Update the robot's pose from Odometry data
        self.pose = msg.pose.pose

    def update_scan(self, msg):
        # Update the LiDAR scan data
        self.lidar_data = msg.ranges

    def is_obstacle_ahead(self):
        if self.lidar_data is None:
            return False
        # Check front sectors for obstacles within threshold
        front_distances = self.lidar_data[0:10] + self.lidar_data[-10:]  # Example range sectors directly ahead
        return any(distance < self.wall_threshold for distance in front_distances if distance > 0)

    def control_loop(self):
        if self.pose is None or self.lidar_data is None:
            return  # No pose or LiDAR data yet
        
        twist = Twist()
        if self.state == 'move_forward':
            twist.linear.x = self.forward_speed
            twist.angular.z = 0.0
            if self.is_obstacle_ahead():
                self.get_logger().info("Obstacle detected ahead! Switching to 'move_backward' state.")
                self.state = 'move_backward'
                self.state_start_time = time.time()
        
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
