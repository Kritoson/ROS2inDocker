import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Int32
from geometry_msgs.msg import Twist


class MovementController(Node):
    def __init__(self):
        super().__init__('movement_controller')

        self.obstacle_detected = False
        self.obstacle_angle = None

        # Subscribers
        self.create_subscription(Bool, '/obstacle_detected', self.detect_cb, 10)
        self.create_subscription(Int32, '/obstacle_angle', self.angle_cb, 10)

        # Publisher
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Timer (10 Hz)
        self.create_timer(0.1, self.control_loop)

        self.get_logger().info("MovementController initialized.")

    def detect_cb(self, msg):
        self.obstacle_detected = msg.data

    def angle_cb(self, msg):
        self.obstacle_angle = msg.data

    def control_loop(self):
        cmd = Twist()

        # Engel yoksa → düz ileri
        if not self.obstacle_detected:
            cmd.linear.x = 0.2
            cmd.angular.z = 0.0
            self.cmd_pub.publish(cmd)
            return

        # Engel varsa → açıya göre davran
        angle = self.obstacle_angle

        if angle is None:
            return

        # Tam ön engel (–10°…+10° alanını da kritik kabul ettim)
        if -10 <= angle <= 10:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.cmd_pub.publish(cmd)
            self.get_logger().info("Front obstacle → STOP")
            return

        # Engel sağda → sola dön
        if angle < -10:
            cmd.linear.x = 0.0
            cmd.angular.z = +0.5
            self.cmd_pub.publish(cmd)
            self.get_logger().info(f"Obstacle right ({angle}°) → TURN LEFT")
            return

        # Engel solda → sağa dön
        if angle > 10:
            cmd.linear.x = 0.0
            cmd.angular.z = -0.5
            self.cmd_pub.publish(cmd)
            self.get_logger().info(f"Obstacle left ({angle}°) → TURN RIGHT")
            return


def main(args=None):
    rclpy.init(args=args)
    node = MovementController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
