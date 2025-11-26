import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int32, Bool


class FrontObstacleDetector(Node):
    def __init__(self):
        super().__init__('front_obstacle_detector')

        self.radius = 1.0
        self.angle_limit = math.radians(90)  # sadece -90 ... +90

        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        self.angle_pub = self.create_publisher(Int32, '/obstacle_angle', 10)
        self.detect_pub = self.create_publisher(Bool, '/obstacle_detected', 10)

        self.get_logger().info("FrontObstacleDetector (RAW angle version) initialized.")

    @staticmethod
    def normalize(angle):
        """normalize to [-pi, +pi]"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def scan_callback(self, msg: LaserScan):

        closest_dist = None
        closest_angle = None

        for i, dist in enumerate(msg.ranges):
            if math.isinf(dist) or math.isnan(dist):
                continue

            # HAM AÇI (sürücüden gelen)
            raw_angle = msg.angle_min + i * msg.angle_increment

            # FİZİKSEL LİDAR YÖNÜ DÜZELTMESİ (ön taraf = 0 derece)
            phys_angle = raw_angle + math.pi  # 180° offset

            # normalize et
            phys_angle = self.normalize(phys_angle)

            # sadece -90 .. +90 arasını kabul et
            if phys_angle < -self.angle_limit or phys_angle > self.angle_limit:
                continue

            # mesafe filtresi
            if dist <= self.radius:
                if closest_dist is None or dist < closest_dist:
                    closest_dist = dist
                    closest_angle = phys_angle

        if closest_angle is None:
            self.detect_pub.publish(Bool(data=False))
            return

        # dereceye çevir → int'e yuvarla
        angle_deg = int(round(math.degrees(closest_angle)))

        self.angle_pub.publish(Int32(data=angle_deg))
        self.detect_pub.publish(Bool(data=True))

        self.get_logger().info(f"Obstacle angle: {angle_deg}°")


def main(args=None):
    rclpy.init(args=args)
    node = FrontObstacleDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

