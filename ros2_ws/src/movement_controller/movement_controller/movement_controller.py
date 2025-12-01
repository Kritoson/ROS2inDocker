import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math
import time

class MovementController(Node):
    def __init__(self):
        super().__init__('movement_controller')

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.lidar_sub = self.create_subscription(
            LaserScan, '/scan', self.lidar_callback, 10
        )

        self.cmd = Twist()

        self.forward_speed = 0.30
        self.turn_speed = 0.60   # sola: +, sağa: -
        self.radius_limit = 1.0  # 1 metre yarıçap
        self.wait_until_ts = None   # engel bekleme süresi sonu
        self.pending_turn = None    # "left" veya "right"

        self.timer = self.create_timer(0.01, self.publish_cmd)

        self.get_logger().info("ANGLE-BASED OBSTACLE AVOIDANCE ACTIVE")

    def publish_cmd(self):
        self.cmd_pub.publish(self.cmd)

    def lidar_callback(self, msg: LaserScan):

        angle_min_deg = msg.angle_min * 180.0 / math.pi
        angle_inc_deg = msg.angle_increment * 180.0 / math.pi

        closest_dist = float('inf')
        closest_angle = None

        # ---------------------------------------
        # LIDAR VERİSİ: ARAMAMIZ GEREKEN AÇI ARALIĞI
        # +90 → +180 → -90
        # ---------------------------------------

        for i, r in enumerate(msg.ranges):

            angle_deg = angle_min_deg + i * angle_inc_deg

            # -------------------------
            # S2E Noise Filtering
            # -------------------------
            if math.isnan(r) or math.isinf(r):
                continue
            if r <= 0.05:
                continue
            if r < 0.20:
                continue
            if r > 8.0:
                continue

            # 1 metre yarıçap içinde olmalı
            if r > self.radius_limit:
                continue

            # -------------------------
            # Only accept angles:
            # +90..+180  OR  -180..-90
            # -------------------------
            in_sector = (
                (90 <= angle_deg <= 180) or
                (-180 <= angle_deg <= -90)
            )

            if not in_sector:
                continue

            # En yakın noktayı seç
            if r < closest_dist:
                closest_dist = r
                closest_angle = angle_deg

        # ---------------------------------------
        # CASE: NESNE YOK → DÜZ GİT
        # ---------------------------------------
        if closest_angle is None:
            self.wait_until_ts = None
            self.pending_turn = None
            self.go_straight()
            self.get_logger().info(
                f"Path Clear → Moving Straight"
            )
            return

        # ---------------------------------------
        # CASE: NESNE VAR → STOP + bekle + yönel
        # ---------------------------------------
        now = time.time()

        # Engeli ilk kez gördüysek bekleme sürecini başlat
        if self.wait_until_ts is None:
            self.wait_until_ts = now + 2.0
            self.pending_turn = "left" if closest_angle > 0 else "right"
            self.stop()
            return

        # Bekleme sürüyor
        if now < self.wait_until_ts:
            self.stop()
            return

        # Bekleme bitti → yönel ve süreci sıfırla
        turn_dir = self.pending_turn
        self.wait_until_ts = None
        self.pending_turn = None

        # Açı pozitif → SAĞ tarafta → SOLA kaç
        if turn_dir == "left":
            self.turn_left()
            self.get_logger().info(
                f"Object RIGHT side angle={closest_angle:.1f} → Turning LEFT"
            )
            return

        # Açı negatif → SOL tarafta → SAĞA kaç
        if turn_dir == "right":
            self.turn_right()
            self.get_logger().info(
                f"Object LEFT side angle={closest_angle:.1f} → Turning RIGHT"
            )
            return

    # ----------------------------------------------------
    # Movement helper functions
    # ----------------------------------------------------
    def stop(self):
        self.cmd.linear.x = 0.0
        self.cmd.angular.z = 0.0

    def go_straight(self):
        self.cmd.linear.x = self.forward_speed
        self.cmd.angular.z = 0.0

    def turn_left(self):
        self.cmd.linear.x = 0.0
        self.cmd.angular.z = +self.turn_speed

    def turn_right(self):
        self.cmd.linear.x = 0.0
        self.cmd.angular.z = -self.turn_speed


def main():
    rclpy.init()
    node = MovementController()
    rclpy.spin(node)
    rclpy.shutdown()
