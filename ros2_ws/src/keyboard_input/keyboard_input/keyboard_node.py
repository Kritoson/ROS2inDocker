import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import sys
import termios
import tty
import select

class KeyboardPublisher(Node):
    """
    Klavyeden gelen tek karakterlik girdileri okur,
    bunları ROS2 üzerinde /keyboard_cmd topiğine publish eder.
    """
    def __init__(self):
        super().__init__('keyboard_publisher')
        self.pub = self.create_publisher(String, '/keyboard_cmd', 10)
        # 50 Hz timer -> sürekli klavye kontrolü
        self.timer = self.create_timer(0.02, self.check_keyboard)

        # Terminal ayarlarını kaydet
        self.stdin_fd = sys.stdin.fileno()
        self.old_settings = termios.tcgetattr(self.stdin_fd)
        tty.setcbreak(self.stdin_fd)

    def check_keyboard(self):
        """
        Non-blocking klavye okuma.
        Eğer bir tuşa basıldıysa ROS topiğine gönder.
        """
        dr, dw, de = select.select([sys.stdin], [], [], 0)
        if dr:
            key = sys.stdin.read(1)
            msg = String()
            msg.data = key
            self.pub.publish(msg)

    def destroy_node(self):
        """
        Terminal ayarlarını geri yükle
        """
        termios.tcsetattr(self.stdin_fd, termios.TCSADRAIN, self.old_settings)
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
