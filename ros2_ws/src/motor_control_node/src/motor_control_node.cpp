#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <iostream>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <string>
#include <sys/select.h>

class MotorControlNode : public rclcpp::Node
{
public:
    MotorControlNode()
    : Node("motor_control_node")
    {
        declare_parameter<std::string>("serial_port", "/dev/ttyUSB0");
        std::string dev = get_parameter("serial_port").as_string();

        // PWM merkez ve limit değerleri
        PWM_Center = 434;
        PWM_2ms = 584;
        PWM_1ms = 293;

        // Başlangıç değerleri
        throttlePWM = PWM_Center;
        rotationPWM = PWM_Center;
        brushState = false;
        vacuumState = false;
        hydrophoreState = false;

        // SERIAL PORT AÇMA
        serial_fd = open_serial(dev.c_str());
        if (serial_fd < 0) {
            RCLCPP_ERROR(get_logger(), "Serial açılamadı: %s", dev.c_str());
        }

        // =====================================
        // ROS SUBSCRIBERS
        // =====================================

        // /cmd_vel -> twist mesajından hız kontrol
        cmd_vel_sub = create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10,
            std::bind(&MotorControlNode::cmd_vel_callback, this, std::placeholders::_1));

        // Brush
        brush_sub = create_subscription<std_msgs::msg::Bool>(
            "/brush", 10,
            std::bind(&MotorControlNode::brush_callback, this, std::placeholders::_1));

        // Vacuum
        vacuum_sub = create_subscription<std_msgs::msg::Bool>(
            "/vacuum", 10,
            std::bind(&MotorControlNode::vacuum_callback, this, std::placeholders::_1));

        // Hydrophore
        hydrophore_sub = create_subscription<std_msgs::msg::Bool>(
            "/hydrophore", 10,
            std::bind(&MotorControlNode::hydrophore_callback, this, std::placeholders::_1));

        // YENİ EKLENDİ → Klavye subscriber
        keyboard_sub = create_subscription<std_msgs::msg::String>(
            "/keyboard_cmd", 10,
            std::bind(&MotorControlNode::keyboard_callback, this, std::placeholders::_1));

        // TELEMETRY PUBLISHER
        status_pub = create_publisher<std_msgs::msg::String>("/motor_status", 10);

        // 50 Hz loop timer
        timer = create_wall_timer(
            std::chrono::milliseconds(20),
            std::bind(&MotorControlNode::update_loop, this));

        RCLCPP_INFO(get_logger(), "MotorControlNode başladı.");
    }

private:

    // ============================================================
    // SERIAL PORT AÇMA
    // ============================================================
    int open_serial(const char *dev)
    {
        int fd = open(dev, O_RDWR | O_NOCTTY | O_NONBLOCK);
        if (fd < 0) return -1;

        struct termios tty{};
        if (tcgetattr(fd, &tty) != 0) return -1;

        cfsetospeed(&tty, B115200);
        cfsetispeed(&tty, B115200);

        tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
        tty.c_iflag = 0;
        tty.c_oflag = 0;
        tty.c_lflag = 0;

        tty.c_cc[VMIN]  = 0;
        tty.c_cc[VTIME] = 1;

        tcsetattr(fd, TCSANOW, &tty);
        return fd;
    }

    void send_serial(const std::string &s)
    {
        if (serial_fd < 0) return;
        std::string msg = s;
        if (msg.back() != '\n') msg += "\n";
        write(serial_fd, msg.c_str(), msg.size());
    }

    std::string read_serial_line()
    {
        if (serial_fd < 0) return "";

        fd_set rfds;
        struct timeval tv{0,0};
        FD_ZERO(&rfds);
        FD_SET(serial_fd, &rfds);

        if (select(serial_fd+1, &rfds, NULL, NULL, &tv) <= 0)
            return "";

        std::string out;
        char c;
        while (true) {
            int r = read(serial_fd, &c, 1);
            if (r <= 0) break;
            if (c == '\n') break;
            if (c != '\r') out.push_back(c);
        }
        return out;
    }

    // ============================================================
    // ROS CALLBACKS
    // ============================================================

    // Twist mesajı ile hız kontrolü
    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        if (msg->linear.x > 0.1)
            throttlePWM = PWM_2ms;
        else if (msg->linear.x < -0.1)
            throttlePWM = PWM_1ms;
        else
            throttlePWM = PWM_Center;

        if (msg->angular.z > 0.1)
            rotationPWM = PWM_2ms;
        else if (msg->angular.z < -0.1)
            rotationPWM = PWM_1ms;
        else
            rotationPWM = PWM_Center;
    }

    void brush_callback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        brushState = msg->data;
        send_serial(std::string("B") + (brushState ? "1" : "0"));
    }

    void vacuum_callback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        vacuumState = msg->data;
        send_serial(std::string("V") + (vacuumState ? "1" : "0"));
    }

    void hydrophore_callback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        hydrophoreState = msg->data;
        send_serial(std::string("H") + (hydrophoreState ? "1" : "0"));
    }

    // ============================================================
    // KLAVYE → PWM – PERİFERİK KONTROL CALLBACK
    // ============================================================
    void keyboard_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        std::string key = msg->data;

        // -----------------------
        // İLERİ – GERİ
        // -----------------------
        if (key == "w") { throttlePWM = PWM_2ms; }
        else if (key == "s") { throttlePWM = PWM_1ms; }
        else if (key == "x") { throttlePWM = PWM_Center; }

        // -----------------------
        // SAĞ – SOL
        // -----------------------
        else if (key == "a") { rotationPWM = PWM_1ms; }
        else if (key == "d") { rotationPWM = PWM_2ms; }
        else if (key == "z") { rotationPWM = PWM_Center; }

        // -----------------------
        // PERİFERİKLER
        // -----------------------
        else if (key == "b") {
            brushState = !brushState;
            send_serial(std::string("B") + (brushState ? "1":"0"));
        }
        else if (key == "v") {
            vacuumState = !vacuumState;
            send_serial(std::string("V") + (vacuumState ? "1":"0"));
        }
        else if (key == "h") {
            hydrophoreState = !hydrophoreState;
            send_serial(std::string("H") + (hydrophoreState ? "1":"0"));
        }

        RCLCPP_INFO(get_logger(), "Klavye komutu alındı: %s", key.c_str());
    }

    // ============================================================
    // ANA LOOP (50 Hz)
    // ============================================================
    void update_loop()
    {
        // PWM çıkışı
        send_serial("T" + std::to_string(throttlePWM));
        send_serial("R" + std::to_string(rotationPWM));

        // Telemetry okuma
        std::string line = read_serial_line();
        if (!line.empty()) {
            auto msg = std_msgs::msg::String();
            msg.data = line;
            status_pub->publish(msg);
        }
    }

private:

    int serial_fd;

    // PWM threshold değerleri
    int PWM_Center, PWM_2ms, PWM_1ms;
    int throttlePWM, rotationPWM;

    // Periferik durumları
    bool brushState, vacuumState, hydrophoreState;

    // ROS subscriber/publisher/timer
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr brush_sub;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr vacuum_sub;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr hydrophore_sub;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr keyboard_sub;

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub;
    rclcpp::TimerBase::SharedPtr timer;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MotorControlNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
