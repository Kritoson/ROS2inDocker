#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <sys/select.h>
#include <string>
#include <sstream>

class MotorControlNode : public rclcpp::Node
{
public:
    MotorControlNode()
        : Node("motor_control_node")
    {
        // -----------------------------
        // PWM ARALIKLARI (DÜZELTİLMİŞ)
        // -----------------------------
        PWM_Center = 1500;
        PWM_1ms    = 1300;
        PWM_2ms    = 1700;

        throttlePWM = PWM_Center;
        rotationPWM = PWM_Center;

        brushState = false;
        vacuumState = false;
        hydrophoreState = false;

        // -----------------------------
        // SERIAL PORT AÇMA
        // -----------------------------
        std::string dev = "/dev/ttyUSB0";
        serial_fd = open_serial(dev.c_str());
        if (serial_fd < 0)
            RCLCPP_ERROR(this->get_logger(), "Serial port açılamadı!");

        // -----------------------------
        // SUBSCRIBERS
        // -----------------------------
        cmd_vel_sub = create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10,
            std::bind(&MotorControlNode::cmd_vel_callback, this, std::placeholders::_1));

        brush_sub = create_subscription<std_msgs::msg::Bool>(
            "/brush", 10,
            std::bind(&MotorControlNode::brush_callback, this, std::placeholders::_1));

        vacuum_sub = create_subscription<std_msgs::msg::Bool>(
            "/vacuum", 10,
            std::bind(&MotorControlNode::vacuum_callback, this, std::placeholders::_1));

        hydrophore_sub = create_subscription<std_msgs::msg::Bool>(
            "/hydrophore", 10,
            std::bind(&MotorControlNode::hydrophore_callback, this, std::placeholders::_1));

        // -----------------------------
        // PUBLISHER
        // -----------------------------
        motor_status_publisher_ = create_publisher<std_msgs::msg::String>("/motor_status", 10);

        // -----------------------------
        // TIMER → Arduino PWM döngüsüne yakın (~71 Hz, 14ms)
        // -----------------------------
        timer_ = create_wall_timer(
            std::chrono::milliseconds(14),
            std::bind(&MotorControlNode::update_loop, this));
    }

private:

    // =============================
    // SERIAL PORT AÇMA
    // =============================
    int open_serial(const char *device)
    {
        int fd = open(device, O_RDWR | O_NOCTTY | O_SYNC);
        if (fd < 0) return -1;

        struct termios tty{};
        if (tcgetattr(fd, &tty) != 0) return -2;

        cfsetospeed(&tty, B115200);
        cfsetispeed(&tty, B115200);

        tty.c_cflag |= (CLOCAL | CREAD);
        tty.c_cflag &= ~CSIZE;
        tty.c_cflag |= CS8;
        tty.c_cflag &= ~PARENB;
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CRTSCTS;

        tty.c_lflag = 0;
        tty.c_iflag = 0;
        tty.c_oflag = 0;

        tty.c_cc[VMIN] = 0;
        tty.c_cc[VTIME] = 1;

        tcsetattr(fd, TCSANOW, &tty);
        return fd;
    }

    // =============================
    // SERIAL'LA MESAJ GÖNDER
    // =============================
    void send_serial(const std::string &msg)
    {
        if (serial_fd < 0) return;
        write(serial_fd, msg.c_str(), msg.size());
        write(serial_fd, "\n", 1);
    }

    // =============================
    // CALLBACK: /cmd_vel
    // =============================
    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        // İLERİ / GERİ
        if (msg->linear.x > 0.1)
            throttlePWM = PWM_2ms;
        else if (msg->linear.x < -0.1)
            throttlePWM = PWM_1ms;
        else
            throttlePWM = PWM_Center;

        // SAĞ / SOL
        if (msg->angular.z > 0.1)
            rotationPWM = PWM_2ms;
        else if (msg->angular.z < -0.1)
            rotationPWM = PWM_1ms;
        else
            rotationPWM = PWM_Center;
    }

    // =============================
    // CALLBACKLER
    // =============================
    void brush_callback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        brushState = msg->data;
    }

    void vacuum_callback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        vacuumState = msg->data;
    }

    void hydrophore_callback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        hydrophoreState = msg->data;
    }

    // =============================
    // ANA UPDATE LOOP
    // =============================
    void update_loop()
    {
        send_serial("T" + std::to_string(throttlePWM));
        send_serial("R" + std::to_string(rotationPWM));

        std_msgs::msg::String msg;
        msg.data = "Throttle=" + std::to_string(throttlePWM) +
                   " Rotation=" + std::to_string(rotationPWM);
        motor_status_publisher_->publish(msg);
    }

private:
    // SERIAL FD
    int serial_fd;

    // PWM DEĞERLERİ
    int PWM_Center;
    int PWM_1ms;
    int PWM_2ms;

    int throttlePWM;
    int rotationPWM;

    bool brushState;
    bool vacuumState;
    bool hydrophoreState;

    // SUBSCRIBERS
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr brush_sub;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr vacuum_sub;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr hydrophore_sub;

    // PUBLISHER
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr motor_status_publisher_;

    // TIMER
    rclcpp::TimerBase::SharedPtr timer_;
};


int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MotorControlNode>());
    rclcpp::shutdown();
    return 0;
}
