#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <chrono>
#include <termios.h>
#include <unistd.h>
#include <iostream>

class GazeboControlNode : public rclcpp::Node
{
public:
    GazeboControlNode() : Node("gazebo_control_node"), linear_speed_(0.0), angular_speed_(0.0)
    {
        RCLCPP_INFO(this->get_logger(), "GazeboControlNode has been started.");
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/demo/cmd_demo", 10);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), std::bind(&GazeboControlNode::send_velocity, this));
    }

    // Function to read single character from the terminal (non-blocking)
    char get_keypress()
    {
        struct termios oldt, newt;
        char ch;
        tcgetattr(STDIN_FILENO, &oldt);
        newt = oldt;
        newt.c_lflag &= ~(ICANON | ECHO); // Disable canonical mode and echoing
        tcsetattr(STDIN_FILENO, TCSANOW, &newt);
        ch = getchar();
        tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
        return ch;
    }

private:
    void send_velocity()
    {
        //char key = get_keypress();  // Get keypress from the user

        geometry_msgs::msg::Twist msg;

        // Key mappings
        // if (key == 65) {  // Arrow Up (Increase speed)
        //     linear_speed_ += 0.5;
        // }
        // else if (key == 66) {  // Arrow Down (Decrease speed)
        //     linear_speed_ -= 0.5;
        // }
        // else if (key == 67) {  // Arrow Right (Turn right)
        //     angular_speed_ -= 0.1;
        // }
        // else if (key == 68) {  // Arrow Left (Turn left)
        //     angular_speed_ += 0.1;
        // }
        linear_speed_ += 0.01;
        // Set the values for speed and steering
        msg.linear.x = linear_speed_;
        msg.angular.z = angular_speed_;

        cmd_vel_pub_->publish(msg);

        RCLCPP_INFO(this->get_logger(), "Published velocity: linear.x=%.2f, angular.z=%.2f", msg.linear.x, msg.angular.z);
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    double linear_speed_;   // Current linear speed (forward)
    double angular_speed_;  // Current angular speed (steering)
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Starting GazeboControlNode...");
    rclcpp::spin(std::make_shared<GazeboControlNode>());
    rclcpp::shutdown();
    return 0;
}
