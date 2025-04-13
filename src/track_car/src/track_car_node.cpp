#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include <fstream>
#include <vector>
#include <termios.h>
#include <unistd.h>
#include <iostream>
#include <thread>
#include <atomic>
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

class TrackCarNode : public rclcpp::Node
{
public:
    TrackCarNode() : Node("MemoryTrajectory_node"), is_recording_(false), last_velocity_(0.0), last_time_(0.0)
    {
        RCLCPP_INFO(this->get_logger(), "🚗 MemoryTrajectory function has been started.");

        // Subscribe to GPS data
        gps_subscription_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
            "/gps/data", 10, std::bind(&TrackCarNode::gps_callback, this, std::placeholders::_1));

        // Subscribe to IMU data (to get acceleration)
        imu_subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu/data", 10, std::bind(&TrackCarNode::imu_callback, this, std::placeholders::_1));

        // Create a timer to save GPS data every second
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500), std::bind(&TrackCarNode::save_gps_data, this));

        // Create a publisher for the trajectory
        trajectory_pub_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("/track/trajectory", 10);

        // Open file to store GPS data
        gps_file_.open("gps_data.txt", std::ios::out | std::ios::app);
        if (!gps_file_.is_open())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to open file to save GPS data.");
        }

        // Start keyboard listener thread
        start_keyboard_listener();
    }

    ~TrackCarNode()
    {
        // Cleanup resources
        if (gps_file_.is_open())
        {
            gps_file_.close();
        }

        if (keyboard_thread_.joinable())
        {
            keyboard_thread_.join();
        }
    }

private:
    // Start keyboard listener thread
    void start_keyboard_listener()
    {
        keyboard_thread_ = std::thread(&TrackCarNode::listen_for_keypress, this);
    }

    // Listen for keyboard input
    void listen_for_keypress()
    {
        while (rclcpp::ok())
        {
            char key = get_keypress();
            if (key == '.')
            { // Toggle recording state when '.' is pressed
                is_recording_ = !is_recording_;
                if (is_recording_)
                {
                    RCLCPP_INFO(this->get_logger(), "Started recording GPS data.");
                }
                else
                {
                    RCLCPP_INFO(this->get_logger(), "Stopped recording GPS data.");
                }
            }
        }
    }

    // Get keyboard input
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

    // Timer callback function, called every second
    void save_gps_data()
    {
        if (latest_gps_data_ && is_recording_)
        {
            // Save data to file
            gps_file_ << "Latitude: " << latest_gps_data_->latitude
                      << ", Longitude: " << latest_gps_data_->longitude
                      << ", Altitude: " << latest_gps_data_->altitude
                      << ", Time: " << this->now().seconds()
                      << ", Velocity: " << current_velocity_
                      << ", Heading: " << current_heading_
                      << std::endl;

            // Store to global container
            gps_trajectory_.push_back(*latest_gps_data_);

            // Publish trajectory with velocity
            trajectory_pub_->publish(*latest_gps_data_);

            RCLCPP_INFO(this->get_logger(), "Saved GPS data: Latitude = %f, Longitude = %f, Velocity = %f",
                        latest_gps_data_->latitude, latest_gps_data_->longitude, current_velocity_);
        }
    }

    // GPS data callback
    void gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
    {
        latest_gps_data_ = msg; // Save the latest GPS data
    }

    // IMU data callback (to get acceleration)
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        double current_time = this->now().seconds();
        double delta_time = 0.2;

        if (delta_time > 0) // Update velocity if sufficient time has passed
        {
            double acceleration = msg->linear_acceleration.x; // Assuming x-axis acceleration

            // Print acceleration and delta_time for debugging
            RCLCPP_INFO(this->get_logger(), "Acceleration: %f, Delta Time: %f", acceleration, delta_time);

            // Use a simple Euler integration: v = v0 + a * dt
            current_velocity_ = last_velocity_ + acceleration * delta_time;

            // Print the calculated velocity for debugging
            RCLCPP_INFO(this->get_logger(), "Calculated Velocity: %f", current_velocity_);

            // Update the last velocity and time
            last_velocity_ = current_velocity_;
        }
        // 提取四元数
        tf2::Quaternion q(
            msg->orientation.x,
            msg->orientation.y,
            msg->orientation.z,
            msg->orientation.w);

        // 转换为欧拉角
        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

        // yaw 即为 heading（单位：弧度），可转为角度输出
        double heading_deg = yaw * 180.0 / M_PI;
        RCLCPP_INFO(this->get_logger(), "Heading (deg): %f", heading_deg);

        // 可以保存为成员变量，用于文件记录
        current_heading_ = heading_deg;

        last_time_ = current_time; // Update last time
    }

    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr trajectory_pub_;
    rclcpp::TimerBase::SharedPtr timer_;                      // Timer
    std::ofstream gps_file_;                                  // File stream for saving GPS data
    sensor_msgs::msg::NavSatFix::SharedPtr latest_gps_data_;  // Latest GPS data
    std::vector<sensor_msgs::msg::NavSatFix> gps_trajectory_; // Container to store trajectory
    std::atomic<bool> is_recording_;                          // Recording status (on/off)
    double last_velocity_;                                    // Last velocity (for integration)
    double current_velocity_;                                 // Current velocity
    double last_time_;                                        // Last timestamp (for time delta calculation)
    double current_heading_;                                  // Current heading (in degrees)
    std::thread keyboard_thread_;                             // Keyboard listener thread
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TrackCarNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
