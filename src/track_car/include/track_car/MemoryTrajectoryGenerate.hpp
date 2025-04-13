#pragma once

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include <vector>
#include <fstream>
#include <thread>
#include <atomic>
#include <termios.h>
#include <unistd.h>
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

class MemoryTrajectoryGenerate
{
public:
    MemoryTrajectoryGenerate(rclcpp::Node* node);
    ~MemoryTrajectoryGenerate();
    
    void updateMemoryTrajectory();
    int getState() const { return state_; };
    std::vector<sensor_msgs::msg::NavSatFix> getTrajectory() const;

private:
    void save_gps_data();
    void process_imu();
    void process_gps();
    void start_keyboard_listener();
    void listen_for_keypress();
    char get_keypress();

    rclcpp::Node* node_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr trajectory_pub_;

    std::ofstream gps_file_;
    sensor_msgs::msg::NavSatFix::SharedPtr latest_gps_data_;
    sensor_msgs::msg::Imu::SharedPtr latest_imu_data_;
    std::vector<sensor_msgs::msg::NavSatFix> gps_trajectory_;

    std::atomic<bool> is_recording_;
    std::atomic<bool> new_imu_received_;
    std::atomic<bool> new_gps_received_;

    double last_velocity_;
    double current_velocity_;
    rclcpp::Time start_time_;
    double last_time_;
    double last_saved_time_ = 0.0;  // 上一次保存时间
    double current_heading_;
    int state_; // 0: off, 1: recording, 2: finished

    std::thread keyboard_thread_;
};
