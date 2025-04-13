#pragma once

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include <vector>
#include <memory>
#include <GeographicLib/LocalCartesian.hpp>
#include "geometry_msgs/msg/twist.hpp"
#include <atomic>
#include <termios.h>
#include <fstream>

class AutoDriveControl
{
public:
    AutoDriveControl(rclcpp::Node *node);
    ~AutoDriveControl();

    void setMemoryTrajectory(const std::vector<sensor_msgs::msg::NavSatFix> &trajectory);
    void setAutoDriveFlag(bool flag);

    void update(const std::vector<sensor_msgs::msg::NavSatFix> &trajectory, const int &MemoryTrajectoryState); // 主逻辑入口，每个周期调用一次
    int getState() const { return state_; }
    double get_distance_from_2pos(const sensor_msgs::msg::NavSatFix &pos1, const sensor_msgs::msg::NavSatFix &pos2)
    {
        return haversineDistance(pos1.latitude, pos1.longitude, pos2.latitude, pos2.longitude);
    }
    void updateImUData(const sensor_msgs::msg::Imu::SharedPtr msg);
    void updateTargetPos();
    void find_nearest_point(const sensor_msgs::msg::NavSatFix &current_pose, const std::vector<sensor_msgs::msg::NavSatFix> &trajectory, size_t &nearest_index);
    void pure_pursuit_control();
    void pid_control_lon();

private:
    void start_keyboard_listener();
    void listen_for_keypress();
    char get_keypress();

    double haversineDistance(double lat1, double lon1, double lat2, double lon2); // 计算两GPS点距离

    // 成员变量
    rclcpp::Node *node_;
    std::vector<sensor_msgs::msg::NavSatFix> trajectory_; // 轨迹数据
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
    bool auto_drive_requested_ = false; // 是否按下了“自动驾驶”按钮
    int state_;                         // 0: 不满足启动条件，1: 正在执行，3: 执行完成
    double target_y_;
    double target_x_;
    double target_yaw_;
    double target_speed_mps_;
    rclcpp::Time finish_time_;
    bool load_trajectory_ = false; // 是否加载轨迹

    // 仿真控制量
    double angular_speed_;
    double linear_speed_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    std::thread keyboard_thread_;
    std::ofstream drive_log_file_; // 文件输出流
    rclcpp::Time log_start_time_;  // 记录起始时间

    // 条件判定
    const size_t min_required_points_ = 10;
    const double min_start_end_dist_ = 5.0; // 起点和终点距离不能太近（米）
    const double max_start_offset_ = 10.0;  // 当前与起点距离不能太远（米）

    // 当前状态
    sensor_msgs::msg::NavSatFix current_pose_;
    double current_heading_;
    double current_velocity_;
};
