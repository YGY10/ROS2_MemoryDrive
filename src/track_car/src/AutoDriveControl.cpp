#include "track_car/AutoDriveControl.hpp"
#include <cmath>
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

AutoDriveControl::AutoDriveControl(rclcpp::Node *node)
    : node_(node), auto_drive_requested_(false), state_(0)
{
    RCLCPP_INFO(node_->get_logger(), "🚙 AutoDriveControl initialized.");

    gps_subscription_ = node_->create_subscription<sensor_msgs::msg::NavSatFix>(
        "/gps/data", 10, [this](const sensor_msgs::msg::NavSatFix::SharedPtr msg)
        { current_pose_ = *msg; });
    imu_subscription_ = node_->create_subscription<sensor_msgs::msg::Imu>(
        "/imu/data", 10, [this](const sensor_msgs::msg::Imu::SharedPtr msg)
        { updateImUData(msg); });
    cmd_vel_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>("/demo/cmd_demo", 10);
    start_keyboard_listener();
    log_start_time_ = node_->now();

    drive_log_file_.open("auto_drive_log.txt", std::ios::out | std::ios::trunc);
    if (!drive_log_file_.is_open())
    {
        RCLCPP_ERROR(node_->get_logger(), "❌ Failed to open auto_drive_log.txt");
    }
    else
    {
        drive_log_file_ << "Latitude: , Longitude: , Altitude: , Time: , Velocity: , Heading:\n";
    }
}
AutoDriveControl::~AutoDriveControl()
{
    if (drive_log_file_.is_open())
    {
        drive_log_file_.close();
    }
    if (keyboard_thread_.joinable())
    {
        keyboard_thread_.join();
    }
}

void AutoDriveControl::updateImUData(const sensor_msgs::msg::Imu::SharedPtr latest_imu_data)
{
    if (!latest_imu_data)
        return;

    double current_time = node_->now().seconds();
    double delta_time = 0.02;

    tf2::Quaternion q(
        latest_imu_data->orientation.x,
        latest_imu_data->orientation.y,
        latest_imu_data->orientation.z,
        latest_imu_data->orientation.w);

    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    // 将加速度从世界坐标系转换到自车坐标系
    double acceleration_x_world = latest_imu_data->linear_acceleration.x;
    double acceleration_y_world = latest_imu_data->linear_acceleration.y;

    // 将加速度投影到自车坐标系
    double acceleration_x_car = acceleration_x_world * cos(yaw) + acceleration_y_world * sin(yaw); // 车体x方向
    // double acceleration_y_car = -acceleration_x_world * sin(yaw) + acceleration_y_world * cos(yaw); // 车体y方向

    // 计算车辆在自车坐标系下的加速度
    double real_acceleration = acceleration_x_car; // 只取x方向的加速度

    // 更新速度
    current_velocity_ += +real_acceleration * delta_time;

    // 更新偏航角
    current_heading_ = yaw;
}

void AutoDriveControl::setMemoryTrajectory(const std::vector<sensor_msgs::msg::NavSatFix> &trajectory)
{
    trajectory_ = trajectory;
}

void AutoDriveControl::setAutoDriveFlag(bool flag)
{
    auto_drive_requested_ = flag;
}

double AutoDriveControl::haversineDistance(double lat1, double lon1, double lat2, double lon2)
{
    const double R = 6371000.0; // 地球半径（米）
    double dLat = (lat2 - lat1) * M_PI / 180.0;
    double dLon = (lon2 - lon1) * M_PI / 180.0;
    double a = std::pow(std::sin(dLat / 2), 2) +
               std::cos(lat1 * M_PI / 180.0) * std::cos(lat2 * M_PI / 180.0) *
                   std::pow(std::sin(dLon / 2), 2);
    double c = 2 * std::atan2(std::sqrt(a), std::sqrt(1 - a));
    return R * c;
}

void AutoDriveControl::find_nearest_point(const sensor_msgs::msg::NavSatFix &current_pose,
                                          const std::vector<sensor_msgs::msg::NavSatFix> &trajectory,
                                          size_t &nearest_index)
{
    double min_distance = std::numeric_limits<double>::max();
    for (size_t i = 0; i < trajectory.size(); ++i)
    {
        double distance = haversineDistance(current_pose.latitude, current_pose.longitude,
                                            trajectory[i].latitude, trajectory[i].longitude);
        if (distance < min_distance)
        {
            min_distance = distance;
            nearest_index = i;
        }
    }
}

void AutoDriveControl::updateTargetPos()
{

    size_t nearest_index = 0;
    find_nearest_point(current_pose_, trajectory_, nearest_index);
    RCLCPP_INFO(node_->get_logger(), "Nearest point index: %zu", nearest_index);
    if (nearest_index + 1 < trajectory_.size())
    {
        const auto &p_nearest = trajectory_[nearest_index];
        const auto &p_next = trajectory_[nearest_index + 1];
        RCLCPP_INFO(node_->get_logger(), "Nearest point: (%.6f, %.6f), Next point: (%.6f, %.6f)", p_nearest.latitude, p_nearest.longitude, p_next.latitude, p_next.longitude);
        // 以当前位置为原点，转换为世界坐标
        GeographicLib::LocalCartesian geo_converter_;
        geo_converter_.Reset(current_pose_.latitude, current_pose_.longitude, 0);
        double x_nearest_ENU, y_nearest_ENU, z_nearest_ENU;
        geo_converter_.Forward(p_nearest.latitude, p_nearest.longitude, p_nearest.altitude, x_nearest_ENU, y_nearest_ENU, z_nearest_ENU);
        double x_next_ENU, y_next_ENU, z_next_ENU;
        geo_converter_.Forward(p_next.latitude, p_next.longitude, p_next.altitude, x_next_ENU, y_next_ENU, z_next_ENU);
        RCLCPP_INFO(node_->get_logger(), "Nearest point ENU: (%.2f, %.2f), Next point ENU: (%.2f, %.2f)", x_nearest_ENU, y_nearest_ENU, x_next_ENU, y_next_ENU);
        // 转换到自车坐标系下
        double x_nearest = -(x_nearest_ENU * cos(-current_heading_) - y_nearest_ENU * sin(-current_heading_));
        double y_nearest = -(-x_nearest_ENU * sin(-current_heading_) + y_nearest_ENU * cos(-current_heading_));
        double x_next = -(x_next_ENU * cos(-current_heading_) - y_next_ENU * sin(-current_heading_));
        double y_next = -(-x_next_ENU * sin(-current_heading_) + y_next_ENU * cos(-current_heading_));
        RCLCPP_INFO(node_->get_logger(), "Nearest point: (%.2f, %.2f), Next point: (%.2f, %.2f)", x_nearest, y_nearest, x_next, y_next);
        // 最近点和前一个点的方向向量
        double x1 = x_next - x_nearest;
        double y1 = y_next - y_nearest;
        double distance1 = std::sqrt(x1 * x1 + y1 * y1);
        // 最近点->自车位置的向量
        double x2 = -x_nearest;
        double y2 = -y_nearest;
        double distance2 = std::sqrt(x2 * x2 + y2 * y2);
        // 将自车位置投影到最近点和前一个点连线的向量上
        // 自车位置在最近点前方
        // 投影长度占比
        double projection_ratio = (x1 * x2 + y1 * y2) / (x1 * x1 + y1 * y1);
        // 计算投影点坐标
        double x_projection = x_nearest + projection_ratio * x1;
        double y_projection = y_nearest + projection_ratio * y1;
        // 计算预瞄点的坐标，向前0.1s,
        target_x_ = x_projection + 0.1 * (x_next - x_nearest);
        target_y_ = y_projection + 0.1 * (y_next - y_nearest);
        // 计算预瞄点的速度
        target_speed_mps_ = 0.5; // 先给0.5，先测试横向
        // 计算预瞄点的航向角
        target_yaw_ = atan2(y_next - y_nearest, x_next - x_nearest);
    }
}

void AutoDriveControl::pure_pursuit_control()
{
    // 计算前轮转角
    double L = 2.5; // 轴距
    double Ld = sqrt(target_x_ * target_x_ + target_y_ * target_y_);
    if (Ld < 0.3)
        Ld = 0.3; // 防止除以0或角度剧烈变化
    double delta = atan2(2 * L * target_y_, Ld * Ld);
    // 计算车速
    double speed = current_velocity_;
    // 计算前轮转角速度
    angular_speed_ = speed / L * tan(delta);
}

void AutoDriveControl::pid_control_lon()
{
    linear_speed_ = target_speed_mps_; // TODO
}

void AutoDriveControl::update(const std::vector<sensor_msgs::msg::NavSatFix> &trajectory, const int &MemoryTrajectoryState)
{
    double now = node_->now().seconds();
    double elapsed = now - log_start_time_.seconds();
    switch (state_)
    {
    case 0:
        if (trajectory.empty())
        {
            RCLCPP_WARN(node_->get_logger(), "❌ No trajectory data available.");
            break;
        }
        if (MemoryTrajectoryState != 2)
        {
            RCLCPP_WARN(node_->get_logger(), "❌ Memory trajectory not finish.");
            break;
        }
        if (get_distance_from_2pos(current_pose_, trajectory[0]) > 10)
        {
            RCLCPP_WARN(node_->get_logger(), "❌ Too far from start point.");
            break;
        }
        if (get_distance_from_2pos(trajectory[0], trajectory[trajectory.size() - 1]) < 10)
        {
            RCLCPP_WARN(node_->get_logger(), "❌ Start and end points too close.");
            break;
        }
        if (!auto_drive_requested_)
        {
            RCLCPP_WARN(node_->get_logger(), "❌ Auto-drive not requested.");
            return;
        }
        state_ = 1;
        load_trajectory_ = false;
        RCLCPP_INFO(node_->get_logger(), "✅ Auto-drive off ->active!");
        break;
    case 1:
        if (!auto_drive_requested_)
        {
            RCLCPP_WARN(node_->get_logger(), "❌ Auto-drive not requested.");
            state_ = 0;
            break;
        }
        if (haversineDistance(current_pose_.latitude, current_pose_.longitude,
                              trajectory[trajectory.size() - 1].latitude, trajectory[trajectory.size() - 1].longitude) < 5)
        {
            RCLCPP_INFO(node_->get_logger(), "✅ Arrive end point! Auto-drive finished!");
            state_ = 3;
            finish_time_ = node_->now();
            break;
        }
        if (!load_trajectory_)
        {
            setMemoryTrajectory(trajectory);
            load_trajectory_ = true;
        }
        updateTargetPos();
        RCLCPP_INFO(node_->get_logger(), " 👀 target_x= %.2f, target_y= %2.f, target_yaw= %2.f", target_x_, target_y_, target_yaw_);
        // 有了目标点的坐标和速度，下面进行控制，先用纯跟踪算法
        pure_pursuit_control(); // 横向控制
        pid_control_lon();      // 纵向控制
        // 发布控制指令
        {
            geometry_msgs::msg::Twist msg;
            msg.linear.x = linear_speed_;
            msg.angular.z = angular_speed_;
            // 发布消息
            cmd_vel_pub_->publish(msg);
            RCLCPP_INFO(node_->get_logger(), "🚗 Auto-driving: linear.x=%.2f, angular.z=%.2f", msg.linear.x, msg.angular.z);
        }
        drive_log_file_ << std::fixed << std::setprecision(8)
                        << "Latitude: " << current_pose_.latitude
                        << ", Longitude: " << current_pose_.longitude
                        << ", Altitude: " << current_pose_.altitude
                        << ", Time: " << std::setprecision(2) << elapsed
                        << ", Velocity: " << std::setprecision(5) << current_velocity_
                        << ", Heading: " << std::setprecision(2) << current_heading_
                        << std::endl;
        break;
    case 3:
        if (!auto_drive_requested_)
        {
            RCLCPP_WARN(node_->get_logger(), "😃 Auto-drive finish -> off");
            state_ = 0;
            break;
        }
        // 如果完成时间超过5秒，自动关闭
        if (node_->now().seconds() - finish_time_.seconds() > 5)
        {
            RCLCPP_INFO(node_->get_logger(), "✅ Auto-drive finished!");
            state_ = 0;
            break;
        }
        break;
    default:
        break;
    }
}

void AutoDriveControl::start_keyboard_listener()
{
    keyboard_thread_ = std::thread(&AutoDriveControl::listen_for_keypress, this);
}

void AutoDriveControl::listen_for_keypress()
{
    while (rclcpp::ok())
    {
        char key = get_keypress();
        if (key == '/')
        {
            auto_drive_requested_ = !auto_drive_requested_;
            if (auto_drive_requested_)
            {
                RCLCPP_INFO(node_->get_logger(), "🚗 AutoDrive ON");
            }
            else
            {
                RCLCPP_INFO(node_->get_logger(), "🅿️ AutoDrive OFF");
            }
        }
    }
}

char AutoDriveControl::get_keypress()
{
    struct termios oldt, newt;
    char ch;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    ch = getchar();
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    return ch;
}
