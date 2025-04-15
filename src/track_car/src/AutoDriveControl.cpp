#include "track_car/AutoDriveControl.hpp"
#include <cmath>
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"

AutoDriveControl::AutoDriveControl(rclcpp::Node *node)
    : node_(node), auto_drive_requested_(false), state_(0) {
    RCLCPP_INFO(node_->get_logger(), "🚙 AutoDriveControl initialized.");

    gps_subscription_ = node_->create_subscription<sensor_msgs::msg::NavSatFix>(
        "/gps/data", 10,
        [this](const sensor_msgs::msg::NavSatFix::SharedPtr msg) { current_pose_ = *msg; });
    imu_subscription_ = node_->create_subscription<sensor_msgs::msg::Imu>(
        "/imu/data", 10,
        [this](const sensor_msgs::msg::Imu::SharedPtr msg) { updateImUData(msg); });
    cmd_vel_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>("/demo/cmd_demo", 10);
    start_keyboard_listener();
    log_start_time_ = node_->now();

    drive_log_file_.open("auto_drive_log.txt", std::ios::out | std::ios::trunc);
    if (!drive_log_file_.is_open()) {
        RCLCPP_ERROR(node_->get_logger(), "❌ Failed to open auto_drive_log.txt");
    } else {
        drive_log_file_ << "Latitude: , Longitude: , Altitude: , Time: , Velocity: , Heading:\n";
    }
}
AutoDriveControl::~AutoDriveControl() {
    // 停止车辆
    geometry_msgs::msg::Twist msg;
    msg.linear.x = 0.0;
    msg.angular.z = 0.0;
    cmd_vel_pub_->publish(msg);
    if (drive_log_file_.is_open()) {
        drive_log_file_.close();
    }

    if (trajectory_log_file_.is_open()) {
        trajectory_log_file_.close();
    }
    if (keyboard_thread_.joinable()) {
        keyboard_thread_.join();
    }
}

void AutoDriveControl::updateImUData(const sensor_msgs::msg::Imu::SharedPtr latest_imu_data) {
    if (!latest_imu_data) return;

    double current_time = node_->now().seconds();
    double delta_time = 0.1;

    tf2::Quaternion q(latest_imu_data->orientation.x, latest_imu_data->orientation.y,
                      latest_imu_data->orientation.z, latest_imu_data->orientation.w);

    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    // 将加速度从世界坐标系转换到自车坐标系
    double acceleration_x_world = latest_imu_data->linear_acceleration.x;
    double acceleration_y_world = latest_imu_data->linear_acceleration.y;

    // 将加速度投影到自车坐标系
    double acceleration_x_car =
        acceleration_x_world;  // * cos(yaw) + acceleration_y_world * sin(yaw); // 车体x方向
    // double acceleration_y_car = -acceleration_x_world * sin(yaw) + acceleration_y_world *
    // cos(yaw); // 车体y方向

    // 计算车辆在自车坐标系下的加速度
    double real_acceleration = acceleration_x_car;  // 只取x方向的加速度

    // 更新速度
    current_velocity_ += +real_acceleration * delta_time;

    // 更新偏航角
    current_heading_ = yaw;
}

void AutoDriveControl::setMemoryTrajectory(
    const std::vector<sensor_msgs::msg::NavSatFix> &trajectory) {
    trajectory_ = trajectory;
}

void AutoDriveControl::setAutoDriveFlag(bool flag) { auto_drive_requested_ = flag; }

double AutoDriveControl::haversineDistance(double lat1, double lon1, double lat2, double lon2) {
    const double R = 6371000.0;  // 地球半径（米）
    double dLat = (lat2 - lat1) * M_PI / 180.0;
    double dLon = (lon2 - lon1) * M_PI / 180.0;
    double a = std::pow(std::sin(dLat / 2), 2) + std::cos(lat1 * M_PI / 180.0) *
                                                     std::cos(lat2 * M_PI / 180.0) *
                                                     std::pow(std::sin(dLon / 2), 2);
    double c = 2 * std::atan2(std::sqrt(a), std::sqrt(1 - a));
    return R * c;
}

void AutoDriveControl::find_nearest_point(
    const sensor_msgs::msg::NavSatFix &current_pose,
    const std::vector<sensor_msgs::msg::NavSatFix> &trajectory, size_t &nearest_index) {
    double min_distance = std::numeric_limits<double>::max();
    for (size_t i = 0; i < trajectory.size(); ++i) {
        double distance = haversineDistance(current_pose.latitude, current_pose.longitude,
                                            trajectory[i].latitude, trajectory[i].longitude);
        if (distance < min_distance) {
            min_distance = distance;
            nearest_index = i;
        }
    }
}

void AutoDriveControl::updateTargetPos() {
    size_t nearest_index = 0;
    find_nearest_point(current_pose_, trajectory_, nearest_index);
    RCLCPP_INFO(node_->get_logger(), "Nearest point index: %zu", nearest_index);
    if (nearest_index + 1 < trajectory_.size()) {
        const auto &p_nearest = trajectory_[nearest_index];
        const auto &p_next = trajectory_[nearest_index + 1];
        int target_index = nearest_index + std::round(current_velocity_ * 2 / resample_spacing_);
        target_index = std::max(target_index, int(nearest_index + 1));
        target_index = std::min(target_index, int(trajectory_.size() - 1));
        const auto &p_target = trajectory_[std::min(nearest_index + 2, trajectory_.size() - 1)];
        if (current_heading_ > M_PI) {
            current_heading_ -= 2 * M_PI;
        } else if (current_heading_ < -M_PI) {
            current_heading_ += 2 * M_PI;
        }
        RCLCPP_INFO(node_->get_logger(), "current pose: (%.7f, %.7f, %.2f)", current_pose_.latitude,
                    current_pose_.longitude, current_heading_ * 180 / M_PI);
        RCLCPP_INFO(node_->get_logger(), "Target point index: %d", target_index);
        RCLCPP_INFO(node_->get_logger(), "Nearest point: (%.7f, %.7f), Next point: (%.7f, %.7f)",
                    p_nearest.latitude, p_nearest.longitude, p_next.latitude, p_next.longitude);
        RCLCPP_INFO(node_->get_logger(), "Target point: (%.7f, %.7f)", p_target.latitude,
                    p_target.longitude);
        // 以当前位置为原点，转换为世界坐标
        GeographicLib::LocalCartesian geo_converter_;
        geo_converter_.Reset(current_pose_.latitude, current_pose_.longitude, 0);
        double x_nearest_ENU, y_nearest_ENU, z_nearest_ENU;
        geo_converter_.Forward(p_nearest.latitude, p_nearest.longitude, p_nearest.altitude,
                               x_nearest_ENU, y_nearest_ENU, z_nearest_ENU);
        double x_next_ENU, y_next_ENU, z_next_ENU;
        geo_converter_.Forward(p_next.latitude, p_next.longitude, p_next.altitude, x_next_ENU,
                               y_next_ENU, z_next_ENU);
        double x_target_ENU, y_target_ENU, z_target_ENU;
        geo_converter_.Forward(p_target.latitude, p_target.longitude, p_target.altitude,
                               x_target_ENU, y_target_ENU, z_target_ENU);
        RCLCPP_INFO(node_->get_logger(),
                    "Nearest point ENU: (%.2f, %.2f), Next point ENU: (%.2f, %.2f), Target point "
                    "ENU: (%.2f, %.2f)",
                    x_nearest_ENU, y_nearest_ENU, x_next_ENU, y_next_ENU, x_target_ENU,
                    y_target_ENU);
        // 转换到Gazebo坐标系下
        double x_nearest_ENU_gazebo = -x_nearest_ENU;
        double y_nearest_ENU_gazebo = -y_nearest_ENU;
        double x_next_ENU_gazebo = -x_next_ENU;
        double y_next_ENU_gazebo = -y_next_ENU;
        double x_target_ENU_gazebo = -x_target_ENU;
        double y_target_ENU_gazebo = -y_target_ENU;
        RCLCPP_INFO(node_->get_logger(),
                    "Nearest point ENU Gazebo: (%.2f, %.2f), Next point ENU Gazebo: (%.2f, %.2f), "
                    "Target point ENU Gazebo: (%.2f, %.2f)",
                    x_nearest_ENU_gazebo, y_nearest_ENU_gazebo, x_next_ENU_gazebo,
                    y_next_ENU_gazebo, x_target_ENU_gazebo, y_target_ENU_gazebo);
        // 转换到自车坐标系下
        // 应用旋转公式
        double x_nearest, y_nearest, x_next, y_next, x_target, y_target;

        x_nearest = (x_nearest_ENU_gazebo * cos(current_heading_) +
                     y_nearest_ENU_gazebo * sin(current_heading_));
        y_nearest = (-x_nearest_ENU_gazebo * sin(current_heading_) +
                     y_nearest_ENU_gazebo * cos(current_heading_));
        x_next =
            (x_next_ENU_gazebo * cos(current_heading_) + y_next_ENU_gazebo * sin(current_heading_));
        y_next = (-x_next_ENU_gazebo * sin(current_heading_) +
                  y_next_ENU_gazebo * cos(current_heading_));
        x_target = (x_target_ENU_gazebo * cos(current_heading_) +
                    y_target_ENU_gazebo * sin(current_heading_));
        y_target = (-x_target_ENU_gazebo * sin(current_heading_) +
                    y_target_ENU_gazebo * cos(current_heading_));

        RCLCPP_INFO(
            node_->get_logger(),
            "Nearest point: (%.2f, %.2f), Next point: (%.2f, %.2f), Target point: (%.2f, %.2f)",
            x_nearest, y_nearest, x_next, y_next, x_target, y_target);
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
        target_x_ = x_projection + 0.5 * (x_next - x_nearest);
        target_y_ = y_projection + 0.5 * (y_next - y_nearest);
        //
        target_x_ = x_target;
        target_y_ = y_target;
        // 计算预瞄点的速度
        target_speed_mps_ = speed_curve_[target_index];  // 先给0.5，先测试横向
        // 计算预瞄点的航向角
        target_yaw_ = atan2(y_next - y_nearest, x_next - x_nearest);
    }
}

void AutoDriveControl::pure_pursuit_control() {
    // 计算前轮转角
    double L = 2.5;  // 轴距
    double Ld = sqrt(target_x_ * target_x_ + target_y_ * target_y_);
    if (Ld < 0.3) {
        Ld = 0.3;  // 防止除以0或角度剧烈变化
        RCLCPP_INFO(node_->get_logger(), "❌ Ld too small, set to 0.3");
    }
    double delta = atan2(2 * L * target_y_, Ld * Ld);
    // 限制最大前轮转角
    delta = std::max(-0.13 * M_PI, std::min(0.13 * M_PI, delta));
    // 计算车速
    double speed = current_velocity_;  // TODO
    // 计算前轮转角速度
    double angular_speed = speed / L * tan(delta);
    angular_speed_ = std::max(angular_speed, angular_speed_ - 0.1);  // 限制最大转角速度
    angular_speed_ = std::min(angular_speed, angular_speed_ + 0.1);  // 限制最小转角速度
}

void AutoDriveControl::pid_control_lon() {
    linear_speed_ = target_speed_mps_;  // TODO
}

// 线性插值函数
double linearInterpolate(const std::vector<double> &x, const std::vector<double> &y,
                         double target_x) {
    // 寻找目标距离所在的区间
    auto it = std::lower_bound(x.begin(), x.end(), target_x);
    if (it == x.end()) return y.back();
    if (it == x.begin()) return y.front();

    size_t idx = std::distance(x.begin(), it);
    double x1 = x[idx - 1];
    double x2 = x[idx];
    double y1 = y[idx - 1];
    double y2 = y[idx];

    // 线性插值公式
    return y1 + (y2 - y1) * (target_x - x1) / (x2 - x1);
}

void AutoDriveControl::smoothMemoryTrajectory(const int &window_size) {
    int n = trajectory_.size();
    std::vector<sensor_msgs::msg::NavSatFix> smoothed;

    // 保持第一个点和最后一个点不变
    smoothed.push_back(trajectory_[0]);  // 保持起始点不变
    for (int i = 1; i < n - 1; ++i) {    // 对中间点进行平滑
        double sum_lat = 0, sum_lon = 0, sum_time = 0;
        int count = 0;
        for (int j = std::max(0, i - window_size); j < std::min(n, i + window_size + 1); ++j) {
            sum_lat += trajectory_[j].latitude;
            sum_lon += trajectory_[j].longitude;
            sum_time += trajectory_[j].header.stamp.sec;
            count++;
        }
        sensor_msgs::msg::NavSatFix smoothed_point;
        smoothed_point.latitude = sum_lat / count;
        smoothed_point.longitude = sum_lon / count;
        smoothed_point.header.stamp.sec = sum_time / count;
        smoothed.push_back(smoothed_point);
    }
    smoothed.push_back(trajectory_[n - 1]);  // 保持终止点不变

    // 更新原始轨迹为平滑后的轨迹
    trajectory_ = smoothed;
    // 重新采样轨迹
    resampleMemoryTrajectoryBySpacing();
    // 生成速度曲线
    buildSpeedCurve();
    writeSmoothedTrajectoryToFile();
}

void AutoDriveControl::buildSpeedCurve() {
    // 每个速度点与路径点一一对应
    speed_curve_.resize(trajectory_.size());
    // 最后一个速度点必须为0，因为要停在目标位置
    speed_curve_.back() = 0.0;
    // 最大加速度 ±0.5m/s, 推荐速度 5.5(20km/h)
    double max_acceleration = 0.5;
    double recommended_speed = 5.5;
    speed_curve_[1] = current_velocity_ + 1;
    // 目标：如果距离允许，尽可能保持推荐速度行驶，如果距离不允许，比如加到推荐速度就得立马减速，甚至还没加到就得减速，就向下寻找合适的速度
    // 从推荐速度开始向下寻找合适的速度
    for (int i = 0; i < 10; i++) {
        recommended_speed -= i * 0.5;
        // 从当前位置加速到推荐速度
        double acc_distance = (recommended_speed * recommended_speed -
                               ((current_velocity_ + 1.0) * (current_velocity_ + 1.0))) /
                              (2 * max_acceleration);
        // 从推荐速度减到0
        double dec_distance = (recommended_speed * recommended_speed) / (2 * max_acceleration);
        if (acc_distance + dec_distance <= 0.5 * traj_length_) {
            // 加减速阶段行驶距离不超过总长度的一般，说明保持推荐速度行驶的时间够长,根据resample_length(每个轨迹点的间隔)和最大加速度设置每个点的速度
            for (size_t j = 1; j < trajectory_.size(); ++j) {
                double distance =
                    haversineDistance(trajectory_[j].latitude, trajectory_[j].longitude,
                                      trajectory_[0].latitude, trajectory_[0].longitude);
                if (distance < acc_distance) {
                    speed_curve_[j] = std::sqrt(speed_curve_[j - 1] * speed_curve_[j - 1] +
                                                2 * max_acceleration * resample_spacing_);
                } else if (distance < traj_length_ - dec_distance && distance > acc_distance) {
                    speed_curve_[j] = recommended_speed;
                } else {
                    speed_curve_[j] = recommended_speed;
                    speed_curve_[j] = std::sqrt(speed_curve_[j - 1] * speed_curve_[j - 1] -
                                                2 * max_acceleration * resample_spacing_);
                }
            }
            break;
        }
    }
}

void AutoDriveControl::writeSmoothedTrajectoryToFile() {
    // 将平滑后的轨迹写入文件
    trajectory_log_file_.open("smoothed_trajectory_log.txt", std::ios::out | std::ios::trunc);
    for (const auto &point : trajectory_) {
        trajectory_log_file_ << std::fixed << std::setprecision(8) << "Latitude: " << point.latitude
                             << ", Longitude:" << point.longitude
                             << ", Altitude: " << point.header.stamp.sec << "\n";
    }
    RCLCPP_INFO(node_->get_logger(), "✅ Smoothed trajectory written to file.");
}

void AutoDriveControl::resampleMemoryTrajectoryBySpacing() {
    // 计算每个点的累积距离
    std::vector<double> distances = {0.0};  // 第一个点的距离为0
    for (size_t i = 1; i < trajectory_.size(); ++i) {
        double d = haversineDistance(trajectory_[i - 1].latitude, trajectory_[i - 1].longitude,
                                     trajectory_[i].latitude, trajectory_[i].longitude);
        distances.push_back(distances.back() + d);
    }

    // 计算均匀间隔的目标距离
    traj_length_ = distances.back();
    std::vector<double> uniform_distances;
    for (double d = 0.0; d < traj_length_; d += resample_spacing_) {
        uniform_distances.push_back(d);
    }
    uniform_distances.push_back(traj_length_);  // 添加最后一个点

    // 进行线性插值，生成新的轨迹
    std::vector<sensor_msgs::msg::NavSatFix> resampled_trajectory;

    // 获取轨迹的纬度、经度、时间等数据
    std::vector<double> latitudes, longitudes, times;
    for (const auto &point : trajectory_) {
        latitudes.push_back(point.latitude);
        longitudes.push_back(point.longitude);
        times.push_back(point.header.stamp.sec);
    }

    for (double target_distance : uniform_distances) {
        sensor_msgs::msg::NavSatFix resampled_point;

        // 使用线性插值生成新的经纬度
        resampled_point.latitude = linearInterpolate(distances, latitudes, target_distance);
        resampled_point.longitude = linearInterpolate(distances, longitudes, target_distance);
        resampled_point.header.stamp.sec =
            static_cast<int>(linearInterpolate(distances, times, target_distance));

        resampled_trajectory.push_back(resampled_point);
    }

    // 更新轨迹为重采样后的轨迹
    trajectory_ = resampled_trajectory;
}

void AutoDriveControl::update(const std::vector<sensor_msgs::msg::NavSatFix> &trajectory,
                              const int &MemoryTrajectoryState) {
    double now = node_->now().seconds();
    double elapsed = now - log_start_time_.seconds();
    switch (state_) {
        case 0:
            if (trajectory.empty()) {
                RCLCPP_WARN(node_->get_logger(), "❌ No trajectory data available.");
                break;
            }
            if (MemoryTrajectoryState != 2) {
                RCLCPP_WARN(node_->get_logger(), "❌ Memory trajectory not finish.");
                break;
            }
            if (get_distance_from_2pos(current_pose_, trajectory[0]) > max_start_offset_) {
                RCLCPP_WARN(node_->get_logger(), "❌ Too far from start point.");
                break;
            }
            if (get_distance_from_2pos(trajectory[0], trajectory[trajectory.size() - 1]) <
                max_start_offset_) {
                RCLCPP_WARN(node_->get_logger(), "❌ Start and end points too close.");
                break;
            }
            if (!auto_drive_requested_) {
                RCLCPP_WARN(node_->get_logger(), "❌ Auto-drive not requested.");
                return;
            }
            state_ = 1;
            load_trajectory_ = false;
            RCLCPP_INFO(node_->get_logger(), "✅ Auto-drive off ->active!");
            break;
        case 1:
            if (!auto_drive_requested_) {
                RCLCPP_WARN(node_->get_logger(), "❌ Auto-drive not requested.");
                state_ = 0;
                break;
            }
            if (haversineDistance(current_pose_.latitude, current_pose_.longitude,
                                  trajectory[trajectory.size() - 1].latitude,
                                  trajectory[trajectory.size() - 1].longitude) < 5) {
                RCLCPP_INFO(node_->get_logger(), "✅ Arrive end point! Auto-drive finished!");
                state_ = 3;
                finish_time_ = node_->now();
                break;
            }
            if (!load_trajectory_) {
                setMemoryTrajectory(trajectory);
                smoothMemoryTrajectory(resample_spacing_);
                load_trajectory_ = true;
            }
            updateTargetPos();
            RCLCPP_INFO(node_->get_logger(), " 👀 target_x= %.2f, target_y= %2.f, target_yaw= %2.f",
                        target_x_, target_y_, target_yaw_);
            // 有了目标点的坐标和速度，下面进行控制，先用纯跟踪算法
            pure_pursuit_control();  // 横向控制
            pid_control_lon();       // 纵向控制
            // 发布控制指令
            {
                geometry_msgs::msg::Twist msg;
                msg.linear.x = linear_speed_;
                msg.angular.z = angular_speed_;
                // 发布消息
                cmd_vel_pub_->publish(msg);
                RCLCPP_INFO(node_->get_logger(), "🚗 Auto-driving: linear.x=%.2f, angular.z=%.2f",
                            msg.linear.x, msg.angular.z);
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
            if (!auto_drive_requested_) {
                RCLCPP_WARN(node_->get_logger(), "😃 Auto-drive finish -> off");
                state_ = 0;
                break;
            }
            // 如果完成时间超过5秒，自动关闭
            if (node_->now().seconds() - finish_time_.seconds() > 5) {
                RCLCPP_INFO(node_->get_logger(), "✅ Auto-drive finished!");
                state_ = 0;
                break;
            }
            break;
        default:
            break;
    }
}

void AutoDriveControl::start_keyboard_listener() {
    keyboard_thread_ = std::thread(&AutoDriveControl::listen_for_keypress, this);
}

void AutoDriveControl::listen_for_keypress() {
    while (rclcpp::ok()) {
        char key = get_keypress();
        if (key == '/') {
            auto_drive_requested_ = !auto_drive_requested_;
            if (auto_drive_requested_) {
                RCLCPP_INFO(node_->get_logger(), "🚗 AutoDrive ON");
            } else {
                RCLCPP_INFO(node_->get_logger(), "🅿️ AutoDrive OFF");
            }
        }
    }
}

char AutoDriveControl::get_keypress() {
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
