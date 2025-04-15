#include "track_car/MemoryTrajectoryGenerate.hpp"
#include <chrono>
#include <iomanip>  // for std::setprecision and std::fixed

using namespace std::chrono_literals;

MemoryTrajectoryGenerate::MemoryTrajectoryGenerate(rclcpp::Node *node)
    : node_(node),
      is_recording_(false),
      new_imu_received_(false),
      new_gps_received_(false),
      last_velocity_(0.0),
      current_velocity_(0.0),
      last_time_(0.0),
      current_heading_(0.0),
      state_(0) {
    RCLCPP_INFO(node_->get_logger(), "🚗 MemoryTrajectory Generate Started.");

    start_time_ = node_->now();  // 记录程序启动时的时间

    gps_subscription_ = node_->create_subscription<sensor_msgs::msg::NavSatFix>(
        "/gps/data", 10, [this](const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
            latest_gps_data_ = msg;
            new_gps_received_ = true;
        });

    imu_subscription_ = node_->create_subscription<sensor_msgs::msg::Imu>(
        "/imu/data", 10, [this](const sensor_msgs::msg::Imu::SharedPtr msg) {
            latest_imu_data_ = msg;
            new_imu_received_ = true;
        });

    trajectory_pub_ = node_->create_publisher<sensor_msgs::msg::NavSatFix>("/track/trajectory", 10);

    start_keyboard_listener();
}

MemoryTrajectoryGenerate::~MemoryTrajectoryGenerate() {
    if (gps_file_.is_open()) gps_file_.close();
    if (keyboard_thread_.joinable()) keyboard_thread_.join();
}

void MemoryTrajectoryGenerate::updateMemoryTrajectory() {
    if (new_imu_received_) {
        process_imu();
        new_imu_received_ = false;
    }

    if (new_gps_received_) {
        process_gps();
        new_gps_received_ = false;
    }

    if (is_recording_) {
        double now = node_->now().seconds();
        if (now - last_saved_time_ >= 1.0) {
            save_gps_data();
            last_saved_time_ = now;
        }
    }
}

void MemoryTrajectoryGenerate::process_imu() {
    if (!latest_imu_data_) return;

    double current_time = node_->now().seconds();
    double delta_time = 0.0;

    // 初始化 last_time_ 以确保第一次计算时没有错误
    if (last_time_ == 0.0) {
        last_time_ = current_time;
        delta_time = 0.1;  // 第一次更新，以函数调用周期为准
    } else {
        delta_time = current_time - last_time_;
    }

    tf2::Quaternion q(latest_imu_data_->orientation.x, latest_imu_data_->orientation.y,
                      latest_imu_data_->orientation.z, latest_imu_data_->orientation.w);

    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

    // 将加速度从世界坐标系转换到自车坐标系
    double acceleration_x_car = latest_imu_data_->linear_acceleration.x;  // 车体x方向

    // 计算车辆在自车坐标系下的加速度
    double real_acceleration = acceleration_x_car;  // 只取x方向的加速度
    // 更新速度
    current_velocity_ = last_velocity_ + real_acceleration * delta_time;
    last_velocity_ = current_velocity_;

    // 更新偏航角
    current_heading_ = yaw * 180.0 / M_PI;

    // 记录当前时间
    last_time_ = current_time;
}

void MemoryTrajectoryGenerate::process_gps() {
    // 可扩展：用于滤波或其他处理
}

void MemoryTrajectoryGenerate::save_gps_data() {
    if (!latest_gps_data_) return;

    double elapsed = (node_->now() - start_time_).seconds();

    gps_file_ << std::fixed << std::setprecision(8)  // 设置浮点格式和精度
              << "Latitude: " << latest_gps_data_->latitude
              << ", Longitude: " << latest_gps_data_->longitude
              << ", Altitude: " << latest_gps_data_->altitude << ", Time: " << std::setprecision(2)
              << elapsed << ", Velocity: " << std::setprecision(5) << current_velocity_
              << ", Heading: " << std::setprecision(2) << current_heading_ << std::endl;

    gps_trajectory_.push_back(*latest_gps_data_);
    trajectory_pub_->publish(*latest_gps_data_);

    RCLCPP_INFO(node_->get_logger(), "📌 GPS: Lat = %f, Lon = %f, Vel = %f",
                latest_gps_data_->latitude, latest_gps_data_->longitude, current_velocity_);
}

std::vector<sensor_msgs::msg::NavSatFix> MemoryTrajectoryGenerate::getTrajectory() const {
    return gps_trajectory_;
}

void MemoryTrajectoryGenerate::start_keyboard_listener() {
    keyboard_thread_ = std::thread(&MemoryTrajectoryGenerate::listen_for_keypress, this);
}

void MemoryTrajectoryGenerate::listen_for_keypress() {
    while (rclcpp::ok()) {
        char key = get_keypress();
        if (key == '.') {
            is_recording_ = !is_recording_;
            if (is_recording_) {
                state_ = 1;
                gps_file_.open("gps_data.txt", std::ios::out | std::ios::trunc);
                RCLCPP_INFO(node_->get_logger(), "✅ Start Recording MemoryTrajectory.");
            } else {
                state_ = 2;
                RCLCPP_INFO(node_->get_logger(), "🔚 Finish Recording MemoryTrajectory.");
            }
        }
    }
}

char MemoryTrajectoryGenerate::get_keypress() {
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
