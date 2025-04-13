#include "track_car/MemoryDrive.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MemoryDriveNode>();

    // 频率更新主逻辑
    rclcpp::Rate rate(10.0);  // 10 Hz
    while (rclcpp::ok()) {
        rclcpp::spin_some(node);
        node->Update();
        rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}
