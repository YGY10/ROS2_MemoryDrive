// include/track_car/MemoryDrive.hpp
#pragma once

#include "rclcpp/rclcpp.hpp"
#include "MemoryTrajectoryGenerate.hpp"
#include "AutoDriveControl.hpp"

class MemoryDriveNode : public rclcpp::Node
{
public:
	MemoryDriveNode();
	~MemoryDriveNode();
	void Update();

private:
	std::shared_ptr<MemoryTrajectoryGenerate> memory_trajectory_generator_;
	std::shared_ptr<AutoDriveControl> auto_drive_control_;
};
