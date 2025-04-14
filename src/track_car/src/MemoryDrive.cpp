// src/MemoryDrive.cpp

#include "track_car/MemoryDrive.hpp"

MemoryDriveNode::MemoryDriveNode() : Node("MemoryDrive_node")
{
    RCLCPP_INFO(this->get_logger(), "🚗 MemoryDrive system started.");

    // 初始化两个模块
    memory_trajectory_generator_ = std::make_shared<MemoryTrajectoryGenerate>(this);
    auto_drive_control_ = std::make_shared<AutoDriveControl>(this);
}

MemoryDriveNode::~MemoryDriveNode()
{
    RCLCPP_INFO(this->get_logger(), "MemoryDrive system shutting down.");
}

void MemoryDriveNode::Update()
{
    // 更新轨迹生成器
    memory_trajectory_generator_->updateMemoryTrajectory();
    auto trajectory =memory_trajectory_generator_ ->getTrajectory();
	std::vector<sensor_msgs::msg::NavSatFix> test_trajectory;
	{
		sensor_msgs::msg::NavSatFix pt;
		pt.latitude = 0.00003504; pt.longitude = 0.00039254; pt.altitude = 0.31153569; test_trajectory.push_back(pt);
		pt.latitude = 0.00003497; pt.longitude = 0.00038896; pt.altitude = 0.31151709; test_trajectory.push_back(pt);
		pt.latitude = 0.00003490; pt.longitude = 0.00038539; pt.altitude = 0.31159778; test_trajectory.push_back(pt);
		pt.latitude = 0.00003481; pt.longitude = 0.00038110; pt.altitude = 0.31144959; test_trajectory.push_back(pt);
		pt.latitude = 0.00003474; pt.longitude = 0.00037753; pt.altitude = 0.31161213; test_trajectory.push_back(pt);
		pt.latitude = 0.00003466; pt.longitude = 0.00037324; pt.altitude = 0.31160273; test_trajectory.push_back(pt);
		pt.latitude = 0.00003458; pt.longitude = 0.00036967; pt.altitude = 0.31142102; test_trajectory.push_back(pt);
		pt.latitude = 0.00003451; pt.longitude = 0.00036609; pt.altitude = 0.31152331; test_trajectory.push_back(pt);
		pt.latitude = 0.00003444; pt.longitude = 0.00036252; pt.altitude = 0.31138054; test_trajectory.push_back(pt);
		pt.latitude = 0.00003436; pt.longitude = 0.00035823; pt.altitude = 0.31149646; test_trajectory.push_back(pt);
		pt.latitude = 0.00003428; pt.longitude = 0.00035466; pt.altitude = 0.31147893; test_trajectory.push_back(pt);
		pt.latitude = 0.00003424; pt.longitude = 0.00034900; pt.altitude = 0.31154121; test_trajectory.push_back(pt);
		pt.latitude = 0.00003461; pt.longitude = 0.00034140; pt.altitude = 0.31136373; test_trajectory.push_back(pt);
		pt.latitude = 0.00003538; pt.longitude = 0.00033184; pt.altitude = 0.31138125; test_trajectory.push_back(pt);
		pt.latitude = 0.00003629; pt.longitude = 0.00032383; pt.altitude = 0.31137147; test_trajectory.push_back(pt);
		pt.latitude = 0.00003744; pt.longitude = 0.00031583; pt.altitude = 0.31130534; test_trajectory.push_back(pt);
		pt.latitude = 0.00003882; pt.longitude = 0.00030788; pt.altitude = 0.31146174; test_trajectory.push_back(pt);
		pt.latitude = 0.00004079; pt.longitude = 0.00029839; pt.altitude = 0.31142878; test_trajectory.push_back(pt);
		pt.latitude = 0.00004318; pt.longitude = 0.00028851; pt.altitude = 0.31161735; test_trajectory.push_back(pt);
		pt.latitude = 0.00004642; pt.longitude = 0.00027693; pt.altitude = 0.31155171; test_trajectory.push_back(pt);
		pt.latitude = 0.00005088; pt.longitude = 0.00026275; pt.altitude = 0.31132908; test_trajectory.push_back(pt);
		pt.latitude = 0.00005430; pt.longitude = 0.00025098; pt.altitude = 0.31153444; test_trajectory.push_back(pt);
		pt.latitude = 0.00005765; pt.longitude = 0.00023925; pt.altitude = 0.31141827; test_trajectory.push_back(pt);
		pt.latitude = 0.00006097; pt.longitude = 0.00022754; pt.altitude = 0.31150280; test_trajectory.push_back(pt);
		pt.latitude = 0.00006492; pt.longitude = 0.00021346; pt.altitude = 0.31141172; test_trajectory.push_back(pt);
		pt.latitude = 0.00006773; pt.longitude = 0.00020170; pt.altitude = 0.31167029; test_trajectory.push_back(pt);
		pt.latitude = 0.00006976; pt.longitude = 0.00018993; pt.altitude = 0.31146602; test_trajectory.push_back(pt);
		pt.latitude = 0.00007131; pt.longitude = 0.00017813; pt.altitude = 0.31131894; test_trajectory.push_back(pt);
		pt.latitude = 0.00007379; pt.longitude = 0.00016221; pt.altitude = 0.31133180; test_trajectory.push_back(pt);
		pt.latitude = 0.00007658; pt.longitude = 0.00014341; pt.altitude = 0.31157457; test_trajectory.push_back(pt);
		pt.latitude = 0.00007947; pt.longitude = 0.00012317; pt.altitude = 0.31139258; test_trajectory.push_back(pt);
		pt.latitude = 0.00008290; pt.longitude = 0.00009855; pt.altitude = 0.31147338; test_trajectory.push_back(pt);
		pt.latitude = 0.00008568; pt.longitude = 0.00007798; pt.altitude = 0.31128235; test_trajectory.push_back(pt);
		pt.latitude = 0.00008809; pt.longitude = 0.00005739; pt.altitude = 0.31130562; test_trajectory.push_back(pt);
		pt.latitude = 0.00008873; pt.longitude = 0.00003694; pt.altitude = 0.31130466; test_trajectory.push_back(pt);
		pt.latitude = 0.00008927; pt.longitude = 0.00001640; pt.altitude = 0.31150784; test_trajectory.push_back(pt);
		pt.latitude = 0.00009226; pt.longitude = -0.00001087; pt.altitude = 0.31124709; test_trajectory.push_back(pt);
		pt.latitude = 0.00009475; pt.longitude = -0.00003561; pt.altitude = 0.31144910; test_trajectory.push_back(pt);
		pt.latitude = 0.00009738; pt.longitude = -0.00006309; pt.altitude = 0.31124968; test_trajectory.push_back(pt);
		pt.latitude = 0.00009998; pt.longitude = -0.00009198; pt.altitude = 0.31145503; test_trajectory.push_back(pt);
		pt.latitude = 0.00010245; pt.longitude = -0.00012114; pt.altitude = 0.31140692; test_trajectory.push_back(pt);
		pt.latitude = 0.00010450; pt.longitude = -0.00014662; pt.altitude = 0.31142890; test_trajectory.push_back(pt);
		pt.latitude = 0.00010653; pt.longitude = -0.00017248; pt.altitude = 0.31140034; test_trajectory.push_back(pt);
		pt.latitude = 0.00010802; pt.longitude = -0.00019207; pt.altitude = 0.31150142; test_trajectory.push_back(pt);
		pt.latitude = 0.00010919; pt.longitude = -0.00020792; pt.altitude = 0.31139408; test_trajectory.push_back(pt);
		pt.latitude = 0.00011019; pt.longitude = -0.00022158; pt.altitude = 0.31141135; test_trajectory.push_back(pt);
		pt.latitude = 0.00011060; pt.longitude = -0.00022723; pt.altitude = 0.31149488; test_trajectory.push_back(pt);
		pt.latitude = 0.00011090; pt.longitude = -0.00023117; pt.altitude = 0.31135723; test_trajectory.push_back(pt);
	}
	
    // 更新控制器
    auto_drive_control_->update(test_trajectory, 2);
}
