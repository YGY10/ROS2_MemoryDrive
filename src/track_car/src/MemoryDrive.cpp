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
//     std::vector<sensor_msgs::msg::NavSatFix> test_trajectory;
// {
//     sensor_msgs::msg::NavSatFix pt;
//     pt.latitude = 0.00000002; pt.longitude = -0.00000003; pt.altitude = 0.31129024; test_trajectory.push_back(pt);
//     pt.latitude = 0.00000001; pt.longitude = -0.00000003; pt.altitude = 0.31151906; test_trajectory.push_back(pt);
//     pt.latitude = 0.00000002; pt.longitude = -0.00000003; pt.altitude = 0.31151772; test_trajectory.push_back(pt);
//     pt.latitude = 0.00000002; pt.longitude = -0.00000003; pt.altitude = 0.31135214; test_trajectory.push_back(pt);
//     pt.latitude = 0.00000002; pt.longitude = -0.00000003; pt.altitude = 0.31153351; test_trajectory.push_back(pt);
//     pt.latitude = 0.00000002; pt.longitude = -0.00000002; pt.altitude = 0.31137484; test_trajectory.push_back(pt);
//     pt.latitude = 0.00000002; pt.longitude = -0.00000083; pt.altitude = 0.31123202; test_trajectory.push_back(pt);
//     pt.latitude = 0.00000001; pt.longitude = -0.00000378; pt.altitude = 0.31130675; test_trajectory.push_back(pt);
//     pt.latitude = 0.00000001; pt.longitude = -0.00000724; pt.altitude = 0.31133626; test_trajectory.push_back(pt);
//     pt.latitude = 0.00000002; pt.longitude = -0.00001079; pt.altitude = 0.31125594; test_trajectory.push_back(pt);
//     pt.latitude = 0.00000001; pt.longitude = -0.00001437; pt.altitude = 0.31141413; test_trajectory.push_back(pt);
//     pt.latitude = 0.00000001; pt.longitude = -0.00001794; pt.altitude = 0.31141519; test_trajectory.push_back(pt);
//     pt.latitude = 0.00000001; pt.longitude = -0.00002151; pt.altitude = 0.31139150; test_trajectory.push_back(pt);
//     pt.latitude = 0.00000001; pt.longitude = -0.00002509; pt.altitude = 0.31144099; test_trajectory.push_back(pt);
//     pt.latitude = 0.00000001; pt.longitude = -0.00002867; pt.altitude = 0.31145914; test_trajectory.push_back(pt);
//     pt.latitude = 0.00000000; pt.longitude = -0.00003224; pt.altitude = 0.31129137; test_trajectory.push_back(pt);
//     pt.latitude = 0.00000000; pt.longitude = -0.00003581; pt.altitude = 0.31161748; test_trajectory.push_back(pt);
//     pt.latitude = 0.00000010; pt.longitude = -0.00003940; pt.altitude = 0.31140790; test_trajectory.push_back(pt);
//     pt.latitude = 0.00000041; pt.longitude = -0.00004424; pt.altitude = 0.31127696; test_trajectory.push_back(pt);
//     pt.latitude = 0.00000124; pt.longitude = -0.00005371; pt.altitude = 0.31111654; test_trajectory.push_back(pt);
//     pt.latitude = 0.00000307; pt.longitude = -0.00006815; pt.altitude = 0.31145107; test_trajectory.push_back(pt);
//     pt.latitude = 0.00000666; pt.longitude = -0.00008759; pt.altitude = 0.31130351; test_trajectory.push_back(pt);
//     pt.latitude = 0.00001070; pt.longitude = -0.00010390; pt.altitude = 0.31120763; test_trajectory.push_back(pt);
//     pt.latitude = 0.00001546; pt.longitude = -0.00012006; pt.altitude = 0.31127578; test_trajectory.push_back(pt);
//     pt.latitude = 0.00002071; pt.longitude = -0.00013928; pt.altitude = 0.31145707; test_trajectory.push_back(pt);
//     pt.latitude = 0.00002500; pt.longitude = -0.00015519; pt.altitude = 0.31147638; test_trajectory.push_back(pt);
//     pt.latitude = 0.00002924; pt.longitude = -0.00017112; pt.altitude = 0.31141521; test_trajectory.push_back(pt);
//     pt.latitude = 0.00003426; pt.longitude = -0.00019024; pt.altitude = 0.31142402; test_trajectory.push_back(pt);
//     pt.latitude = 0.00003838; pt.longitude = -0.00020619; pt.altitude = 0.31144280; test_trajectory.push_back(pt);
//     pt.latitude = 0.00004245; pt.longitude = -0.00022215; pt.altitude = 0.31146376; test_trajectory.push_back(pt);
//     pt.latitude = 0.00004647; pt.longitude = -0.00023812; pt.altitude = 0.31149248; test_trajectory.push_back(pt);
//     pt.latitude = 0.00005044; pt.longitude = -0.00025411; pt.altitude = 0.31129327; test_trajectory.push_back(pt);
//     pt.latitude = 0.00005436; pt.longitude = -0.00027011; pt.altitude = 0.31146770; test_trajectory.push_back(pt);
//     pt.latitude = 0.00005823; pt.longitude = -0.00028612; pt.altitude = 0.31131125; test_trajectory.push_back(pt);
//     pt.latitude = 0.00006206; pt.longitude = -0.00030214; pt.altitude = 0.31131130; test_trajectory.push_back(pt);
//     pt.latitude = 0.00006659; pt.longitude = -0.00032138; pt.altitude = 0.31137730; test_trajectory.push_back(pt);
//     pt.latitude = 0.00007031; pt.longitude = -0.00033743; pt.altitude = 0.31139062; test_trajectory.push_back(pt);
//     pt.latitude = 0.00007105; pt.longitude = -0.00034064; pt.altitude = 0.31139857; test_trajectory.push_back(pt);
//     pt.latitude = 0.00007586; pt.longitude = -0.00036193; pt.altitude = 0.31170816; test_trajectory.push_back(pt);
//     pt.latitude = 0.00007840; pt.longitude = -0.00037325; pt.altitude = 0.31147530; test_trajectory.push_back(pt);
//     pt.latitude = 0.00007995; pt.longitude = -0.00038021; pt.altitude = 0.31156292; test_trajectory.push_back(pt);
//     pt.latitude = 0.00008090; pt.longitude = -0.00038437; pt.altitude = 0.31144918; test_trajectory.push_back(pt);
//     pt.latitude = 0.00008129; pt.longitude = -0.00038626; pt.altitude = 0.31144720; test_trajectory.push_back(pt);
//     pt.latitude = 0.00008130; pt.longitude = -0.00038631; pt.altitude = 0.31156910; test_trajectory.push_back(pt);
//     pt.latitude = 0.00008130; pt.longitude = -0.00038630; pt.altitude = 0.31146792; test_trajectory.push_back(pt);
//     pt.latitude = 0.00008130; pt.longitude = -0.00038631; pt.altitude = 0.31131417; test_trajectory.push_back(pt);
//     pt.latitude = 0.00008130; pt.longitude = -0.00038630; pt.altitude = 0.31164167; test_trajectory.push_back(pt);
// }
    // 更新控制器
    auto_drive_control_->update(trajectory, memory_trajectory_generator_->getState());
}
