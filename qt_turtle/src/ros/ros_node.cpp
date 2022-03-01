#include <qt_turtle/ros/ros_node.h>

RosNode::RosNode()
 : rclcpp::Node("qt_turtle") {
    m_subscriber = create_subscription<turtlesim::msg::Pose>(
                   "turtle1/pose", 10,
                   std::bind(&RosNode::subscribe_pose, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "started");
}

void RosNode::subscribe_pose(const turtlesim::msg::Pose::SharedPtr pose){
    m_pose = *pose;
}
