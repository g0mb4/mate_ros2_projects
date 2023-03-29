#pragma once

#include <rclcpp/rclcpp.hpp>
#include <turtlesim/msg/pose.hpp>

class RosNode : public rclcpp::Node
{
public:
    RosNode();

    float x() const { return m_pose.x; }
    float y() const { return m_pose.y; }
    float theta() const { return m_pose.theta; }

private:
    void subscribe_pose(const turtlesim::msg::Pose::SharedPtr pose);

    turtlesim::msg::Pose m_pose;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr m_subscriber;
};
