#pragma once

#include <rclcpp/rclcpp.hpp>
#include <turtlesim/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <turtle_catcher/srv/target_position.hpp>

class Turtle1Controller: public rclcpp::Node{
public:
    Turtle1Controller();

private:
    void control_loop_tick();

    void subscribe_pose(const turtlesim::msg::Pose::SharedPtr);

    void serve_target_position( 
        const turtle_catcher::srv::TargetPosition::Request::SharedPtr,
        const turtle_catcher::srv::TargetPosition::Response::SharedPtr);

    turtlesim::msg::Pose::SharedPtr m_pose { nullptr };
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr m_pose_subscriber;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr m_cmd_vel_publisher;
    rclcpp::TimerBase::SharedPtr m_control_loop_timer;
    rclcpp::Service<turtle_catcher::srv::TargetPosition>::SharedPtr m_target_position_server;

    float m_target_x {0}, m_target_y {0};
    float m_tolerance;
    float m_P_distance, m_P_angle;
    bool m_running {false};
};