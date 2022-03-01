#pragma once

#include <rclcpp/rclcpp.hpp>
#include <turtlesim/srv/kill.hpp>
#include <turtlesim/srv/spawn.hpp>
#include <turtlesim/msg/pose.hpp>
#include <random>
#include <turtle_catcher/srv/target_position.hpp>

class GameMaster: public rclcpp::Node {
public:
    GameMaster();

    void reset_target();
private:
    void control_loop_tick();

    void kill_target_impl();
    bool spawn_target_impl();
    void send_target_position_impl();

    void subscribe_pose(const turtlesim::msg::Pose::SharedPtr);

    turtlesim::msg::Pose::SharedPtr m_pose {nullptr};
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr m_pose_subscriber;
    rclcpp::TimerBase::SharedPtr m_control_loop_timer;

    std::mt19937 m_random_generator;
    float m_target_x {0}, m_target_y {0};
    float m_tolerance {0};
};
