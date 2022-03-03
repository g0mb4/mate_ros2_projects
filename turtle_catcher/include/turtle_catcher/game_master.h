#pragma once

#include <random>
#include <rclcpp/rclcpp.hpp>
#include <turtle_catcher/srv/target_position.hpp>
#include <turtlesim/msg/pose.hpp>
#include <turtlesim/srv/kill.hpp>
#include <turtlesim/srv/spawn.hpp>

class GameMaster : public rclcpp::Node {
public:
    GameMaster();

    void reset_target();

private:
    void control_loop_tick();

    bool kill_spawn_send_position();
    bool spawn_send_position();
    bool send_position();

    void subscribe_pose(const turtlesim::msg::Pose::SharedPtr);

    turtlesim::msg::Pose::SharedPtr m_pose { nullptr };
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr m_pose_subscriber;
    rclcpp::TimerBase::SharedPtr m_control_loop_timer;

    std::mt19937 m_random_generator;
    float m_target_x { 0 }, m_target_y { 0 };
    float m_tolerance { 0 };
};
