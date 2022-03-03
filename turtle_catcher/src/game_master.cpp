#include <turtle_catcher/game_master.h>

GameMaster::GameMaster()
    : rclcpp::Node("game_master")
    , m_random_generator(std::random_device()())
{
    declare_parameter<float>("tolerance", 0.1);
    m_tolerance = (float)get_parameter("tolerance").as_double();

    m_pose_subscriber = create_subscription<turtlesim::msg::Pose>(
        "turtle1/pose", 10,
        std::bind(&GameMaster::subscribe_pose,
            this, std::placeholders::_1));

    // tick-rate = 30 Hz -> 1s / 30 ~= 33 ms
    m_control_loop_timer = create_wall_timer(std::chrono::milliseconds(33),
        std::bind(&GameMaster::control_loop_tick, this));

    RCLCPP_INFO(get_logger(), "started");
    reset_target();
}

void GameMaster::reset_target()
{
    auto thread = std::make_unique<std::thread>(
        [this] {
            while (1) {
                if (kill_spawn_send_position()) {
                    break;
                }
            }
        });

    thread->detach();
}

void GameMaster::control_loop_tick()
{
    if (m_pose == nullptr) {
        return;
    }

    auto compute_target_distance = [this] {
        float dx = m_target_x - m_pose->x;
        float dy = m_target_y - m_pose->y;
        return sqrt(dx * dx + dy * dy);
    };

    if (compute_target_distance() <= m_tolerance) {
        reset_target();
    }
}

bool GameMaster::kill_spawn_send_position()
{
    auto client = create_client<turtlesim::srv::Kill>("kill");

    while (client->wait_for_service(std::chrono::seconds(1)) == false) {
        RCLCPP_WARN(get_logger(), "waiting for server ...");
    }

    auto request = std::make_shared<turtlesim::srv::Kill::Request>();
    request->name = "target";

    auto future = client->async_send_request(request);
    try {
        auto response = future.get();
        RCLCPP_INFO(get_logger(), "target killed.");

        return spawn_send_position();
    } catch (const std::exception& e) {
        RCLCPP_ERROR(get_logger(), "kill_target_impl: %s", e.what());
        return false;
    }
}

bool GameMaster::spawn_send_position()
{
    auto client = create_client<turtlesim::srv::Spawn>("spawn");

    while (client->wait_for_service(std::chrono::seconds(1)) == false) {
        RCLCPP_WARN(get_logger(), "waiting for server ...");
    }

    auto generate_random = [this](float min, float max) {
        std::uniform_real_distribution<double> distribution(min, max);
        return distribution(m_random_generator);
    };

    m_target_x = generate_random(1, 9);
    m_target_y = generate_random(1, 9);

    auto request = std::make_shared<turtlesim::srv::Spawn::Request>();
    request->x = m_target_x;
    request->y = m_target_y;

    request->name = "target";

    auto future = client->async_send_request(request);
    try {
        auto response = future.get();
        if (response->name != "target") {
            RCLCPP_WARN(get_logger(), "unable to spawn target.");
            return false;
        } else {
            RCLCPP_INFO(get_logger(), "target spawned: (%f; %f).", m_target_x, m_target_y);
            return send_position();
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(get_logger(), "spawn_target_impl: %s", e.what());
        return false;
    }
}

bool GameMaster::send_position()
{
    auto client = create_client<turtle_catcher::srv::TargetPosition>("target_position");

    while (client->wait_for_service(std::chrono::seconds(1)) == false) {
        RCLCPP_WARN(get_logger(), "waiting for server ...");
    }

    auto request = std::make_shared<turtle_catcher::srv::TargetPosition::Request>();
    request->x = m_target_x;
    request->y = m_target_y;

    auto future = client->async_send_request(request);
    try {
        auto response = future.get();
        RCLCPP_INFO(get_logger(), "target position sent: (%f; %f).", m_target_x, m_target_y);
        return true;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(get_logger(), "send_target_position_impl: %s", e.what());
        return false;
    }
}

void GameMaster::subscribe_pose(const turtlesim::msg::Pose::SharedPtr pose)
{
    m_pose = pose;
}
