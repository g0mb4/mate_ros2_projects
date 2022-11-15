#include <turtle_catcher/turtle1_controller.h>

Turtle1Controller::Turtle1Controller()
    : rclcpp::Node("turtle1_controller")
{
    declare_parameter<float>("P_distance", 1.0);
    declare_parameter<float>("P_angle", 1.0);
    declare_parameter<float>("tolerance", 1.0);

    m_P_distance = (float)get_parameter("P_distance").as_double();
    m_P_angle = (float)get_parameter("P_angle").as_double();
    m_tolerance = (float)get_parameter("tolerance").as_double();

    m_pose_subscriber = create_subscription<turtlesim::msg::Pose>(
        "turtle1/pose", 10,
        std::bind(&Turtle1Controller::subscribe_pose,
            this, std::placeholders::_1));

    m_cmd_vel_publisher = create_publisher<geometry_msgs::msg::Twist>(
        "turtle1/cmd_vel", 10);

    m_target_position_server = create_service<TargetPosition>(
        "target_position",
        std::bind(
            &Turtle1Controller::serve_target_position, this,
            std::placeholders::_1));

    m_control_loop_timer = create_wall_timer(std::chrono::milliseconds((int64_t)(1000.0 / 60.0)),
        std::bind(&Turtle1Controller::control_loop_tick, this));

    RCLCPP_INFO(get_logger(), "started");
}

void Turtle1Controller::control_loop_tick()
{
    if (m_pose == nullptr || m_running == false) {
        return;
    }

    auto compute_target_distance = [this] {
        float dx = m_target_x - m_pose->x;
        float dy = m_target_y - m_pose->y;
        return sqrt(dx * dx + dy * dy);
    };

    auto compute_target_angle = [this] {
        float dx = m_target_x - m_pose->x;
        float dy = m_target_y - m_pose->y;
        float goal_theta = atan2(dy, dx);

        float error = goal_theta - m_pose->theta;

        if (error > M_PI) {
            error -= 2 * M_PI;
        } else if (error < -M_PI) {
            error += 2 * M_PI;
        }

        return error;
    };

    auto msg = geometry_msgs::msg::Twist();

    float distance_error = compute_target_distance();
    float angle_error = compute_target_angle();

    if (distance_error < m_tolerance) {
        msg.linear.x = 0;
        msg.angular.z = 0;
    } else {
        msg.linear.x = m_P_distance * distance_error;
        msg.angular.z = m_P_angle * angle_error;
    }

    m_cmd_vel_publisher->publish(msg);
}

void Turtle1Controller::subscribe_pose(const turtlesim::msg::Pose::SharedPtr pose)
{
    m_pose = pose;
}

void Turtle1Controller::serve_target_position(
    const TargetPosition::Request::SharedPtr request)
{
    m_target_x = request->x;
    m_target_y = request->y;

    m_running = true;

    RCLCPP_INFO(get_logger(), "target: (%f; %f)", m_target_x, m_target_y);
}
