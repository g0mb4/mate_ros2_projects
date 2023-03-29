#include <turtle_catcher/turtle1_controller.hpp>

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<Turtle1Controller>());

    rclcpp::shutdown();
    return 0;
}