#include <turtle_catcher/turtle1_controller.h>

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<Turtle1Controller>();
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}