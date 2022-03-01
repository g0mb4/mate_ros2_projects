#include <turtle_catcher/game_master.h>

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<GameMaster>();
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}