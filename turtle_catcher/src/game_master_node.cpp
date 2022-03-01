#include <turtle_catcher/game_master.h>

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<GameMaster>());

    rclcpp::shutdown();
    return 0;
}