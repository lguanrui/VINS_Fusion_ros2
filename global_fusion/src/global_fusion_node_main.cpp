#include <iostream>
#include <stdexcept>

#include <rclcpp/rclcpp.hpp>

#include "runner.hpp"

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    try
    {
        global_fusion::GlobalFusionRunner runner{rclcpp::NodeOptions()};
        rclcpp::spin(runner.node());
    }
    catch (const std::exception & ex)
    {
        std::cerr << ex.what() << std::endl;
        rclcpp::shutdown();
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}
