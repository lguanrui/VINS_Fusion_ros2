#include <iostream>
#include <stdexcept>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "runner.hpp"

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    if (argc != 2)
    {
        std::cerr
            << "please intput: rosrun loop_fusion loop_fusion_node [config file]\n"
            << "for example: rosrun loop_fusion loop_fusion_node "
            << "/home/tony-ws1/catkin_ws/src/VINS-Fusion/config/euroc/euroc_stereo_imu_config.yaml\n";
        return 1;
    }

    try
    {
        rclcpp::NodeOptions options;
        options.append_parameter_override("config_file", std::string(argv[1]));
        options.append_parameter_override("enable_keyboard_commands", true);

        loop_fusion::LoopFusionRunner runner(options);
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
