#include <iostream>
#include <stdexcept>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "runners.hpp"

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    if (argc != 3)
    {
        std::cerr
            << "please intput: rosrun vins kitti_odom_test [config file] [data folder]\n"
            << "for example: rosrun vins kitti_odom_test "
            << "~/catkin_ws/src/VINS-Fusion/config/kitti_odom/kitti_config00-02.yaml "
            << "/media/tony-ws1/disk_D/kitti/odometry/sequences/00/\n";
        return 1;
    }

    try
    {
        rclcpp::NodeOptions options;
        options.append_parameter_override("config_file", std::string(argv[1]));
        options.append_parameter_override("sequence_path", std::string(argv[2]));

        vins::KittiOdomTestRunner runner(options);
        runner.wait();
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
