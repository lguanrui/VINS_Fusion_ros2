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
            << "please intput: rosrun vins kitti_gps_test [config file] [data folder]\n"
            << "for example: rosrun vins kitti_gps_test "
            << "~/catkin_ws/src/VINS-Fusion/config/kitti_raw/kitti_10_03_config.yaml "
            << "/media/tony-ws1/disk_D/kitti/2011_10_03/2011_10_03_drive_0027_sync/\n";
        return 1;
    }

    try
    {
        rclcpp::NodeOptions options;
        options.append_parameter_override("config_file", std::string(argv[1]));
        options.append_parameter_override("sequence_path", std::string(argv[2]));

        vins::KittiGpsTestRunner runner(options);
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
