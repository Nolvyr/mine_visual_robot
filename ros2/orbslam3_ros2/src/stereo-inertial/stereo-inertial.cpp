#include <iostream>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "System.h"
#include "stereo-inertial-node.hpp"

int main(int argc, char **argv)
{
    if (argc < 4)
    {
        std::cerr
            << "\nUsage:\n"
            << "ros2 run orbslam3 stereo-inertial "
            << "path_to_vocabulary "
            << "path_to_settings "
            << "do_rectify "
            << "[do_equalize]\n"
            << std::endl;

        return 1;
    }

    /*
     * argv[1]：ORB词典
     * argv[2]：YAML配置文件
     * argv[3]：是否由节点进行双目矫正
     * argv[4]：是否进行CLAHE均衡化，可选
     */
    const std::string vocabularyFile = argv[1];
    const std::string settingsFile = argv[2];
    const std::string doRectify = argv[3];

    std::string doEqualize = "false";

    /*
     * 当没有显式填写第四个普通参数时，
     * argv[4]可能已经是--ros-args，不能把它当成bool。
     */
    if (argc >= 5)
    {
        const std::string argument4 = argv[4];

        if (argument4 != "--ros-args")
        {
            doEqualize = argument4;
        }
    }

    rclcpp::init(argc, argv);

    /*
     * S100上先关闭Viewer，减少CPU和GPU负载。
     */
    const bool visualization = false;

    ORB_SLAM3::System slamSystem(
        vocabularyFile,
        settingsFile,
        ORB_SLAM3::System::IMU_STEREO,
        visualization);

    auto node = std::make_shared<StereoInertialNode>(
        &slamSystem,
        settingsFile,
        doRectify,
        doEqualize);

    std::cout
        << "============================"
        << std::endl;

    rclcpp::spin(node);

    /*
     * 先析构节点，让同步线程退出并关闭SLAM，
     * 然后再结束ROS2。
     */
    node.reset();

    if (rclcpp::ok())
    {
        rclcpp::shutdown();
    }

    return 0;
}