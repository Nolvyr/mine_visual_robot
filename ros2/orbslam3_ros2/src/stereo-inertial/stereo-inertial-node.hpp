#ifndef STEREO_INERTIAL_NODE_HPP
#define STEREO_INERTIAL_NODE_HPP

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <mutex>
#include <queue>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"

#include <cv_bridge/cv_bridge.h>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include "System.h"
#include "utility.hpp"

using ImuMsg = sensor_msgs::msg::Imu;
using ImageMsg = sensor_msgs::msg::Image;

class StereoInertialNode : public rclcpp::Node
{
public:
    StereoInertialNode(
        ORB_SLAM3::System *pSLAM,
        const std::string &strSettingsFile,
        const std::string &strDoRectify,
        const std::string &strDoEqual);

    ~StereoInertialNode() override;

private:
    void GrabImu(const ImuMsg::SharedPtr msg);

    void GrabImageLeft(
        const ImageMsg::SharedPtr msgLeft);

    void GrabImageRight(
        const ImageMsg::SharedPtr msgRight);

    cv::Mat GetImage(
        const ImageMsg::SharedPtr msg);

    void SyncWithImu();

private:
    rclcpp::Subscription<ImuMsg>::SharedPtr subImu_;

    rclcpp::Subscription<ImageMsg>::SharedPtr
        subImgLeft_;

    rclcpp::Subscription<ImageMsg>::SharedPtr
        subImgRight_;

    ORB_SLAM3::System *SLAM_;

    /*
     * 使用thread对象，而不是new thread指针，
     * 避免手动delete和异常情况下的资源泄漏。
     */
    std::thread syncThread_;
    std::atomic<bool> running_;

    /*
     * IMU缓存。
     */
    std::queue<ImuMsg::SharedPtr> imuBuf_;
    std::mutex bufMutexImu_;

    /*
     * 左右图像缓存。
     */
    std::queue<ImageMsg::SharedPtr> imgLeftBuf_;
    std::queue<ImageMsg::SharedPtr> imgRightBuf_;

    std::mutex bufMutexLeft_;
    std::mutex bufMutexRight_;

    /*
     * 输入时间戳检查。
     */
    double lastImuInputTime_;
    double lastLeftInputTime_;
    double lastRightInputTime_;

    std::uint64_t imuMessageCount_;

    /*
     * 图像处理选项。
     */
    bool doRectify_;
    bool doEqual_;
    bool bClahe_;

    cv::Mat M1l_;
    cv::Mat M2l_;
    cv::Mat M1r_;
    cv::Mat M2r_;

    cv::Ptr<cv::CLAHE> clahe_;
};

#endif