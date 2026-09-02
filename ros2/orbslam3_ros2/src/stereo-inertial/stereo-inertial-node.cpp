#include "stereo-inertial-node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <utility>
#include <vector>

#include "sensor_msgs/image_encodings.hpp"

#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

using std::placeholders::_1;

namespace
{

bool ParseBool(const std::string &value)
{
    bool result = false;

    std::istringstream stream(value);
    stream >> std::boolalpha >> result;

    if (!stream.fail())
    {
        return result;
    }

    /*
     * 同时兼容0和1。
     */
    return value == "1";
}

bool IsFiniteImu(const ImuMsg::SharedPtr &msg)
{
    return
        std::isfinite(msg->linear_acceleration.x) &&
        std::isfinite(msg->linear_acceleration.y) &&
        std::isfinite(msg->linear_acceleration.z) &&
        std::isfinite(msg->angular_velocity.x) &&
        std::isfinite(msg->angular_velocity.y) &&
        std::isfinite(msg->angular_velocity.z);
}

double VectorNorm(
    double x,
    double y,
    double z)
{
    return std::sqrt(
        x * x +
        y * y +
        z * z);
}

} // namespace

StereoInertialNode::StereoInertialNode(
    ORB_SLAM3::System *SLAM,
    const std::string &strSettingsFile,
    const std::string &strDoRectify,
    const std::string &strDoEqual)
    : Node("ORB_SLAM3_ROS2"),
      SLAM_(SLAM),
      running_(true),
      lastImuInputTime_(-1.0),
      lastLeftInputTime_(-1.0),
      lastRightInputTime_(-1.0),
      imuMessageCount_(0),
      doRectify_(ParseBool(strDoRectify)),
      doEqual_(ParseBool(strDoEqual)),
      bClahe_(doEqual_),
      clahe_(cv::createCLAHE(
          3.0,
          cv::Size(8, 8)))
{
    if (SLAM_ == nullptr)
    {
        throw std::runtime_error(
            "ORB-SLAM3 System pointer is null");
    }

    std::cout
        << "Rectify: "
        << doRectify_
        << std::endl;

    std::cout
        << "Equal: "
        << doEqual_
        << std::endl;

    /*
     * 只有输入原始、未矫正图像时才开启。
     *
     * 你当前订阅的是：
     * infra1/image_rect_raw
     * infra2/image_rect_raw
     *
     * 所以启动参数应该传false。
     */
    if (doRectify_)
    {
        cv::FileStorage fsSettings(
            strSettingsFile,
            cv::FileStorage::READ);

        if (!fsSettings.isOpened())
        {
            throw std::runtime_error(
                "Cannot open settings file: " +
                strSettingsFile);
        }

        cv::Mat K_l;
        cv::Mat K_r;
        cv::Mat P_l;
        cv::Mat P_r;
        cv::Mat R_l;
        cv::Mat R_r;
        cv::Mat D_l;
        cv::Mat D_r;

        fsSettings["LEFT.K"] >> K_l;
        fsSettings["RIGHT.K"] >> K_r;

        fsSettings["LEFT.P"] >> P_l;
        fsSettings["RIGHT.P"] >> P_r;

        fsSettings["LEFT.R"] >> R_l;
        fsSettings["RIGHT.R"] >> R_r;

        fsSettings["LEFT.D"] >> D_l;
        fsSettings["RIGHT.D"] >> D_r;

        const int rows_l =
            static_cast<int>(
                fsSettings["LEFT.height"]);

        const int cols_l =
            static_cast<int>(
                fsSettings["LEFT.width"]);

        const int rows_r =
            static_cast<int>(
                fsSettings["RIGHT.height"]);

        const int cols_r =
            static_cast<int>(
                fsSettings["RIGHT.width"]);

        if (K_l.empty() ||
            K_r.empty() ||
            P_l.empty() ||
            P_r.empty() ||
            R_l.empty() ||
            R_r.empty() ||
            D_l.empty() ||
            D_r.empty() ||
            rows_l <= 0 ||
            rows_r <= 0 ||
            cols_l <= 0 ||
            cols_r <= 0)
        {
            throw std::runtime_error(
                "Stereo rectification parameters "
                "are missing from settings file");
        }

        cv::initUndistortRectifyMap(
            K_l,
            D_l,
            R_l,
            P_l.rowRange(0, 3).colRange(0, 3),
            cv::Size(cols_l, rows_l),
            CV_32F,
            M1l_,
            M2l_);

        cv::initUndistortRectifyMap(
            K_r,
            D_r,
            R_r,
            P_r.rowRange(0, 3).colRange(0, 3),
            cv::Size(cols_r, rows_r),
            CV_32F,
            M1r_,
            M2r_);
    }

    /*
     * RealSense发布SensorDataQoS。
     *
     * 默认SensorDataQoS深度较小，
     * 这里将200Hz IMU订阅缓存增加到500。
     */
    auto imuQos =
        rclcpp::SensorDataQoS();

    imuQos.keep_last(500);

    /*
     * 图像使用SensorDataQoS以匹配RealSense发布端。
     */
    auto imageQos =
        rclcpp::SensorDataQoS();

    imageQos.keep_last(30);

    subImu_ =
        this->create_subscription<ImuMsg>(
            "/imu",
            imuQos,
            std::bind(
                &StereoInertialNode::GrabImu,
                this,
                _1));

    subImgLeft_ =
        this->create_subscription<ImageMsg>(
            "/left/image_rect",
            imageQos,
            std::bind(
                &StereoInertialNode::GrabImageLeft,
                this,
                _1));

    subImgRight_ =
        this->create_subscription<ImageMsg>(
            "/right/image_rect",
            imageQos,
            std::bind(
                &StereoInertialNode::GrabImageRight,
                this,
                _1));

    syncThread_ =
        std::thread(
            &StereoInertialNode::SyncWithImu,
            this);
}

StereoInertialNode::~StereoInertialNode()
{
    running_.store(false);

    if (syncThread_.joinable())
    {
        syncThread_.join();
    }

    if (SLAM_ != nullptr)
    {
        SLAM_->Shutdown();

        try
        {
            SLAM_->SaveKeyFrameTrajectoryTUM(
                "KeyFrameTrajectory.txt");
        }
        catch (const std::exception &e)
        {
            std::cerr
                << "Failed to save trajectory: "
                << e.what()
                << std::endl;
        }
    }
}

void StereoInertialNode::GrabImu(
    const ImuMsg::SharedPtr msg)
{
    if (!msg)
    {
        return;
    }

    if (!IsFiniteImu(msg))
    {
        RCLCPP_WARN(
            this->get_logger(),
            "Drop IMU containing NaN or Inf");

        return;
    }

    const double currentTime =
        Utility::StampToSec(
            msg->header.stamp);

    std::lock_guard<std::mutex> lock(
        bufMutexImu_);

    /*
     * ORB-SLAM3要求IMU时间戳严格递增。
     */
    if (lastImuInputTime_ >= 0.0 &&
        currentTime <= lastImuInputTime_)
    {
        RCLCPP_WARN(
            this->get_logger(),
            "Drop non-monotonic IMU: "
            "current=%.9f previous=%.9f",
            currentTime,
            lastImuInputTime_);

        return;
    }

    lastImuInputTime_ = currentTime;

    imuBuf_.push(msg);

    /*
     * 最多保存约10秒的200Hz IMU。
     * 防止节点异常卡顿时无限增长。
     */
    while (imuBuf_.size() > 2000)
    {
        imuBuf_.pop();
    }

    imuMessageCount_++;

    /*
     * 每约1秒打印一次IMU状态。
     */
    if (imuMessageCount_ % 200 == 0)
    {
        const double accNorm =
            VectorNorm(
                msg->linear_acceleration.x,
                msg->linear_acceleration.y,
                msg->linear_acceleration.z);

        const double gyroNorm =
            VectorNorm(
                msg->angular_velocity.x,
                msg->angular_velocity.y,
                msg->angular_velocity.z);

        RCLCPP_INFO(
            this->get_logger(),
            "[IMU] count=%lu frame=%s "
            "|acc|=%.4f |gyro|=%.4f "
            "buffer=%zu",
            static_cast<unsigned long>(
                imuMessageCount_),
            msg->header.frame_id.c_str(),
            accNorm,
            gyroNorm,
            imuBuf_.size());
    }
}

void StereoInertialNode::GrabImageLeft(
    const ImageMsg::SharedPtr msgLeft)
{
    if (!msgLeft)
    {
        return;
    }

    const double timestamp =
        Utility::StampToSec(
            msgLeft->header.stamp);

    std::lock_guard<std::mutex> lock(
        bufMutexLeft_);

    if (lastLeftInputTime_ >= 0.0 &&
        timestamp <= lastLeftInputTime_)
    {
        RCLCPP_WARN(
            this->get_logger(),
            "Drop non-monotonic left image: "
            "current=%.9f previous=%.9f",
            timestamp,
            lastLeftInputTime_);

        return;
    }

    lastLeftInputTime_ = timestamp;

    imgLeftBuf_.push(msgLeft);

    /*
     * 保存最近10帧。
     *
     * 30FPS下约为333ms缓存。
     * 不宜无限保存，否则SLAM会处理很久以前的图像。
     */
    while (imgLeftBuf_.size() > 10)
    {
        imgLeftBuf_.pop();
    }
}

void StereoInertialNode::GrabImageRight(
    const ImageMsg::SharedPtr msgRight)
{
    if (!msgRight)
    {
        return;
    }

    const double timestamp =
        Utility::StampToSec(
            msgRight->header.stamp);

    std::lock_guard<std::mutex> lock(
        bufMutexRight_);

    if (lastRightInputTime_ >= 0.0 &&
        timestamp <= lastRightInputTime_)
    {
        RCLCPP_WARN(
            this->get_logger(),
            "Drop non-monotonic right image: "
            "current=%.9f previous=%.9f",
            timestamp,
            lastRightInputTime_);

        return;
    }

    lastRightInputTime_ = timestamp;

    imgRightBuf_.push(msgRight);

    while (imgRightBuf_.size() > 10)
    {
        imgRightBuf_.pop();
    }
}

cv::Mat StereoInertialNode::GetImage(
    const ImageMsg::SharedPtr msg)
{
    if (!msg)
    {
        return cv::Mat();
    }

    try
    {
        const cv_bridge::CvImageConstPtr cvPtr =
            cv_bridge::toCvShare(
                msg,
                sensor_msgs::image_encodings::MONO8);

        if (!cvPtr)
        {
            RCLCPP_ERROR(
                this->get_logger(),
                "cv_bridge returned null image");

            return cv::Mat();
        }

        /*
         * clone很重要。
         * 回调消息释放后图像仍然需要交给ORB-SLAM3。
         */
        return cvPtr->image.clone();
    }
    catch (const cv_bridge::Exception &e)
    {
        RCLCPP_ERROR(
            this->get_logger(),
            "cv_bridge exception: %s",
            e.what());

        return cv::Mat();
    }
}

void StereoInertialNode::SyncWithImu()
{
    /*
     * D435i左右红外为硬件同步。
     * 正常时间差应接近0。
     */
    const double maxStereoTimeDiff = 0.01;

    unsigned long frameCount = 0;
    unsigned long droppedOldImageCount = 0;
    unsigned long insufficientImuCount = 0;

    double lastProcessedImageTime = -1.0;

    while (running_.load() &&
           rclcpp::ok())
    {
        ImageMsg::SharedPtr msgLeft;
        ImageMsg::SharedPtr msgRight;

        std::vector<ORB_SLAM3::IMU::Point>
            vImuMeas;

        double tImLeft = 0.0;
        double tImRight = 0.0;

        double firstImuTime = 0.0;
        double lastImuTime = 0.0;

        double imageGapMs = 0.0;

        bool dataReady = false;
        bool imageTooOld = false;

        /*
         * 同时锁定三个队列，保证本次图像与IMU
         * 提取过程中的队列状态一致。
         */
        {
            std::unique_lock<std::mutex> lockLeft(
                bufMutexLeft_,
                std::defer_lock);

            std::unique_lock<std::mutex> lockRight(
                bufMutexRight_,
                std::defer_lock);

            std::unique_lock<std::mutex> lockImu(
                bufMutexImu_,
                std::defer_lock);

            std::lock(
                lockLeft,
                lockRight,
                lockImu);

            if (!imgLeftBuf_.empty() &&
                !imgRightBuf_.empty() &&
                !imuBuf_.empty())
            {
                /*
                 * 丢弃左右图中时间较早的一侧，
                 * 直到找到时间差允许的图像对。
                 */
                while (!imgLeftBuf_.empty() &&
                       !imgRightBuf_.empty())
                {
                    tImLeft =
                        Utility::StampToSec(
                            imgLeftBuf_
                                .front()
                                ->header.stamp);

                    tImRight =
                        Utility::StampToSec(
                            imgRightBuf_
                                .front()
                                ->header.stamp);

                    const double dt =
                        tImLeft - tImRight;

                    if (std::fabs(dt) <=
                        maxStereoTimeDiff)
                    {
                        break;
                    }

                    if (dt < 0.0)
                    {
                        /*
                         * 左图更早，丢弃左图。
                         */
                        imgLeftBuf_.pop();
                    }
                    else
                    {
                        /*
                         * 右图更早，丢弃右图。
                         */
                        imgRightBuf_.pop();
                    }
                }

                if (!imgLeftBuf_.empty() &&
                    !imgRightBuf_.empty() &&
                    !imuBuf_.empty())
                {
                    tImLeft =
                        Utility::StampToSec(
                            imgLeftBuf_
                                .front()
                                ->header.stamp);

                    tImRight =
                        Utility::StampToSec(
                            imgRightBuf_
                                .front()
                                ->header.stamp);

                    const double oldestImuTime =
                        Utility::StampToSec(
                            imuBuf_
                                .front()
                                ->header.stamp);

                    const double newestImuTime =
                        Utility::StampToSec(
                            imuBuf_
                                .back()
                                ->header.stamp);

                    /*
                     * IMU尚未覆盖到当前图像时间。
                     * 保留图像并等待后续IMU。
                     */
                    if (newestImuTime < tImLeft)
                    {
                        dataReady = false;
                    }
                    /*
                     * 最旧IMU已经比图像更新，
                     * 说明该图像之前的IMU已经丢失，
                     * 无法进行完整预积分。
                     */
                    else if (oldestImuTime > tImLeft)
                    {
                        imgLeftBuf_.pop();
                        imgRightBuf_.pop();

                        imageTooOld = true;
                    }
                    else
                    {
                        msgLeft =
                            imgLeftBuf_.front();

                        msgRight =
                            imgRightBuf_.front();

                        imgLeftBuf_.pop();
                        imgRightBuf_.pop();

                        /*
                         * 提取IMU直到包含：
                         *
                         * 当前图像时间之前的IMU
                         * 加上当前图像时间之后的第一条IMU
                         *
                         * 这样ORB-SLAM3可以在图像时间边界插值。
                         */
                        while (!imuBuf_.empty())
                        {
                            const auto imu =
                                imuBuf_.front();

                            const double t =
                                Utility::StampToSec(
                                    imu->header.stamp);

                            const cv::Point3f acc(
                                static_cast<float>(
                                    imu->linear_acceleration.x),
                                static_cast<float>(
                                    imu->linear_acceleration.y),
                                static_cast<float>(
                                    imu->linear_acceleration.z));

                            const cv::Point3f gyro(
                                static_cast<float>(
                                    imu->angular_velocity.x),
                                static_cast<float>(
                                    imu->angular_velocity.y),
                                static_cast<float>(
                                    imu->angular_velocity.z));

                            if (vImuMeas.empty())
                            {
                                firstImuTime = t;
                            }

                            lastImuTime = t;

                            /*
                             * ORB_SLAM3::IMU::Point顺序：
                             * acceleration, gyro, timestamp。
                             */
                            vImuMeas.emplace_back(
                                acc,
                                gyro,
                                t);

                            imuBuf_.pop();

                            /*
                             * 已经取得图像时间之后
                             * 或恰好等于图像时间的第一条IMU。
                             */
                            if (t >= tImLeft)
                            {
                                break;
                            }
                        }

                        /*
                         * 图像时间戳必须严格递增。
                         */
                        if (lastProcessedImageTime >= 0.0 &&
                            tImLeft <= lastProcessedImageTime)
                        {
                            msgLeft.reset();
                            msgRight.reset();
                            vImuMeas.clear();
                        }
                        else if (vImuMeas.size() >= 2)
                        {
                            if (lastProcessedImageTime >= 0.0)
                            {
                                imageGapMs =
                                    (tImLeft -
                                     lastProcessedImageTime) *
                                    1000.0;
                            }

                            lastProcessedImageTime =
                                tImLeft;

                            dataReady = true;
                        }
                        else
                        {
                            /*
                             * 至少需要两条采样，
                             * 否则无法形成积分时间段。
                             */
                            insufficientImuCount++;
                        }
                    }
                }
            }
        }

        if (imageTooOld)
        {
            droppedOldImageCount++;

            if (droppedOldImageCount % 30 == 0)
            {
                RCLCPP_WARN(
                    this->get_logger(),
                    "[SYNC] dropped old image pairs=%lu",
                    droppedOldImageCount);
            }
        }

        if (!dataReady)
        {
            if (insufficientImuCount > 0 &&
                insufficientImuCount % 30 == 0)
            {
                RCLCPP_WARN(
                    this->get_logger(),
                    "[SYNC] insufficient IMU "
                    "events=%lu",
                    insufficientImuCount);

                /*
                 * 防止每次循环重复打印同一个计数。
                 */
                insufficientImuCount++;
            }

            std::this_thread::sleep_for(
                std::chrono::milliseconds(1));

            continue;
        }

        cv::Mat imLeft =
            GetImage(msgLeft);

        cv::Mat imRight =
            GetImage(msgRight);

        if (imLeft.empty() ||
            imRight.empty())
        {
            RCLCPP_WARN(
                this->get_logger(),
                "[SYNC] empty stereo image");

            continue;
        }

        if (bClahe_)
        {
            clahe_->apply(
                imLeft,
                imLeft);

            clahe_->apply(
                imRight,
                imRight);
        }

        if (doRectify_)
        {
            cv::remap(
                imLeft,
                imLeft,
                M1l_,
                M2l_,
                cv::INTER_LINEAR);

            cv::remap(
                imRight,
                imRight,
                M1r_,
                M2r_,
                cv::INTER_LINEAR);
        }

        const auto trackingStart =
            std::chrono::steady_clock::now();

        SLAM_->TrackStereo(
            imLeft,
            imRight,
            tImLeft,
            vImuMeas);

        const auto trackingEnd =
            std::chrono::steady_clock::now();

        const double trackingTimeMs =
            std::chrono::duration<
                double,
                std::milli>(
                    trackingEnd -
                    trackingStart)
                .count();

        frameCount++;

        /*
         * 每30帧打印一次。
         *
         * 正常情况下：
         * imu_count约为7
         * lr_dt_ms接近0
         * imu_end_offset_ms应大于或等于0
         * image_gap_ms约为33.3
         */
        if (frameCount % 30 == 0)
        {
            std::cout
                << std::fixed
                << std::setprecision(6)
                << "[SYNC]"
                << " frame=" << frameCount
                << " imu_count="
                << vImuMeas.size()
                << " lr_dt_ms="
                << std::fabs(
                       tImLeft -
                       tImRight) *
                       1000.0
                << " image_gap_ms="
                << imageGapMs
                << " track_ms="
                << trackingTimeMs
                << " image_t="
                << tImLeft
                << " imu_first_t="
                << firstImuTime
                << " imu_last_t="
                << lastImuTime
                << " imu_end_offset_ms="
                << (lastImuTime -
                    tImLeft) *
                       1000.0
                << std::endl;
        }
    }
}