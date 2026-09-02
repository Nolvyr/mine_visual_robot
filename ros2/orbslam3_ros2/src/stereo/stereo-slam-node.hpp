#ifndef __STEREO_SLAM_NODE_HPP__
#define __STEREO_SLAM_NODE_HPP__

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"

#include "nav_msgs/msg/path.hpp"

#include "std_msgs/msg/header.hpp"
#include "std_msgs/msg/int32.hpp"

#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"

#include <cv_bridge/cv_bridge.h>

#include "System.h"
#include "Frame.h"
#include "Map.h"
#include "MapPoint.h"
#include "Tracking.h"

#include "utility.hpp"
#include "comm/comm_types.hpp"
#include "comm/binary_protocol.hpp"
#include "comm/comm_map.hpp"
#include "comm/comm_worker.hpp"
#include "comm/map_producer.hpp"


class StereoSlamNode : public rclcpp::Node
{
public:

    StereoSlamNode(
        ORB_SLAM3::System* pSLAM,
        const std::string& strSettingsFile,
        const std::string& strDoRectify);


    ~StereoSlamNode() override;


private:

    using ImageMsg =
        sensor_msgs::msg::Image;


    using ApproximateSyncPolicy =
        message_filters::sync_policies::ApproximateTime<
            ImageMsg,
            ImageMsg>;


    /*
     * ==========================================================
     * Stereo输入
     * ==========================================================
     */
    void GrabStereo(
        const ImageMsg::ConstSharedPtr& msgLeft,
        const ImageMsg::ConstSharedPtr& msgRight);


    /*
     * ==========================================================
     * 发布SLAM输出
     * ==========================================================
     */
    void PublishSlamOutputs(
        const Sophus::SE3f& Tcw,
        const std_msgs::msg::Header& imageHeader);


    /*
     * ==========================================================
     * RViz MapPoint
     *
     * 保留原有float MapPoint显示。
     *
     * 与通信分支完全独立。
     * ==========================================================
     */
    void PublishTrackedMapPoints(
        const std_msgs::msg::Header& imageHeader);


    /*
     * ==========================================================
     * V5.1 Atlas MapPoint Batch
     * ==========================================================
     */
    void PrepareMapPointBatch(
        const std_msgs::msg::Header& imageHeader);


    void ProcessTxCompletions();


    /*
     * ==========================================================
     * V5.1 二进制Frame
     * ==========================================================
     *
     * Header：
     *
     * AA 55
     * Version
     * Message Type
     * Session ID
     * Sequence
     * Active Map ID
     * Point Count
     * Payload Length
     *
     * Payload
     *
     * CRC16
     */


    /*
     * ==========================================================
     * ORB-SLAM3
     * ==========================================================
     */
    ORB_SLAM3::System* m_SLAM{nullptr};


    /*
     * ==========================================================
     * Stereo Rectification
     * ==========================================================
     */
    bool doRectify{false};


    cv::Mat M1l;
    cv::Mat M2l;

    cv::Mat M1r;
    cv::Mat M2r;


    cv_bridge::CvImageConstPtr cv_ptrLeft;

    cv_bridge::CvImageConstPtr cv_ptrRight;


    /*
     * ==========================================================
     * Stereo subscribers
     * ==========================================================
     */
    std::shared_ptr<
        message_filters::Subscriber<ImageMsg>>
        left_sub;


    std::shared_ptr<
        message_filters::Subscriber<ImageMsg>>
        right_sub;


    std::shared_ptr<
        message_filters::Synchronizer<
            ApproximateSyncPolicy>>
        syncApproximate;


    /*
     * ==========================================================
     * ROS2 publishers
     * ==========================================================
     */
    rclcpp::Publisher<
        geometry_msgs::msg::PoseStamped>::SharedPtr
        pose_pub_;


    rclcpp::Publisher<
        std_msgs::msg::Int32>::SharedPtr
        tracking_state_pub_;


    rclcpp::Publisher<
        nav_msgs::msg::Path>::SharedPtr
        path_pub_;


    rclcpp::Publisher<
        sensor_msgs::msg::PointCloud2>::SharedPtr
        map_points_pub_;


    rclcpp::Publisher<
        sensor_msgs::msg::PointCloud2>::SharedPtr
        comm_map_points_pub_;


    /*
     * ==========================================================
     * Path
     * ==========================================================
     */
    nav_msgs::msg::Path path_msg_;


    std::uint32_t path_decimation_counter_{0};


    std::uint32_t map_points_decimation_counter_{0};


    std::uint32_t map_batch_decimation_counter_{0};


    /*
     * ==========================================================
     * V5 Session
     * ==========================================================
     *
     * 每次SLAM进程启动产生新的Session ID。
     */
    std::uint32_t session_id_{0};


    /*
     * ==========================================================
     * Frame Sequence
     * ==========================================================
     */
    std::uint32_t tx_sequence_{0};


    comm::BinaryProtocol binary_protocol_;


    comm::CommMap comm_map_{0.15F};


    comm::CommWorker comm_worker_{16};
    std::unique_ptr<comm::MapProducer> gateway_producer_;


    struct PendingTx
    {
        std::uint32_t sequence{0};
        std::vector<comm::MapPointBatchData> batch;
    };


    bool has_pending_tx_{false};


    PendingTx pending_tx_;


    /*
     * ==========================================================
     * Tracking
     * ==========================================================
     */
    int last_tracking_state_{-999};


    bool has_valid_pose_{false};
};


#endif
