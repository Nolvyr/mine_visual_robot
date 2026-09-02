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
     * MapPoint通信操作类型
     * ==========================================================
     *
     * 0x01 = ADD
     * 0x02 = UPDATE
     * 0x03 = DELETE
     */
    enum class MapPointBatchOperation : std::uint8_t
    {
        ADD = 0x01,
        UPDATE = 0x02,
        DELETE = 0x03
    };


    /*
     * ==========================================================
     * 一条待发送的MapPoint操作
     * ==========================================================
     */
    struct MapPointBatchData
    {
        MapPointBatchOperation operation{
            MapPointBatchOperation::ADD};


        /*
         * V5.1：
         *
         * 当前MapPoint属于Atlas中的哪一张Map。
         */
        std::uint16_t map_id{0};


        /*
         * UPDATE之前所属Map。
         *
         * Merge之后可能出现：
         *
         * Map1 -> Map0
         */
        std::uint16_t previous_map_id{0};


        /*
         * ORB-SLAM3 MapPoint mnId
         *
         * 通信压缩为uint32。
         */
        std::uint32_t id{0};


        /*
         * 原始ORB-SLAM3 float坐标。
         *
         * 只用于本地调试。
         *
         * 不发送。
         */
        float raw_x{0.0F};
        float raw_y{0.0F};
        float raw_z{0.0F};


        /*
         * V4开始：
         *
         * 真正通信的数据。
         *
         * 单位：
         * cm
         */
        std::int16_t x_cm{0};
        std::int16_t y_cm{0};
        std::int16_t z_cm{0};


        /*
         * UPDATE / DELETE之前的状态。
         *
         * 只用于本地调试。
         */
        std::int16_t previous_x_cm{0};
        std::int16_t previous_y_cm{0};
        std::int16_t previous_z_cm{0};


        /*
         * UPDATE的三维变化距离。
         *
         * 单位：
         * cm
         */
        float delta_cm{0.0F};


        /*
         * V5.1：
         *
         * 同一个Point ID是否改变了所属Map。
         *
         * 例如Merge：
         *
         * Map1 -> Map0
         */
        bool map_changed{false};
    };


    /*
     * ==========================================================
     * S100当前认为PC已经同步的状态
     * ==========================================================
     *
     * Key：
     *
     * Point ID
     *
     * Value：
     *
     * Map ID
     * XYZ
     * missing_count
     */
    struct SyncedMapPointState
    {
        /*
         * 当前PC认为这个Point属于哪张Map。
         */
        std::uint16_t map_id{0};


        /*
         * PC已经同步的整数厘米坐标。
         */
        std::int16_t x_cm{0};
        std::int16_t y_cm{0};
        std::int16_t z_cm{0};


        /*
         * 连续多少次Atlas快照没有看到这个Point。
         *
         * 达到阈值才DELETE。
         */
        std::uint8_t missing_count{0};
    };


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
    void PrepareMapPointBatch();


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
    std::vector<std::uint8_t> BuildMapPointBinaryFrame(
        const std::vector<MapPointBatchData>& batch,
        std::uint32_t sequence,
        std::uint16_t active_map_id,
        std::size_t& payload_bytes,
        std::uint16_t& crc_value) const;


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
     * 通信同步缓存
     * ==========================================================
     *
     * PointID -> 已同步状态
     *
     * 注意：
     *
     * Point ID仍然是身份。
     *
     * Map ID只是Point当前所属Atlas Map属性。
     */
    std::unordered_map<
        std::uint32_t,
        SyncedMapPointState>
        synced_map_points_;


    /*
     * ==========================================================
     * 通信累计统计
     * ==========================================================
     */
    std::size_t map_point_add_total_{0};


    std::size_t map_point_update_total_{0};


    std::size_t map_point_delete_total_{0};


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


    /*
     * ==========================================================
     * V5.1 Atlas状态检测
     * ==========================================================
     */
    bool atlas_state_initialized_{false};


    /*
     * 上一次看到的有效Atlas Map数量。
     */
    std::size_t last_live_map_count_{0};


    /*
     * 上一次Active Map ID。
     *
     * 0xFFFF：
     * 无有效Map。
     */
    std::uint16_t last_active_map_id_{0xFFFFU};


    /*
     * ==========================================================
     * Tracking
     * ==========================================================
     */
    int last_tracking_state_{-999};


    bool has_valid_pose_{false};
};


#endif