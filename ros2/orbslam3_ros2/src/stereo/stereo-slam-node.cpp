#include "stereo-slam-node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include <Eigen/Geometry>

#include <opencv2/core/core.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "comm/binary_protocol.hpp"


using std::placeholders::_1;
using std::placeholders::_2;


namespace
{

/*
 * ==========================================================
 * ROS坐标系
 * ==========================================================
 */
constexpr const char* kOrbMapFrame =
    "orb_map";


/*
 * ==========================================================
 * Path发布
 * ==========================================================
 */
constexpr std::uint32_t kPathPublishStride =
    3;


constexpr std::size_t kMaxPathPoses =
    2000;


/*
 * ==========================================================
 * RViz MapPoint
 *
 * 这一支保持原有逻辑。
 * 与通信压缩无关。
 * ==========================================================
 */
constexpr std::uint32_t kMapPointsPublishStride =
    15;


constexpr std::size_t kMaxPublishedMapPoints =
    300;


/*
 * ==========================================================
 * V5.1 MapPoint Batch
 * ==========================================================
 *
 * V5：
 *
 * 20个点
 *
 * V5.1：
 *
 * ADD / UPDATE多了2Byte MapID。
 *
 * 为了控制后续LoRa单帧大小，
 * V5.2：最坏情况每条ADD/UPDATE为13Byte。
 * 17Byte Header + 13 * 13Byte + 2Byte CRC = 188Byte。
 */
constexpr std::size_t kMapPointsPerBatch =
    13;


/*
 * 每15个OK Stereo frame处理一次通信Batch。
 *
 * 30FPS下约2Hz。
 */
constexpr std::uint32_t kMapBatchPublishStride =
    15;


/*
 * UPDATE阈值：
 *
 * 2cm
 */
constexpr std::int32_t kMapPointUpdateThresholdCm =
    2;


/*
 * 连续两次Atlas快照不存在才DELETE。
 */
constexpr std::uint8_t kDeleteMissingBatchThreshold =
    2;


/*
 * ==========================================================
 * V5.1 Binary Protocol
 * ==========================================================
 *
 * Frame：
 *
 * AA 55
 *
 * Version       1B
 * Message Type  1B
 *
 * Session ID    4B
 * Sequence      4B
 *
 * Active Map ID 2B
 *
 * Point Count   1B
 * Payload Len   2B
 *
 * Payload
 *
 * CRC16         2B
 */


/*
 * 帧头同步字节。
 */
constexpr std::uint8_t kProtocolMagic1 =
    0xAA;


constexpr std::uint8_t kProtocolMagic2 =
    0x55;


/*
 * V5原来：
 *
 * 0x01
 *
 * V5.1增加MapID，
 * 帧结构发生不兼容变化，
 * 所以升级到：
 *
 * 0x02
 */
constexpr std::uint8_t kProtocolVersion =
    0x02;


/*
 * MapPoint Batch消息类型。
 */
constexpr std::uint8_t kMessageTypeMapPointBatch =
    0x10;


/*
 * ==========================================================
 * V5.1 Header
 * ==========================================================
 *
 * Magic         2
 * Version       1
 * Message Type  1
 * Session       4
 * Sequence      4
 * Active Map    2
 * Point Count   1
 * Payload Len   2
 *
 * 总计：
 *
 * 17 Byte
 */
constexpr std::size_t kProtocolHeaderBytes =
    17;


/*
 * CRC16：
 *
 * 2 Byte
 */
constexpr std::size_t kProtocolCrcBytes =
    2;


/*
 * ==========================================================
 * V5.1 ADD / UPDATE
 * ==========================================================
 *
 * Operation   1B
 * Map ID      2B
 * Point ID    4B
 * X           2B
 * Y           2B
 * Z           2B
 *
 * 总计：
 *
 * 13 Byte
 */
constexpr std::size_t kAddUpdateWireBytes =
    13;


/*
 * ==========================================================
 * DELETE
 * ==========================================================
 *
 * Operation   1B
 * Point ID    4B
 *
 * 总计：
 *
 * 5 Byte
 */
constexpr std::size_t kDeleteWireBytes =
    5;


/*
 * ==========================================================
 * Little Endian helpers
 * ==========================================================
 */


/*
 * ==========================================================
 * Tracking状态名称
 * ==========================================================
 */
const char* TrackingStateName(
    const int state)
{
    switch (state)
    {
        case -1:
            return "SYSTEM_NOT_READY";

        case 0:
            return "NO_IMAGES_YET";

        case 1:
            return "NOT_INITIALIZED";

        case 2:
            return "OK";

        case 3:
            return "RECENTLY_LOST";

        case 4:
            return "LOST";

        case 5:
            return "OK_KLT";

        default:
            return "UNKNOWN";
    }
}

} // namespace



/*
 * ==========================================================
 * Constructor
 * ==========================================================
 */
StereoSlamNode::StereoSlamNode(
    ORB_SLAM3::System* pSLAM,
    const std::string& strSettingsFile,
    const std::string& strDoRectify)
    :
    Node("ORB_SLAM3_ROS2"),
    m_SLAM(pSLAM)
{
    /*
     * ==========================================================
     * Rectification option
     * ==========================================================
     */

    std::stringstream ss(
        strDoRectify);


    if (!(ss >> std::boolalpha >> doRectify))
    {
        RCLCPP_WARN(
            this->get_logger(),
            "Invalid rectify option '%s', using false",
            strDoRectify.c_str());


        doRectify =
            false;
    }


    /*
     * ==========================================================
     * Session ID
     * ==========================================================
     *
     * 每次启动产生一个32bit Session。
     *
     * 用来防止不同SLAM运行周期的Point ID混在一起。
     */

    const std::uint64_t now_value =
        static_cast<std::uint64_t>(
            std::chrono::high_resolution_clock::
                now().
                time_since_epoch().
                count());


    session_id_ =
        static_cast<std::uint32_t>(
            now_value ^
            (now_value >> 32U));


    if (
        session_id_ ==
        0U)
    {
        session_id_ =
            1U;
    }


    /*
     * ==========================================================
     * Stereo Rectification
     * ==========================================================
     */

    if (doRectify)
    {
        cv::FileStorage fsSettings(
            strSettingsFile,
            cv::FileStorage::READ);


        if (!fsSettings.isOpened())
        {
            throw std::runtime_error(
                "Wrong path to ORB-SLAM3 settings file: " +
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


        fsSettings["LEFT.K"] >>
            K_l;

        fsSettings["RIGHT.K"] >>
            K_r;


        fsSettings["LEFT.P"] >>
            P_l;

        fsSettings["RIGHT.P"] >>
            P_r;


        fsSettings["LEFT.R"] >>
            R_l;

        fsSettings["RIGHT.R"] >>
            R_r;


        fsSettings["LEFT.D"] >>
            D_l;

        fsSettings["RIGHT.D"] >>
            D_r;


        const int rows_l =
            static_cast<int>(
                fsSettings[
                    "LEFT.height"]);


        const int cols_l =
            static_cast<int>(
                fsSettings[
                    "LEFT.width"]);


        const int rows_r =
            static_cast<int>(
                fsSettings[
                    "RIGHT.height"]);


        const int cols_r =
            static_cast<int>(
                fsSettings[
                    "RIGHT.width"]);


        if (
            K_l.empty() ||
            K_r.empty() ||
            P_l.empty() ||
            P_r.empty() ||
            R_l.empty() ||
            R_r.empty() ||
            D_l.empty() ||
            D_r.empty() ||
            rows_l == 0 ||
            rows_r == 0 ||
            cols_l == 0 ||
            cols_r == 0)
        {
            throw std::runtime_error(
                "Calibration parameters used to rectify "
                "stereo images are missing");
        }


        cv::initUndistortRectifyMap(
            K_l,
            D_l,
            R_l,
            P_l.rowRange(
                0,
                3).colRange(
                    0,
                    3),
            cv::Size(
                cols_l,
                rows_l),
            CV_32F,
            M1l,
            M2l);


        cv::initUndistortRectifyMap(
            K_r,
            D_r,
            R_r,
            P_r.rowRange(
                0,
                3).colRange(
                    0,
                    3),
            cv::Size(
                cols_r,
                rows_r),
            CV_32F,
            M1r,
            M2r);
    }


    /*
     * ==========================================================
     * Publishers
     * ==========================================================
     */

    pose_pub_ =
        this->create_publisher<
            geometry_msgs::msg::PoseStamped>(
                "/orbslam3/pose",
                rclcpp::QoS(10));


    tracking_state_pub_ =
        this->create_publisher<
            std_msgs::msg::Int32>(
                "/orbslam3/tracking_state",
                rclcpp::QoS(10));


    path_pub_ =
        this->create_publisher<
            nav_msgs::msg::Path>(
                "/orbslam3/path",
                rclcpp::QoS(
                    rclcpp::KeepLast(1))
                    .reliable()
                    .transient_local());


    map_points_pub_ =
        this->create_publisher<
            sensor_msgs::msg::PointCloud2>(
                "/orbslam3/map_points",
                rclcpp::QoS(1));


    comm_map_points_pub_ =
        this->create_publisher<
            sensor_msgs::msg::PointCloud2>(
                "/orbslam3/comm_map_points",
                rclcpp::QoS(1));


    path_msg_.header.frame_id =
        kOrbMapFrame;


    /*
     * ==========================================================
     * Subscribers
     * ==========================================================
     */

    const auto image_qos =
        rclcpp::SensorDataQoS()
            .get_rmw_qos_profile();


    left_sub =
        std::make_shared<
            message_filters::Subscriber<
                ImageMsg>>(
                    this,
                    "/left/image_rect",
                    image_qos);


    right_sub =
        std::make_shared<
            message_filters::Subscriber<
                ImageMsg>>(
                    this,
                    "/right/image_rect",
                    image_qos);


    syncApproximate =
        std::make_shared<
            message_filters::Synchronizer<
                ApproximateSyncPolicy>>(
                    ApproximateSyncPolicy(
                        10),
                    *left_sub,
                    *right_sub);


    syncApproximate->registerCallback(
        std::bind(
            &StereoSlamNode::GrabStereo,
            this,
            _1,
            _2));


    const auto comm_mode = declare_parameter<std::string>("comm_mode", "gateway_v3");
    if (comm_mode == "gateway_v3")
        gateway_producer_.reset(new comm::MapProducer(*this, comm_map_));
    else if (comm_mode == "legacy_v2")
        comm_worker_.Start();
    else
        throw std::invalid_argument("comm_mode must be gateway_v3 or legacy_v2");
    RCLCPP_INFO(get_logger(), "Communication mode: %s", comm_mode.c_str());


    /*
     * ==========================================================
     * Startup logs
     * ==========================================================
     */

    RCLCPP_INFO(
        this->get_logger(),
        "Stereo SLAM publishers initialized");


    RCLCPP_INFO(
        this->get_logger(),
        "Pose topic: /orbslam3/pose");


    RCLCPP_INFO(
        this->get_logger(),
        "Tracking state topic: /orbslam3/tracking_state");


    RCLCPP_INFO(
        this->get_logger(),
        "Path topic: /orbslam3/path");


    RCLCPP_INFO(
        this->get_logger(),
        "Map points topic: /orbslam3/map_points");


    RCLCPP_INFO(
        this->get_logger(),
        "MapPoint Batch V5.1 initialized: "
        "max=%zu "
        "coordinate=int16_cm "
        "point_id=uint32 "
        "map_id=uint16 "
        "update_threshold=%dcm "
        "delete_missing_batches=%u "
        "binary_protocol=AA55_V2 "
        "session=0x%08X",
        kMapPointsPerBatch,
        kMapPointUpdateThresholdCm,
        static_cast<unsigned int>(
            kDeleteMissingBatchThreshold),
        static_cast<unsigned int>(
            session_id_));
}



/*
 * ==========================================================
 * Destructor
 * ==========================================================
 */
StereoSlamNode::~StereoSlamNode()
{
    comm_worker_.Stop();


    if (
        m_SLAM ==
        nullptr)
    {
        return;
    }


    m_SLAM->Shutdown();


    if (has_valid_pose_)
    {
        m_SLAM->SaveTrajectoryTUM(
            "CameraTrajectory.txt");


        m_SLAM->SaveKeyFrameTrajectoryTUM(
            "KeyFrameTrajectory.txt");


        RCLCPP_INFO(
            this->get_logger(),
            "Saved CameraTrajectory.txt and "
            "KeyFrameTrajectory.txt");
    }
}



/*
 * ==========================================================
 * GrabStereo
 * ==========================================================
 */
void StereoSlamNode::GrabStereo(
    const ImageMsg::ConstSharedPtr& msgLeft,
    const ImageMsg::ConstSharedPtr& msgRight)
{
    try
    {
        cv_ptrLeft =
            cv_bridge::toCvShare(
                msgLeft);


        cv_ptrRight =
            cv_bridge::toCvShare(
                msgRight);
    }
    catch (
        const cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(
            this->get_logger(),
            "cv_bridge exception: %s",
            e.what());


        return;
    }


    const double timestamp =
        Utility::StampToSec(
            msgLeft->header.stamp);


    Sophus::SE3f Tcw;


    if (doRectify)
    {
        cv::Mat imLeft;
        cv::Mat imRight;


        cv::remap(
            cv_ptrLeft->image,
            imLeft,
            M1l,
            M2l,
            cv::INTER_LINEAR);


        cv::remap(
            cv_ptrRight->image,
            imRight,
            M1r,
            M2r,
            cv::INTER_LINEAR);


        Tcw =
            m_SLAM->TrackStereo(
                imLeft,
                imRight,
                timestamp);
    }
    else
    {
        Tcw =
            m_SLAM->TrackStereo(
                cv_ptrLeft->image,
                cv_ptrRight->image,
                timestamp);
    }


    PublishSlamOutputs(
        Tcw,
        msgLeft->header);
}



/*
 * ==========================================================
 * RViz MapPoint
 * ==========================================================
 *
 * 注意：
 *
 * 这里继续使用原来的GetAllMapPoints()。
 *
 * 这一支只是RViz本地观察。
 *
 * 不进行cm量化。
 * 不加入MapID。
 * 不影响通信协议。
 */
void StereoSlamNode::PublishTrackedMapPoints(
    const std_msgs::msg::Header& imageHeader)
{
    const std::vector<
        ORB_SLAM3::MapPoint*>
        map_points =
            m_SLAM->GetAllMapPoints();


    if (map_points.empty())
    {
        return;
    }


    const std::size_t sampling_stride =
        std::max<std::size_t>(
            1,
            (
                map_points.size()
                +
                kMaxPublishedMapPoints
                -
                1
            )
            /
            kMaxPublishedMapPoints);


    std::vector<Eigen::Vector3f>
        valid_points;


    valid_points.reserve(
        std::min(
            map_points.size(),
            kMaxPublishedMapPoints));


    for (
        std::size_t i = 0;
        i < map_points.size();
        i += sampling_stride)
    {
        ORB_SLAM3::MapPoint* pMP =
            map_points[i];


        if (
            pMP == nullptr ||
            pMP->isBad())
        {
            continue;
        }


        const Eigen::Vector3f world_pos =
            pMP->GetWorldPos();


        if (
            !std::isfinite(
                world_pos.x()) ||
            !std::isfinite(
                world_pos.y()) ||
            !std::isfinite(
                world_pos.z()))
        {
            continue;
        }


        valid_points.push_back(
            world_pos);


        if (
            valid_points.size()
            >=
            kMaxPublishedMapPoints)
        {
            break;
        }
    }


    if (valid_points.empty())
    {
        return;
    }


    sensor_msgs::msg::PointCloud2
        cloud_msg;


    cloud_msg.header.stamp =
        imageHeader.stamp;


    cloud_msg.header.frame_id =
        kOrbMapFrame;


    cloud_msg.height =
        1;


    sensor_msgs::PointCloud2Modifier
        modifier(
            cloud_msg);


    modifier.setPointCloud2FieldsByString(
        1,
        "xyz");


    modifier.resize(
        valid_points.size());


    sensor_msgs::PointCloud2Iterator<float>
        iter_x(
            cloud_msg,
            "x");


    sensor_msgs::PointCloud2Iterator<float>
        iter_y(
            cloud_msg,
            "y");


    sensor_msgs::PointCloud2Iterator<float>
        iter_z(
            cloud_msg,
            "z");


    for (
        const Eigen::Vector3f& point :
        valid_points)
    {
        *iter_x =
            point.x();


        *iter_y =
            point.y();


        *iter_z =
            point.z();


        ++iter_x;
        ++iter_y;
        ++iter_z;
    }


    cloud_msg.is_dense =
        true;


    map_points_pub_->publish(
        cloud_msg);
}



/*
 * ==========================================================
 * V5.1 Binary Frame Builder
 * ==========================================================
 */



/*
 * ==========================================================
 * V5.1 Atlas MapPoint Batch
 * ==========================================================
 */
void StereoSlamNode::PrepareMapPointBatch(
    const std_msgs::msg::Header& imageHeader)
{
    ProcessTxCompletions();

    const auto comm_map_begin =
        std::chrono::steady_clock::now();

    const comm::CommMapResult result =
        comm_map_.Update(m_SLAM);

    const auto comm_map_end =
        std::chrono::steady_clock::now();

    if (!result.points.empty())
    {
        sensor_msgs::msg::PointCloud2 cloud_msg;
        cloud_msg.header.stamp = imageHeader.stamp;
        cloud_msg.header.frame_id = kOrbMapFrame;
        cloud_msg.height = 1;
        sensor_msgs::PointCloud2Modifier modifier(cloud_msg);
        modifier.setPointCloud2FieldsByString(1, "xyz");
        modifier.resize(result.points.size());
        sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_msg, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(cloud_msg, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(cloud_msg, "z");
        for (const comm::CommPoint& point : result.points)
        {
            *iter_x = point.x; *iter_y = point.y; *iter_z = point.z;
            ++iter_x; ++iter_y; ++iter_z;
        }
        cloud_msg.is_dense = true;
        comm_map_points_pub_->publish(cloud_msg);
    }

    RCLCPP_INFO(
        this->get_logger(),
        "[CommMapV5.2] maps=%zu active_map=%u raw=%zu "
        "orb_valid=%zu candidates=%zu voxel=%zu dropped=%zu "
        "keep=%.2f%% voxel_size=%.2fm",
        result.stats.live_map_count,
        static_cast<unsigned int>(result.active_map_id),
        result.stats.raw_point_count,
        result.stats.orb_valid_count,
        result.stats.candidate_count,
        result.stats.voxel_count,
        result.stats.dropped_count,
        result.stats.keep_percent,
        static_cast<double>(comm_map_.voxel_size_m()));

    if (result.atlas_state_changed)
    {
        RCLCPP_INFO(
            this->get_logger(),
            "[AtlasV5.1] live_maps=%zu active_map=%u atlas_points=%zu",
            result.stats.live_map_count,
            static_cast<unsigned int>(result.active_map_id),
            result.stats.voxel_count);
    }

    if (gateway_producer_)
    {
        if (!gateway_producer_->pending() && !result.batch.empty())
        {
            if (gateway_producer_->Submit(result, session_id_, tx_sequence_)) ++tx_sequence_;
        }
        RCLCPP_INFO(get_logger(), "[CommPerf] comm_map=%ldus mode=gateway_v3 pending=%d ready=%d",
            static_cast<long>(std::chrono::duration_cast<std::chrono::microseconds>(
                comm_map_end - comm_map_begin).count()),
            gateway_producer_->pending(), gateway_producer_->ready());
        return;
    }

    if (has_pending_tx_)
    {
        RCLCPP_INFO(
            this->get_logger(),
            "[CommPendingV6.1] waiting_seq=%u batch=%zu",
            static_cast<unsigned int>(pending_tx_.sequence),
            pending_tx_.batch.size());
        return;
    }

    if (result.batch.empty()) return;

    const std::uint32_t sequence = tx_sequence_++;
    const auto binary_begin =
        std::chrono::steady_clock::now();

    comm::BinaryFrameResult frame_result =
        binary_protocol_.BuildMapPointFrame(
            result.batch, session_id_, sequence, result.active_map_id);

    const auto binary_end =
        std::chrono::steady_clock::now();

    const std::size_t binary_frame_size =
        frame_result.frame.size();

    RCLCPP_INFO(
        this->get_logger(),
        "[BinaryV5.1] session=0x%08X seq=%u active_map=%u count=%zu "
        "payload=%zuB frame=%zuB crc=0x%04X",
        static_cast<unsigned int>(session_id_),
        static_cast<unsigned int>(sequence),
        static_cast<unsigned int>(result.active_map_id),
        result.batch.size(), frame_result.payload_bytes,
        binary_frame_size, static_cast<unsigned int>(frame_result.crc));

    const std::string hex_preview =
        comm::BinaryProtocol::MakeHexPreview(frame_result.frame);
    RCLCPP_INFO(this->get_logger(), "  HEX: %s", hex_preview.c_str());

    for (const comm::MapPointBatchData& point : result.batch)
    {
        if (point.operation == comm::MapPointOperation::UPDATE && point.map_changed)
        {
            RCLCPP_INFO(
                this->get_logger(),
                "  MAP_MOVE id=%u map=%u->%u xyz=(%d,%d,%d)cm delta=%.2fcm",
                static_cast<unsigned int>(point.id),
                static_cast<unsigned int>(point.previous_map_id),
                static_cast<unsigned int>(point.map_id),
                static_cast<int>(point.x_cm), static_cast<int>(point.y_cm),
                static_cast<int>(point.z_cm), point.delta_cm);
        }
    }

    RCLCPP_INFO(
        this->get_logger(),
        "[MapBatchV5.1] maps=%zu active_map=%u orb_valid=%zu comm_valid=%zu "
        "rejected=%zu add_pending=%zu update_pending=%zu delete_pending=%zu "
        "batch=%zu add=%zu update=%zu delete=%zu map_move=%zu synced_now=%zu "
        "payload=%zuB frame=%zuB add_total=%zu update_total=%zu delete_total=%zu",
        result.stats.live_map_count, static_cast<unsigned int>(result.active_map_id),
        result.stats.orb_valid_count, result.stats.voxel_count,
        result.stats.rejected_count, result.add_pending, result.update_pending,
        result.delete_pending, result.batch.size(), result.batch_add,
        result.batch_update, result.batch_delete, result.batch_map_move,
        result.synced_now, frame_result.payload_bytes, binary_frame_size,
        result.add_total, result.update_total, result.delete_total);

    comm::TxFrame tx_frame;
    tx_frame.sequence = sequence;
    tx_frame.active_map_id = result.active_map_id;
    tx_frame.bytes = std::move(frame_result.frame);

    const auto enqueue_begin =
        std::chrono::steady_clock::now();

    const bool enqueued =
        comm_worker_.Enqueue(std::move(tx_frame));

    const auto enqueue_end =
        std::chrono::steady_clock::now();

    const comm::CommWorkerStats worker_stats =
        comm_worker_.GetStats();

    if (!enqueued)
    {
        RCLCPP_ERROR(
            this->get_logger(),
            "[CommWorkerV6.0] enqueue rejected seq=%u queue=%zu rejected=%zu",
            static_cast<unsigned int>(sequence),
            worker_stats.queue_size,
            worker_stats.enqueue_rejected);
    }
    else
    {
        pending_tx_.sequence = sequence;
        pending_tx_.batch = result.batch;
        has_pending_tx_ = true;

        RCLCPP_INFO(
            this->get_logger(),
            "[CommPendingV6.1] seq=%u state=PENDING batch=%zu",
            static_cast<unsigned int>(pending_tx_.sequence),
            pending_tx_.batch.size());
    }

    const auto comm_map_us =
        std::chrono::duration_cast<std::chrono::microseconds>(
            comm_map_end - comm_map_begin).count();
    const auto binary_us =
        std::chrono::duration_cast<std::chrono::microseconds>(
            binary_end - binary_begin).count();
    const auto enqueue_us =
        std::chrono::duration_cast<std::chrono::microseconds>(
            enqueue_end - enqueue_begin).count();

    RCLCPP_INFO(
        this->get_logger(),
        "[CommPerf] comm_map=%ldus binary=%ldus enqueue=%ldus "
        "queue=%zu high_watermark=%zu enqueued=%zu processed=%zu",
        static_cast<long>(comm_map_us),
        static_cast<long>(binary_us),
        static_cast<long>(enqueue_us),
        worker_stats.queue_size,
        worker_stats.queue_high_watermark,
        worker_stats.enqueue_total,
        worker_stats.processed_total);

    return;

}


void StereoSlamNode::ProcessTxCompletions()
{
    const std::vector<comm::TxCompletion> completions =
        comm_worker_.DrainCompletions();

    for (const comm::TxCompletion& completion : completions)
    {
        if (!has_pending_tx_ || completion.sequence != pending_tx_.sequence)
        {
            RCLCPP_ERROR(
                this->get_logger(),
                "[CommCommitV6.1] seq=%u result=%s error=SEQUENCE_MISMATCH "
                "pending=%s pending_seq=%u",
                static_cast<unsigned int>(completion.sequence),
                completion.success ? "SUCCESS" : "FAILED",
                has_pending_tx_ ? "YES" : "NO",
                static_cast<unsigned int>(pending_tx_.sequence));
            continue;
        }

        const std::size_t batch_size = pending_tx_.batch.size();
        if (completion.success)
        {
            const comm::CommCommitResult commit =
                comm_map_.CommitBatch(pending_tx_.batch);

            RCLCPP_INFO(
                this->get_logger(),
                "[CommCommitV6.1] seq=%u result=SUCCESS batch=%zu "
                "synced_now=%zu add_total=%zu update_total=%zu delete_total=%zu",
                static_cast<unsigned int>(completion.sequence),
                batch_size,
                commit.synced_now,
                commit.add_total,
                commit.update_total,
                commit.delete_total);
        }
        else
        {
            RCLCPP_WARN(
                this->get_logger(),
                "[CommCommitV6.1] seq=%u result=FAILED commit=NO batch=%zu",
                static_cast<unsigned int>(completion.sequence),
                batch_size);
        }

        pending_tx_.batch.clear();
        has_pending_tx_ = false;
    }
}



/*
 * ==========================================================
 * Publish SLAM Outputs
 * ==========================================================
 */
void StereoSlamNode::PublishSlamOutputs(
    const Sophus::SE3f& Tcw,
    const std_msgs::msg::Header& imageHeader)
{
    /*
     * ==========================================================
     * Tracking state
     * ==========================================================
     */

    const int tracking_state =
        m_SLAM->GetTrackingState();


    std_msgs::msg::Int32
        state_msg;


    state_msg.data =
        tracking_state;


    tracking_state_pub_->publish(
        state_msg);


    if (
        tracking_state
        !=
        last_tracking_state_)
    {
        RCLCPP_INFO(
            this->get_logger(),
            "ORB-SLAM3 tracking state: %d (%s)",
            tracking_state,
            TrackingStateName(
                tracking_state));


        last_tracking_state_ =
            tracking_state;
    }


    /*
     * ==========================================================
     * 只有Tracking OK才继续发布Pose / Point / Batch
     * ==========================================================
     *
     * 这也意味着：
     *
     * LOST期间不做DELETE判断。
     *
     * 避免因为追踪失败而误删Atlas点。
     */

    if (
        tracking_state
        !=
        ORB_SLAM3::Tracking::OK)
    {
        return;
    }


    /*
     * ==========================================================
     * Camera Pose
     * ==========================================================
     */

    const Sophus::SE3f Twc =
        Tcw.inverse();


    const Eigen::Vector3f translation =
        Twc.translation();


    Eigen::Quaternionf quaternion =
        Twc.unit_quaternion();


    quaternion.normalize();


    if (
        !std::isfinite(
            translation.x())
        ||
        !std::isfinite(
            translation.y())
        ||
        !std::isfinite(
            translation.z())
        ||
        !std::isfinite(
            quaternion.x())
        ||
        !std::isfinite(
            quaternion.y())
        ||
        !std::isfinite(
            quaternion.z())
        ||
        !std::isfinite(
            quaternion.w()))
    {
        return;
    }


    geometry_msgs::msg::PoseStamped
        pose_msg;


    pose_msg.header.stamp =
        imageHeader.stamp;


    pose_msg.header.frame_id =
        kOrbMapFrame;


    pose_msg.pose.position.x =
        translation.x();


    pose_msg.pose.position.y =
        translation.y();


    pose_msg.pose.position.z =
        translation.z();


    pose_msg.pose.orientation.x =
        quaternion.x();


    pose_msg.pose.orientation.y =
        quaternion.y();


    pose_msg.pose.orientation.z =
        quaternion.z();


    pose_msg.pose.orientation.w =
        quaternion.w();


    pose_pub_->publish(
        pose_msg);


    has_valid_pose_ =
        true;


    /*
     * ==========================================================
     * RViz MapPoint
     * ==========================================================
     */

    ++map_points_decimation_counter_;


    if (
        (
            map_points_decimation_counter_
            %
            kMapPointsPublishStride
        )
        ==
        0)
    {
        PublishTrackedMapPoints(
            imageHeader);
    }


    /*
     * ==========================================================
     * Communication Batch
     * ==========================================================
     */

    ++map_batch_decimation_counter_;


    if (
        (
            map_batch_decimation_counter_
            %
            kMapBatchPublishStride
        )
        ==
        0)
    {
        PrepareMapPointBatch(
            imageHeader);
    }


    /*
     * ==========================================================
     * Path
     * ==========================================================
     */

    ++path_decimation_counter_;


    if (
        (
            path_decimation_counter_
            %
            kPathPublishStride
        )
        !=
        0)
    {
        return;
    }


    path_msg_.header.stamp =
        imageHeader.stamp;


    path_msg_.header.frame_id =
        kOrbMapFrame;


    path_msg_.poses.push_back(
        pose_msg);


    if (
        path_msg_.poses.size()
        >
        kMaxPathPoses)
    {
        path_msg_.poses.erase(
            path_msg_.poses.begin(),
            path_msg_.poses.begin()
            +
            500);
    }


    path_pub_->publish(
        path_msg_);
}
