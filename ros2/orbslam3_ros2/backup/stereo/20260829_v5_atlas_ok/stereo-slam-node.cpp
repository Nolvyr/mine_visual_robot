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
#include <vector>

#include <Eigen/Geometry>

#include <opencv2/core/core.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

#include "sensor_msgs/point_cloud2_iterator.hpp"


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
 * 改成16条。
 */
constexpr std::size_t kMapPointsPerBatch =
    16;


/*
 * DELETE优先配额。
 *
 * 不是硬上限。
 */
constexpr std::size_t kPreferredDeletesPerBatch =
    6;


/*
 * UPDATE优先配额。
 *
 * 不是硬上限。
 */
constexpr std::size_t kPreferredUpdatesPerBatch =
    5;


/*
 * 每15个OK Stereo frame处理一次通信Batch。
 *
 * 30FPS下约2Hz。
 */
constexpr std::uint32_t kMapBatchPublishStride =
    15;


/*
 * ==========================================================
 * V4 / V5.1 坐标量化
 * ==========================================================
 *
 * meter × 100
 *
 * -> centimeter
 *
 * -> int16
 */
constexpr float kCoordinateScaleCm =
    100.0F;


/*
 * UPDATE阈值：
 *
 * 2cm
 */
constexpr std::int32_t kMapPointUpdateThresholdCm =
    2;


constexpr std::int64_t kMapPointUpdateThresholdSquaredCm =
    static_cast<std::int64_t>(
        kMapPointUpdateThresholdCm)
    *
    static_cast<std::int64_t>(
        kMapPointUpdateThresholdCm);


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

void AppendUint16LE(
    std::vector<std::uint8_t>& buffer,
    const std::uint16_t value)
{
    buffer.push_back(
        static_cast<std::uint8_t>(
            value & 0xFFU));


    buffer.push_back(
        static_cast<std::uint8_t>(
            (value >> 8U) & 0xFFU));
}



void AppendInt16LE(
    std::vector<std::uint8_t>& buffer,
    const std::int16_t value)
{
    /*
     * 保留int16二进制补码，
     * 再按uint16拆字节。
     */
    AppendUint16LE(
        buffer,
        static_cast<std::uint16_t>(
            value));
}



void AppendUint32LE(
    std::vector<std::uint8_t>& buffer,
    const std::uint32_t value)
{
    buffer.push_back(
        static_cast<std::uint8_t>(
            value & 0xFFU));


    buffer.push_back(
        static_cast<std::uint8_t>(
            (value >> 8U) & 0xFFU));


    buffer.push_back(
        static_cast<std::uint8_t>(
            (value >> 16U) & 0xFFU));


    buffer.push_back(
        static_cast<std::uint8_t>(
            (value >> 24U) & 0xFFU));
}


/*
 * ==========================================================
 * CRC-16/CCITT-FALSE
 * ==========================================================
 *
 * Width：
 * 16
 *
 * Polynomial：
 * 0x1021
 *
 * Initial：
 * 0xFFFF
 *
 * RefIn：
 * false
 *
 * RefOut：
 * false
 *
 * XorOut：
 * 0x0000
 *
 * CRC覆盖：
 *
 * AA 55
 * 一直到Payload最后一个Byte。
 *
 * CRC本身不参与计算。
 */
std::uint16_t ComputeCrc16CcittFalse(
    const std::vector<std::uint8_t>& data)
{
    std::uint16_t crc =
        0xFFFFU;


    for (
        const std::uint8_t byte :
        data)
    {
        crc ^=
            static_cast<std::uint16_t>(
                byte)
            << 8U;


        for (
            int bit = 0;
            bit < 8;
            ++bit)
        {
            if (
                (crc & 0x8000U)
                !=
                0U)
            {
                crc =
                    static_cast<std::uint16_t>(
                        (crc << 1U)
                        ^
                        0x1021U);
            }
            else
            {
                crc =
                    static_cast<std::uint16_t>(
                        crc << 1U);
            }
        }
    }


    return crc;
}


/*
 * ==========================================================
 * HEX预览
 * ==========================================================
 *
 * 默认只显示前64Byte，
 * 防止237Byte持续刷满终端。
 */
std::string MakeHexPreview(
    const std::vector<std::uint8_t>& data,
    const std::size_t max_bytes = 64)
{
    std::ostringstream oss;


    oss
        << std::uppercase
        << std::hex
        << std::setfill('0');


    const std::size_t count =
        std::min(
            data.size(),
            max_bytes);


    for (
        std::size_t i = 0;
        i < count;
        ++i)
    {
        if (i != 0)
        {
            oss << ' ';
        }


        oss
            << std::setw(2)
            << static_cast<unsigned int>(
                data[i]);
    }


    if (
        data.size() >
        max_bytes)
    {
        oss << " ...";
    }


    return oss.str();
}


/*
 * ==========================================================
 * float meter -> int16 centimeter
 * ==========================================================
 */
bool QuantizeCoordinateCm(
    const float meters,
    std::int16_t& value_cm)
{
    if (!std::isfinite(meters))
    {
        return false;
    }


    const double rounded =
        std::round(
            static_cast<double>(
                meters)
            *
            static_cast<double>(
                kCoordinateScaleCm));


    const double minimum =
        static_cast<double>(
            std::numeric_limits<
                std::int16_t>::min());


    const double maximum =
        static_cast<double>(
            std::numeric_limits<
                std::int16_t>::max());


    if (
        rounded < minimum ||
        rounded > maximum)
    {
        return false;
    }


    value_cm =
        static_cast<std::int16_t>(
            rounded);


    return true;
}


/*
 * ==========================================================
 * ORB MapPoint ID -> uint32 wire ID
 * ==========================================================
 */
bool ConvertMapPointIdToWire(
    const unsigned long id,
    std::uint32_t& wire_id)
{
    const unsigned long maximum =
        static_cast<unsigned long>(
            std::numeric_limits<
                std::uint32_t>::max());


    if (id > maximum)
    {
        return false;
    }


    wire_id =
        static_cast<std::uint32_t>(
            id);


    return true;
}


/*
 * ==========================================================
 * ORB Map ID -> uint16 wire Map ID
 * ==========================================================
 *
 * 0xFFFF保留为：
 *
 * Invalid / No Active Map
 */
bool ConvertMapIdToWire(
    const unsigned long id,
    std::uint16_t& wire_id)
{
    constexpr unsigned long kMaximumMapId =
        0xFFFEUL;


    if (id > kMaximumMapId)
    {
        return false;
    }


    wire_id =
        static_cast<std::uint16_t>(
            id);


    return true;
}


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
std::vector<std::uint8_t>
StereoSlamNode::BuildMapPointBinaryFrame(
    const std::vector<MapPointBatchData>& batch,
    const std::uint32_t sequence,
    const std::uint16_t active_map_id,
    std::size_t& payload_bytes,
    std::uint16_t& crc_value) const
{
    /*
     * ==========================================================
     * Payload
     * ==========================================================
     */

    std::vector<std::uint8_t>
        payload;


    payload.reserve(
        kMapPointsPerBatch
        *
        kAddUpdateWireBytes);


    for (
        const MapPointBatchData& point :
        batch)
    {
        /*
         * Operation
         *
         * 01 = ADD
         * 02 = UPDATE
         * 03 = DELETE
         */
        payload.push_back(
            static_cast<std::uint8_t>(
                point.operation));


        if (
            point.operation
                ==
                MapPointBatchOperation::ADD
            ||
            point.operation
                ==
                MapPointBatchOperation::UPDATE)
        {
            /*
             * ==================================================
             * ADD / UPDATE
             * ==================================================
             *
             * Operation   1B
             * Map ID      2B
             * Point ID    4B
             * X           2B
             * Y           2B
             * Z           2B
             *
             * =13B
             */


            /*
             * Map ID
             */
            AppendUint16LE(
                payload,
                point.map_id);


            /*
             * Point ID
             */
            AppendUint32LE(
                payload,
                point.id);


            /*
             * XYZ
             */
            AppendInt16LE(
                payload,
                point.x_cm);


            AppendInt16LE(
                payload,
                point.y_cm);


            AppendInt16LE(
                payload,
                point.z_cm);
        }
        else
        {
            /*
             * ==================================================
             * DELETE
             * ==================================================
             *
             * Point ID是本次Session Atlas中的全局身份。
             *
             * 因此DELETE只需要：
             *
             * Operation
             * Point ID
             *
             * =5B
             */

            AppendUint32LE(
                payload,
                point.id);
        }
    }


    payload_bytes =
        payload.size();


    /*
     * ==========================================================
     * Frame
     * ==========================================================
     */

    std::vector<std::uint8_t>
        frame;


    frame.reserve(
        kProtocolHeaderBytes
        +
        payload.size()
        +
        kProtocolCrcBytes);


    /*
     * Magic
     *
     * AA 55
     */
    frame.push_back(
        kProtocolMagic1);


    frame.push_back(
        kProtocolMagic2);


    /*
     * Version
     *
     * 02
     */
    frame.push_back(
        kProtocolVersion);


    /*
     * Message Type
     *
     * 10 = MapPoint Batch
     */
    frame.push_back(
        kMessageTypeMapPointBatch);


    /*
     * Session ID
     */
    AppendUint32LE(
        frame,
        session_id_);


    /*
     * Sequence
     */
    AppendUint32LE(
        frame,
        sequence);


    /*
     * V5.1：
     *
     * 当前ORB-SLAM3 Active Map。
     */
    AppendUint16LE(
        frame,
        active_map_id);


    /*
     * 本帧MapPoint操作条数。
     */
    frame.push_back(
        static_cast<std::uint8_t>(
            batch.size()));


    /*
     * Payload Length
     */
    AppendUint16LE(
        frame,
        static_cast<std::uint16_t>(
            payload.size()));


    /*
     * Payload
     */
    frame.insert(
        frame.end(),
        payload.begin(),
        payload.end());


    /*
     * CRC计算范围：
     *
     * AA55
     * ...
     * Payload最后一个Byte
     */
    crc_value =
        ComputeCrc16CcittFalse(
            frame);


    /*
     * CRC16小端写入。
     */
    AppendUint16LE(
        frame,
        crc_value);


    return frame;
}



/*
 * ==========================================================
 * V5.1 Atlas MapPoint Batch
 * ==========================================================
 */
void StereoSlamNode::PrepareMapPointBatch()
{
    /*
     * ==========================================================
     * 1. 获取整个Atlas
     * ==========================================================
     *
     * V5：
     *
     * GetAllMapPoints()
     *
     * 只处理当前地图。
     *
     *
     * V5.1：
     *
     * GetAllMaps()
     *
     * 获取：
     *
     * Map0
     * Map1
     * Map2
     * ...
     */

    const std::vector<ORB_SLAM3::Map*>
        atlas_maps =
            m_SLAM->GetAllMaps();


    /*
     * 当前真正Active的Map。
     */
    ORB_SLAM3::Map* active_map =
        m_SLAM->GetCurrentMap();


    /*
     * 0xFFFF：
     *
     * 无有效Active Map。
     */
    constexpr std::uint16_t
        kInvalidMapId =
            0xFFFFU;


    std::uint16_t active_map_id =
        kInvalidMapId;


    if (
        active_map != nullptr
        &&
        !active_map->IsBad())
    {
        if (
            !ConvertMapIdToWire(
                active_map->GetId(),
                active_map_id))
        {
            RCLCPP_WARN(
                this->get_logger(),
                "Active Map ID exceeds "
                "uint16 wire range: %lu",
                active_map->GetId());


            active_map_id =
                kInvalidMapId;
        }
    }


    /*
     * ==========================================================
     * 2. Atlas MapPoint Snapshot
     * ==========================================================
     */

    struct MapPointSnapshot
    {
        /*
         * 当前所属Atlas Map。
         */
        std::uint16_t map_id{0};


        /*
         * MapPoint ID。
         */
        std::uint32_t id{0};


        /*
         * 原始float。
         */
        float raw_x{0.0F};
        float raw_y{0.0F};
        float raw_z{0.0F};


        /*
         * 通信整数厘米。
         */
        std::int16_t x_cm{0};
        std::int16_t y_cm{0};
        std::int16_t z_cm{0};
    };


    /*
     * ==========================================================
     * PointID -> Snapshot
     * ==========================================================
     *
     * PointID仍然作为真正身份。
     *
     * MapID只是属性。
     *
     * 这样Merge时：
     *
     * Point100：
     *
     * Map1 -> Map0
     *
     * 仍然是同一个Point100。
     */
    std::unordered_map<
        std::uint32_t,
        MapPointSnapshot>
        snapshot_by_id;


    /*
     * ==========================================================
     * 当前整个Atlas仍然存在的Point IDs
     * ==========================================================
     *
     * DELETE判断使用这个集合。
     */
    std::unordered_set<std::uint32_t>
        current_ids;


    /*
     * ==========================================================
     * 防止Atlas Merge期间重复遍历同一个mnId
     * ==========================================================
     */
    std::unordered_set<unsigned long>
        visited_orb_ids;


    /*
     * ==========================================================
     * Statistics
     * ==========================================================
     */
    std::size_t orb_valid_count =
        0;


    std::size_t rejected_count =
        0;


    std::size_t live_map_count =
        0;


    /*
     * ==========================================================
     * 3. 遍历整个Atlas
     * ==========================================================
     */

    for (
        ORB_SLAM3::Map* pMap :
        atlas_maps)
    {
        if (
            pMap == nullptr
            ||
            pMap->IsBad())
        {
            continue;
        }


        ++live_map_count;


        /*
         * 当前这张Map里的全部MapPoint。
         */
        const std::vector<
            ORB_SLAM3::MapPoint*>
            map_points =
                pMap->GetAllMapPoints();


        for (
            ORB_SLAM3::MapPoint* pMP :
            map_points)
        {
            if (
                pMP == nullptr
                ||
                pMP->isBad())
            {
                continue;
            }


            /*
             * ==================================================
             * 防止同一个Point重复处理
             * ==================================================
             */

            const auto inserted =
                visited_orb_ids.insert(
                    pMP->mnId);


            if (!inserted.second)
            {
                continue;
            }


            /*
             * ==================================================
             * 读取坐标
             * ==================================================
             */

            const Eigen::Vector3f world_pos =
                pMP->GetWorldPos();


            if (
                !std::isfinite(
                    world_pos.x())
                ||
                !std::isfinite(
                    world_pos.y())
                ||
                !std::isfinite(
                    world_pos.z()))
            {
                continue;
            }


            ++orb_valid_count;


            /*
             * ==================================================
             * Point ID
             * ==================================================
             */

            std::uint32_t wire_point_id =
                0;


            if (
                !ConvertMapPointIdToWire(
                    pMP->mnId,
                    wire_point_id))
            {
                ++rejected_count;


                continue;
            }


            /*
             * Point真实存在于Atlas，
             * 先放进current_ids。
             *
             * 即便后面的MapID或XYZ无法通信，
             * 也不应该立即把旧Point错误DELETE。
             */
            current_ids.insert(
                wire_point_id);


            /*
             * ==================================================
             * 当前Point真正所属Map
             * ==================================================
             *
             * 注意：
             *
             * 不直接用外层pMap。
             *
             * 因为Merge过程中MapPoint可能已经：
             *
             * UpdateMap()
             *
             * 从Map1转到了Map0。
             */

            ORB_SLAM3::Map* owner_map =
                pMP->GetMap();


            if (
                owner_map == nullptr
                ||
                owner_map->IsBad())
            {
                ++rejected_count;


                continue;
            }


            std::uint16_t wire_map_id =
                0;


            if (
                !ConvertMapIdToWire(
                    owner_map->GetId(),
                    wire_map_id))
            {
                ++rejected_count;


                continue;
            }


            /*
             * ==================================================
             * float meter -> int16 centimeter
             * ==================================================
             */

            std::int16_t x_cm =
                0;


            std::int16_t y_cm =
                0;


            std::int16_t z_cm =
                0;


            if (
                !QuantizeCoordinateCm(
                    world_pos.x(),
                    x_cm)
                ||
                !QuantizeCoordinateCm(
                    world_pos.y(),
                    y_cm)
                ||
                !QuantizeCoordinateCm(
                    world_pos.z(),
                    z_cm))
            {
                ++rejected_count;


                continue;
            }


            /*
             * ==================================================
             * 保存snapshot
             * ==================================================
             */

            MapPointSnapshot snapshot;


            snapshot.map_id =
                wire_map_id;


            snapshot.id =
                wire_point_id;


            snapshot.raw_x =
                world_pos.x();


            snapshot.raw_y =
                world_pos.y();


            snapshot.raw_z =
                world_pos.z();


            snapshot.x_cm =
                x_cm;


            snapshot.y_cm =
                y_cm;


            snapshot.z_cm =
                z_cm;


            /*
             * 同一个Point ID最多保留一份。
             */
            snapshot_by_id[
                wire_point_id] =
                snapshot;
        }
    }


    /*
     * ==========================================================
     * 4. unordered_map -> vector
     * ==========================================================
     *
     * 按PointID排序。
     *
     * 这样Batch发送顺序更稳定。
     */

    std::vector<MapPointSnapshot>
        snapshots;


    snapshots.reserve(
        snapshot_by_id.size());


    for (
        const auto& entry :
        snapshot_by_id)
    {
        snapshots.push_back(
            entry.second);
    }


    std::sort(
        snapshots.begin(),
        snapshots.end(),
        [](
            const MapPointSnapshot& lhs,
            const MapPointSnapshot& rhs)
        {
            return lhs.id <
                rhs.id;
        });


    /*
     * ==========================================================
     * 5. Atlas状态日志
     * ==========================================================
     *
     * 只有：
     *
     * Map数量变化
     * 或
     * Active Map变化
     *
     * 才打印。
     */

    if (
        !atlas_state_initialized_
        ||
        live_map_count
            !=
            last_live_map_count_
        ||
        active_map_id
            !=
            last_active_map_id_)
    {
        RCLCPP_INFO(
            this->get_logger(),
            "[AtlasV5.1] "
            "live_maps=%zu "
            "active_map=%u "
            "atlas_points=%zu",
            live_map_count,
            static_cast<unsigned int>(
                active_map_id),
            snapshots.size());


        atlas_state_initialized_ =
            true;


        last_live_map_count_ =
            live_map_count;


        last_active_map_id_ =
            active_map_id;
    }


    /*
     * ==========================================================
     * 6. Candidates
     * ==========================================================
     */

    std::vector<MapPointBatchData>
        delete_candidates;


    std::vector<MapPointBatchData>
        update_candidates;


    std::vector<MapPointBatchData>
        add_candidates;


    /*
     * ==========================================================
     * 7. DELETE
     * ==========================================================
     *
     * 最大变化：
     *
     * V5：
     *
     * 只看当前Map。
     *
     * 所以Map0切换到Map1，
     * Map0所有Point都开始DELETE。
     *
     *
     * V5.1：
     *
     * current_ids包含整个Atlas。
     *
     * 只要Map0还Stored在Atlas，
     * Map0 Point就不会DELETE。
     */

    for (
        auto& entry :
        synced_map_points_)
    {
        const std::uint32_t id =
            entry.first;


        SyncedMapPointState& state =
            entry.second;


        if (
            current_ids.find(id)
            !=
            current_ids.end())
        {
            state.missing_count =
                0;


            continue;
        }


        /*
         * 防uint8溢出。
         */
        if (
            state.missing_count
            <
            std::numeric_limits<
                std::uint8_t>::max())
        {
            ++state.missing_count;
        }


        /*
         * 第一次没看见：
         *
         * 不删。
         *
         * 连续第二次没看见：
         *
         * 才成为DELETE candidate。
         */
        if (
            state.missing_count
            <
            kDeleteMissingBatchThreshold)
        {
            continue;
        }


        MapPointBatchData point;


        point.operation =
            MapPointBatchOperation::DELETE;


        point.id =
            id;


        point.previous_map_id =
            state.map_id;


        point.previous_x_cm =
            state.x_cm;


        point.previous_y_cm =
            state.y_cm;


        point.previous_z_cm =
            state.z_cm;


        delete_candidates.push_back(
            point);
    }


    /*
     * ==========================================================
     * 8. ADD / UPDATE
     * ==========================================================
     */

    for (
        const MapPointSnapshot& snapshot :
        snapshots)
    {
        const auto it =
            synced_map_points_.find(
                snapshot.id);


        /*
         * ======================================================
         * ADD
         * ======================================================
         */

        if (
            it
            ==
            synced_map_points_.end())
        {
            MapPointBatchData point;


            point.operation =
                MapPointBatchOperation::ADD;


            point.map_id =
                snapshot.map_id;


            point.id =
                snapshot.id;


            point.raw_x =
                snapshot.raw_x;


            point.raw_y =
                snapshot.raw_y;


            point.raw_z =
                snapshot.raw_z;


            point.x_cm =
                snapshot.x_cm;


            point.y_cm =
                snapshot.y_cm;


            point.z_cm =
                snapshot.z_cm;


            add_candidates.push_back(
                point);


            continue;
        }


        /*
         * ======================================================
         * UPDATE
         * ======================================================
         *
         * 当前整数厘米
         *
         * VS
         *
         * PC上次同步整数厘米
         */

        const std::int32_t dx_cm =
            static_cast<std::int32_t>(
                snapshot.x_cm)
            -
            static_cast<std::int32_t>(
                it->second.x_cm);


        const std::int32_t dy_cm =
            static_cast<std::int32_t>(
                snapshot.y_cm)
            -
            static_cast<std::int32_t>(
                it->second.y_cm);


        const std::int32_t dz_cm =
            static_cast<std::int32_t>(
                snapshot.z_cm)
            -
            static_cast<std::int32_t>(
                it->second.z_cm);


        /*
         * int64防止平方溢出。
         */
        const std::int64_t distance_squared_cm =
            static_cast<std::int64_t>(
                dx_cm)
                *
            static_cast<std::int64_t>(
                dx_cm)
            +
            static_cast<std::int64_t>(
                dy_cm)
                *
            static_cast<std::int64_t>(
                dy_cm)
            +
            static_cast<std::int64_t>(
                dz_cm)
                *
            static_cast<std::int64_t>(
                dz_cm);


        /*
         * ======================================================
         * V5.1：
         *
         * Map变化也必须UPDATE
         * ======================================================
         *
         * 例如：
         *
         * ID=500
         *
         * Merge前：
         *
         * Map1
         *
         * Merge后：
         *
         * Map0
         *
         * 即使XYZ变化不足2cm，
         * MapID也必须同步给PC。
         */

        const bool map_changed =
            snapshot.map_id
            !=
            it->second.map_id;


        /*
         * Map没变化
         *
         * 并且
         *
         * XYZ变化小于2cm
         *
         * 才真正不用UPDATE。
         */
        if (
            !map_changed
            &&
            distance_squared_cm
            <
            kMapPointUpdateThresholdSquaredCm)
        {
            continue;
        }


        MapPointBatchData point;


        point.operation =
            MapPointBatchOperation::UPDATE;


        /*
         * 当前Map。
         */
        point.map_id =
            snapshot.map_id;


        /*
         * 之前Map。
         */
        point.previous_map_id =
            it->second.map_id;


        point.map_changed =
            map_changed;


        point.id =
            snapshot.id;


        point.raw_x =
            snapshot.raw_x;


        point.raw_y =
            snapshot.raw_y;


        point.raw_z =
            snapshot.raw_z;


        point.x_cm =
            snapshot.x_cm;


        point.y_cm =
            snapshot.y_cm;


        point.z_cm =
            snapshot.z_cm;


        point.previous_x_cm =
            it->second.x_cm;


        point.previous_y_cm =
            it->second.y_cm;


        point.previous_z_cm =
            it->second.z_cm;


        point.delta_cm =
            std::sqrt(
                static_cast<float>(
                    distance_squared_cm));


        update_candidates.push_back(
            point);
    }


    /*
     * ==========================================================
     * 9. Nothing to send
     * ==========================================================
     */

    if (
        delete_candidates.empty()
        &&
        update_candidates.empty()
        &&
        add_candidates.empty())
    {
        return;
    }


    /*
     * ==========================================================
     * 10. Batch scheduler
     * ==========================================================
     */

    std::vector<MapPointBatchData>
        batch;


    batch.reserve(
        kMapPointsPerBatch);


    std::size_t delete_index =
        0;


    std::size_t update_index =
        0;


    std::size_t add_index =
        0;


    /*
     * ==========================================================
     * Preferred DELETE
     * ==========================================================
     */

    while (
        delete_index
            <
            delete_candidates.size()
        &&
        delete_index
            <
            kPreferredDeletesPerBatch
        &&
        batch.size()
            <
            kMapPointsPerBatch)
    {
        batch.push_back(
            delete_candidates[
                delete_index++]);


    }


    /*
     * ==========================================================
     * Preferred UPDATE
     * ==========================================================
     */

    while (
        update_index
            <
            update_candidates.size()
        &&
        update_index
            <
            kPreferredUpdatesPerBatch
        &&
        batch.size()
            <
            kMapPointsPerBatch)
    {
        batch.push_back(
            update_candidates[
                update_index++]);
    }


    /*
     * ==========================================================
     * ADD fill
     * ==========================================================
     */

    while (
        add_index
            <
            add_candidates.size()
        &&
        batch.size()
            <
            kMapPointsPerBatch)
    {
        batch.push_back(
            add_candidates[
                add_index++]);
    }


    /*
     * ==========================================================
     * Extra DELETE
     * ==========================================================
     */

    while (
        delete_index
            <
            delete_candidates.size()
        &&
        batch.size()
            <
            kMapPointsPerBatch)
    {
        batch.push_back(
            delete_candidates[
                delete_index++]);
    }


    /*
     * ==========================================================
     * Extra UPDATE
     * ==========================================================
     */

    while (
        update_index
            <
            update_candidates.size()
        &&
        batch.size()
            <
            kMapPointsPerBatch)
    {
        batch.push_back(
            update_candidates[
                update_index++]);
    }


    if (batch.empty())
    {
        return;
    }


    /*
     * ==========================================================
     * 11. Build V5.1 Binary Frame
     * ==========================================================
     */

    const std::uint32_t sequence =
        tx_sequence_++;


    std::size_t payload_bytes =
        0;


    std::uint16_t crc_value =
        0;


    const std::vector<std::uint8_t>
        binary_frame =
            BuildMapPointBinaryFrame(
                batch,
                sequence,
                active_map_id,
                payload_bytes,
                crc_value);


    /*
     * ==========================================================
     * 12. Batch statistics
     * ==========================================================
     */

    std::size_t batch_add_count =
        0;


    std::size_t batch_update_count =
        0;


    std::size_t batch_delete_count =
        0;


    std::size_t batch_map_move_count =
        0;


    for (
        const MapPointBatchData& point :
        batch)
    {
        if (
            point.operation
            ==
            MapPointBatchOperation::ADD)
        {
            ++batch_add_count;
        }


        else if (
            point.operation
            ==
            MapPointBatchOperation::UPDATE)
        {
            ++batch_update_count;


            if (point.map_changed)
            {
                ++batch_map_move_count;
            }
        }


        else
        {
            ++batch_delete_count;
        }
    }


    /*
     * ==========================================================
     * 13. Binary Frame Log
     * ==========================================================
     */

    RCLCPP_INFO(
        this->get_logger(),
        "[BinaryV5.1] "
        "session=0x%08X "
        "seq=%u "
        "active_map=%u "
        "count=%zu "
        "payload=%zuB "
        "frame=%zuB "
        "crc=0x%04X",
        static_cast<unsigned int>(
            session_id_),
        static_cast<unsigned int>(
            sequence),
        static_cast<unsigned int>(
            active_map_id),
        batch.size(),
        payload_bytes,
        binary_frame.size(),
        static_cast<unsigned int>(
            crc_value));


    const std::string hex_preview =
        MakeHexPreview(
            binary_frame);


    RCLCPP_INFO(
        this->get_logger(),
        "  HEX: %s",
        hex_preview.c_str());


    /*
     * ==========================================================
     * 14. Map Move Log
     * ==========================================================
     *
     * Merge时非常重要。
     *
     * 例如：
     *
     * MAP_MOVE
     * id=500
     * map=1->0
     */

    for (
        const MapPointBatchData& point :
        batch)
    {
        if (
            point.operation
                ==
                MapPointBatchOperation::UPDATE
            &&
            point.map_changed)
        {
            RCLCPP_INFO(
                this->get_logger(),
                "  MAP_MOVE "
                "id=%u "
                "map=%u->%u "
                "xyz=(%d,%d,%d)cm "
                "delta=%.2fcm",
                static_cast<unsigned int>(
                    point.id),
                static_cast<unsigned int>(
                    point.previous_map_id),
                static_cast<unsigned int>(
                    point.map_id),
                static_cast<int>(
                    point.x_cm),
                static_cast<int>(
                    point.y_cm),
                static_cast<int>(
                    point.z_cm),
                point.delta_cm);
        }
    }


    /*
     * ==========================================================
     * 15. 模拟发送成功
     * ==========================================================
     *
     * 当前仍然没有真正UART。
     *
     * 所以：
     *
     * binary_frame构造出来
     *
     * 就模拟PC已经成功收到。
     *
     *
     * V6真正UART之后：
     *
     * 这里必须改成：
     *
     * unsent
     * ↓
     * pending
     * ↓
     * UART send
     * ↓
     * ACK
     * ↓
     * commit synced cache
     */

    for (
        const MapPointBatchData& point :
        batch)
    {
        /*
         * ======================================================
         * ADD
         * ======================================================
         */

        if (
            point.operation
            ==
            MapPointBatchOperation::ADD)
        {
            SyncedMapPointState state;


            state.map_id =
                point.map_id;


            state.x_cm =
                point.x_cm;


            state.y_cm =
                point.y_cm;


            state.z_cm =
                point.z_cm;


            state.missing_count =
                0;


            synced_map_points_[
                point.id] =
                state;


            ++map_point_add_total_;
        }


        /*
         * ======================================================
         * UPDATE
         * ======================================================
         */

        else if (
            point.operation
            ==
            MapPointBatchOperation::UPDATE)
        {
            auto it =
                synced_map_points_.find(
                    point.id);


            if (
                it
                !=
                synced_map_points_.end())
            {
                /*
                 * Map ID也同步。
                 */
                it->second.map_id =
                    point.map_id;


                it->second.x_cm =
                    point.x_cm;


                it->second.y_cm =
                    point.y_cm;


                it->second.z_cm =
                    point.z_cm;


                it->second.missing_count =
                    0;
            }


            ++map_point_update_total_;
        }


        /*
         * ======================================================
         * DELETE
         * ======================================================
         */

        else
        {
            synced_map_points_.erase(
                point.id);


            ++map_point_delete_total_;
        }
    }


    /*
     * ==========================================================
     * 16. Final MapBatch Log
     * ==========================================================
     */

    RCLCPP_INFO(
        this->get_logger(),
        "[MapBatchV5.1] "
        "maps=%zu "
        "active_map=%u "
        "orb_valid=%zu "
        "comm_valid=%zu "
        "rejected=%zu "
        "add_pending=%zu "
        "update_pending=%zu "
        "delete_pending=%zu "
        "batch=%zu "
        "add=%zu "
        "update=%zu "
        "delete=%zu "
        "map_move=%zu "
        "synced_now=%zu "
        "payload=%zuB "
        "frame=%zuB "
        "add_total=%zu "
        "update_total=%zu "
        "delete_total=%zu",
        live_map_count,
        static_cast<unsigned int>(
            active_map_id),
        orb_valid_count,
        snapshots.size(),
        rejected_count,
        add_candidates.size(),
        update_candidates.size(),
        delete_candidates.size(),
        batch.size(),
        batch_add_count,
        batch_update_count,
        batch_delete_count,
        batch_map_move_count,
        synced_map_points_.size(),
        payload_bytes,
        binary_frame.size(),
        map_point_add_total_,
        map_point_update_total_,
        map_point_delete_total_);
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
        PrepareMapPointBatch();
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