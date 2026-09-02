#ifndef ORBSLAM3_ROS2_COMM_TYPES_HPP
#define ORBSLAM3_ROS2_COMM_TYPES_HPP

#include <cstddef>
#include <cstdint>
#include <vector>

namespace comm
{

enum class MapPointOperation : std::uint8_t
{
    ADD = 0x01,
    UPDATE = 0x02,
    DELETE = 0x03
};

struct MapPointBatchData
{
    MapPointOperation operation{MapPointOperation::ADD};
    std::uint16_t map_id{0};
    std::uint16_t previous_map_id{0};
    std::uint32_t id{0};
    float raw_x{0.0F};
    float raw_y{0.0F};
    float raw_z{0.0F};
    std::int16_t x_cm{0};
    std::int16_t y_cm{0};
    std::int16_t z_cm{0};
    std::int16_t previous_x_cm{0};
    std::int16_t previous_y_cm{0};
    std::int16_t previous_z_cm{0};
    float delta_cm{0.0F};
    bool map_changed{false};
};

struct SyncedPointState
{
    std::uint16_t map_id{0};
    std::int16_t x_cm{0};
    std::int16_t y_cm{0};
    std::int16_t z_cm{0};
    std::uint8_t missing_count{0};
};

struct CommPoint
{
    std::uint16_t map_id{0};
    std::uint32_t point_id{0};
    float x{0.0F};
    float y{0.0F};
    float z{0.0F};
    std::int16_t x_cm{0};
    std::int16_t y_cm{0};
    std::int16_t z_cm{0};
};

struct CommMapStats
{
    std::size_t live_map_count{0};
    std::size_t raw_point_count{0};
    std::size_t orb_valid_count{0};
    std::size_t candidate_count{0};
    std::size_t voxel_count{0};
    std::size_t dropped_count{0};
    std::size_t rejected_count{0};
    double keep_percent{0.0};
};

struct CommMapResult
{
    std::uint16_t active_map_id{0xFFFFU};
    std::vector<CommPoint> points;
    std::vector<MapPointBatchData> batch;
    CommMapStats stats;
    std::size_t add_pending{0};
    std::size_t update_pending{0};
    std::size_t delete_pending{0};
    std::size_t batch_add{0};
    std::size_t batch_update{0};
    std::size_t batch_delete{0};
    std::size_t batch_map_move{0};
    std::size_t synced_now{0};
    std::size_t add_total{0};
    std::size_t update_total{0};
    std::size_t delete_total{0};
    bool atlas_state_changed{false};
};

struct CommCommitResult
{
    std::size_t batch_add{0};
    std::size_t batch_update{0};
    std::size_t batch_delete{0};
    std::size_t synced_now{0};
    std::size_t add_total{0};
    std::size_t update_total{0};
    std::size_t delete_total{0};
};

struct TxFrame
{
    std::uint32_t sequence{0};
    std::uint16_t active_map_id{0xFFFFU};
    std::vector<std::uint8_t> bytes;
};

struct TxCompletion
{
    std::uint32_t sequence{0};
    bool success{false};
};

struct CommWorkerStats
{
    std::size_t queue_size{0};
    std::size_t enqueue_total{0};
    std::size_t processed_total{0};
    std::size_t queue_high_watermark{0};
    std::size_t enqueue_rejected{0};
    bool running{false};
};

}  // namespace comm

#endif
