#ifndef ORBSLAM3_ROS2_COMM_MAP_HPP
#define ORBSLAM3_ROS2_COMM_MAP_HPP

#include <cstdint>
#include <unordered_map>

#include "comm/comm_types.hpp"

namespace ORB_SLAM3 { class System; }

namespace comm
{

class CommMap
{
public:
    explicit CommMap(float voxel_size_m = 0.15F);
    CommMapResult Update(ORB_SLAM3::System* slam);
    CommCommitResult CommitBatch(
        const std::vector<MapPointBatchData>& batch);

    bool GetSyncedPoint(
        std::uint32_t point_id,
        SyncedPointState& state) const;
    std::size_t synced_size() const { return synced_points_.size(); }

    float voxel_size_m() const { return voxel_size_m_; }

private:
    float voxel_size_m_{0.15F};
    std::unordered_map<std::uint32_t, SyncedPointState> synced_points_;
    std::size_t add_total_{0};
    std::size_t update_total_{0};
    std::size_t delete_total_{0};
    bool atlas_initialized_{false};
    std::size_t last_live_map_count_{0};
    std::uint16_t last_active_map_id_{0xFFFFU};
};

}  // namespace comm

#endif
