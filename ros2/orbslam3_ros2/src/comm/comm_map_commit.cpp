#include "comm/comm_map.hpp"

#include <cmath>
#include <stdexcept>

namespace comm
{

CommMap::CommMap(const float voxel_size_m) : voxel_size_m_(voxel_size_m)
{
    if (!(voxel_size_m_ > 0.0F) || !std::isfinite(voxel_size_m_))
    {
        throw std::invalid_argument("voxel size must be finite and positive");
    }
}

CommCommitResult CommMap::CommitBatch(
    const std::vector<MapPointBatchData>& batch)
{
    for (const MapPointBatchData& operation : batch)
    {
        if (operation.operation != MapPointOperation::ADD
            && operation.operation != MapPointOperation::UPDATE
            && operation.operation != MapPointOperation::DELETE)
        {
            throw std::invalid_argument("invalid MapPoint operation in commit batch");
        }
    }

    CommCommitResult result;
    for (const MapPointBatchData& operation : batch)
    {
        if (operation.operation == MapPointOperation::ADD)
        {
            synced_points_[operation.id] = SyncedPointState{operation.map_id,
                operation.x_cm, operation.y_cm, operation.z_cm, 0};
            ++result.batch_add;
            ++add_total_;
        }
        else if (operation.operation == MapPointOperation::UPDATE)
        {
            synced_points_[operation.id] = SyncedPointState{operation.map_id,
                operation.x_cm, operation.y_cm, operation.z_cm, 0};
            ++result.batch_update;
            ++update_total_;
        }
        else
        {
            synced_points_.erase(operation.id);
            ++result.batch_delete;
            ++delete_total_;
        }
    }

    result.synced_now = synced_points_.size();
    result.add_total = add_total_;
    result.update_total = update_total_;
    result.delete_total = delete_total_;
    return result;
}

bool CommMap::GetSyncedPoint(
    const std::uint32_t point_id,
    SyncedPointState& state) const
{
    const auto found = synced_points_.find(point_id);
    if (found == synced_points_.end())
    {
        return false;
    }
    state = found->second;
    return true;
}

}  // namespace comm
