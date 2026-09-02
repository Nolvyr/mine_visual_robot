#include "comm/comm_map.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "System.h"
#include "Map.h"
#include "MapPoint.h"

namespace
{

constexpr std::uint16_t kInvalidMapId = 0xFFFFU;
constexpr std::int64_t kUpdateThresholdSquaredCm = 4;
constexpr std::uint8_t kDeleteMissingThreshold = 2;
constexpr std::size_t kMaxBatch = 13;
constexpr std::size_t kPreferredDeletes = 6;
constexpr std::size_t kPreferredUpdates = 5;

bool PointIdToWire(const unsigned long id, std::uint32_t& output)
{
    if (id > static_cast<unsigned long>(std::numeric_limits<std::uint32_t>::max()))
        return false;
    output = static_cast<std::uint32_t>(id);
    return true;
}

bool MapIdToWire(const unsigned long id, std::uint16_t& output)
{
    if (id > 0xFFFEUL) return false;
    output = static_cast<std::uint16_t>(id);
    return true;
}

bool QuantizeCm(const float meters, std::int16_t& output)
{
    if (!std::isfinite(meters)) return false;
    const double value = std::round(static_cast<double>(meters) * 100.0);
    if (value < std::numeric_limits<std::int16_t>::min()
        || value > std::numeric_limits<std::int16_t>::max()) return false;
    output = static_cast<std::int16_t>(value);
    return true;
}

struct VoxelKey
{
    std::uint16_t map_id;
    std::int32_t x;
    std::int32_t y;
    std::int32_t z;
    bool operator==(const VoxelKey& other) const
    {
        return map_id == other.map_id && x == other.x
            && y == other.y && z == other.z;
    }
};

struct VoxelHash
{
    std::size_t operator()(const VoxelKey& key) const
    {
        std::size_t seed = std::hash<std::uint16_t>{}(key.map_id);
        const auto combine = [&seed](const std::size_t value) {
            seed ^= value + 0x9e3779b9U + (seed << 6U) + (seed >> 2U);
        };
        combine(std::hash<std::int32_t>{}(key.x));
        combine(std::hash<std::int32_t>{}(key.y));
        combine(std::hash<std::int32_t>{}(key.z));
        return seed;
    }
};

void FillBatch(
    std::vector<comm::MapPointBatchData>& batch,
    const std::vector<comm::MapPointBatchData>& source,
    std::size_t& index,
    const std::size_t limit)
{
    std::size_t added = 0;
    while (index < source.size() && batch.size() < kMaxBatch && added < limit)
    {
        batch.push_back(source[index++]);
        ++added;
    }
}

}  // namespace

namespace comm
{

CommMapResult CommMap::Update(ORB_SLAM3::System* slam)
{
    if (slam == nullptr) throw std::invalid_argument("slam must not be null");
    CommMapResult result;
    const auto maps = slam->GetAllMaps();
    ORB_SLAM3::Map* active = slam->GetCurrentMap();
    if (active != nullptr && !active->IsBad())
        MapIdToWire(active->GetId(), result.active_map_id);

    std::unordered_map<VoxelKey, CommPoint, VoxelHash> voxels;
    std::unordered_set<unsigned long> visited;
    for (ORB_SLAM3::Map* map : maps)
    {
        if (map == nullptr || map->IsBad()) continue;
        ++result.stats.live_map_count;
        const auto points = map->GetAllMapPoints();
        result.stats.raw_point_count += points.size();
        for (ORB_SLAM3::MapPoint* point : points)
        {
            if (point == nullptr || point->isBad() || !visited.insert(point->mnId).second)
                continue;
            const Eigen::Vector3f position = point->GetWorldPos();
            if (!std::isfinite(position.x()) || !std::isfinite(position.y())
                || !std::isfinite(position.z())) continue;
            ++result.stats.orb_valid_count;

            CommPoint snapshot;
            ORB_SLAM3::Map* owner = point->GetMap();
            if (!PointIdToWire(point->mnId, snapshot.point_id)
                || owner == nullptr || owner->IsBad()
                || !MapIdToWire(owner->GetId(), snapshot.map_id)
                || !QuantizeCm(position.x(), snapshot.x_cm)
                || !QuantizeCm(position.y(), snapshot.y_cm)
                || !QuantizeCm(position.z(), snapshot.z_cm))
            {
                ++result.stats.rejected_count;
                continue;
            }
            snapshot.x = position.x(); snapshot.y = position.y(); snapshot.z = position.z();
            const VoxelKey key{snapshot.map_id,
                static_cast<std::int32_t>(std::floor(snapshot.x / voxel_size_m_)),
                static_cast<std::int32_t>(std::floor(snapshot.y / voxel_size_m_)),
                static_cast<std::int32_t>(std::floor(snapshot.z / voxel_size_m_))};
            const auto it = voxels.find(key);
            if (it == voxels.end() || snapshot.point_id < it->second.point_id)
                voxels[key] = snapshot;
        }
    }

    std::vector<CommPoint> snapshots;
    snapshots.reserve(voxels.size());
    std::unordered_set<std::uint32_t> current_ids;
    for (const auto& entry : voxels)
    {
        snapshots.push_back(entry.second);
        current_ids.insert(entry.second.point_id);
        if (entry.second.map_id == result.active_map_id) result.points.push_back(entry.second);
    }
    const auto by_id = [](const CommPoint& a, const CommPoint& b) {
        return a.point_id < b.point_id;
    };
    std::sort(snapshots.begin(), snapshots.end(), by_id);
    std::sort(result.points.begin(), result.points.end(), by_id);

    result.stats.candidate_count = result.stats.orb_valid_count - result.stats.rejected_count;
    result.stats.voxel_count = snapshots.size();
    result.stats.dropped_count = result.stats.candidate_count - result.stats.voxel_count;
    result.stats.keep_percent = result.stats.candidate_count == 0 ? 0.0
        : 100.0 * static_cast<double>(result.stats.voxel_count)
            / static_cast<double>(result.stats.candidate_count);
    result.atlas_state_changed = !atlas_initialized_
        || result.stats.live_map_count != last_live_map_count_
        || result.active_map_id != last_active_map_id_;
    atlas_initialized_ = true;
    last_live_map_count_ = result.stats.live_map_count;
    last_active_map_id_ = result.active_map_id;

    std::vector<MapPointBatchData> deletes, updates, adds;
    for (auto& entry : synced_points_)
    {
        if (current_ids.count(entry.first) != 0U)
        {
            entry.second.missing_count = 0;
            continue;
        }
        if (entry.second.missing_count < std::numeric_limits<std::uint8_t>::max())
            ++entry.second.missing_count;
        if (entry.second.missing_count < kDeleteMissingThreshold) continue;
        MapPointBatchData operation;
        operation.operation = MapPointOperation::DELETE;
        operation.id = entry.first;
        operation.previous_map_id = entry.second.map_id;
        operation.previous_x_cm = entry.second.x_cm;
        operation.previous_y_cm = entry.second.y_cm;
        operation.previous_z_cm = entry.second.z_cm;
        deletes.push_back(operation);
    }

    for (const CommPoint& snapshot : snapshots)
    {
        const auto synced = synced_points_.find(snapshot.point_id);
        MapPointBatchData operation;
        operation.map_id = snapshot.map_id; operation.id = snapshot.point_id;
        operation.raw_x = snapshot.x; operation.raw_y = snapshot.y; operation.raw_z = snapshot.z;
        operation.x_cm = snapshot.x_cm; operation.y_cm = snapshot.y_cm; operation.z_cm = snapshot.z_cm;
        if (synced == synced_points_.end())
        {
            operation.operation = MapPointOperation::ADD;
            adds.push_back(operation);
            continue;
        }
        const std::int32_t dx = snapshot.x_cm - synced->second.x_cm;
        const std::int32_t dy = snapshot.y_cm - synced->second.y_cm;
        const std::int32_t dz = snapshot.z_cm - synced->second.z_cm;
        const std::int64_t distance = static_cast<std::int64_t>(dx) * dx
            + static_cast<std::int64_t>(dy) * dy + static_cast<std::int64_t>(dz) * dz;
        const bool map_changed = snapshot.map_id != synced->second.map_id;
        if (!map_changed && distance < kUpdateThresholdSquaredCm) continue;
        operation.operation = MapPointOperation::UPDATE;
        operation.previous_map_id = synced->second.map_id;
        operation.previous_x_cm = synced->second.x_cm;
        operation.previous_y_cm = synced->second.y_cm;
        operation.previous_z_cm = synced->second.z_cm;
        operation.map_changed = map_changed;
        operation.delta_cm = std::sqrt(static_cast<float>(distance));
        updates.push_back(operation);
    }

    result.add_pending = adds.size(); result.update_pending = updates.size();
    result.delete_pending = deletes.size();
    std::size_t di = 0, ui = 0, ai = 0;
    FillBatch(result.batch, deletes, di, kPreferredDeletes);
    FillBatch(result.batch, updates, ui, kPreferredUpdates);
    FillBatch(result.batch, adds, ai, kMaxBatch);
    FillBatch(result.batch, deletes, di, kMaxBatch);
    FillBatch(result.batch, updates, ui, kMaxBatch);

    for (const MapPointBatchData& operation : result.batch)
    {
        if (operation.operation == MapPointOperation::ADD)
        {
            ++result.batch_add;
        }
        else if (operation.operation == MapPointOperation::UPDATE)
        {
            ++result.batch_update;
            if (operation.map_changed) ++result.batch_map_move;
        }
        else if (operation.operation == MapPointOperation::DELETE)
        {
            ++result.batch_delete;
        }
    }
    result.synced_now = synced_points_.size();
    result.add_total = add_total_; result.update_total = update_total_;
    result.delete_total = delete_total_;
    return result;
}

}  // namespace comm
