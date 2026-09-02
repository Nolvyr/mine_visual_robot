#ifndef ORBSLAM3_COMM_MAP_PACKET_ENCODER_HPP
#define ORBSLAM3_COMM_MAP_PACKET_ENCODER_HPP

#include "comm/comm_types.hpp"
#include "comm/packet_encoder.hpp"

namespace comm
{
// Value-data boundary for a future ROS message adapter, not a ROS message or wire struct.
struct MapBatchMessage
{
    std::uint32_t session_id{0};
    std::uint32_t sequence{0};
    std::uint16_t active_map_id{0xFFFF};
    std::vector<MapPointBatchData> operations;
};

class MapPacketEncoder
{
public:
    static constexpr std::size_t kMetadataBytes = 7; // session(4), map(2), count(1)
    static constexpr std::size_t kAddUpdateBytes = 13;
    static constexpr std::size_t kDeleteBytes = 5;
    static constexpr std::size_t kCapacityByBudget =
        (PacketEncoder::kMaxPayloadBytes - kMetadataBytes) / kAddUpdateBytes;
    static constexpr std::size_t kMaxBatchOperations =
        kCapacityByBudget < 13 ? kCapacityByBudget : 13;

    std::vector<std::uint8_t> Encode(
        const MapBatchMessage& message,
        std::uint8_t source = node_id::Explorer,
        std::uint8_t destination = node_id::GroundStation,
        std::uint8_t ttl = 1) const;
};
static_assert(PacketEncoder::kHeaderBytes + MapPacketEncoder::kMetadataBytes
    + MapPacketEncoder::kMaxBatchOperations * MapPacketEncoder::kAddUpdateBytes
    + PacketEncoder::kCrcBytes <= PacketEncoder::kMaxRadioPacketSize,
    "Map batch exceeds packet budget");
} // namespace comm
#endif
