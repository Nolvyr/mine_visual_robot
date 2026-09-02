#include "comm/map_packet_encoder.hpp"

#include <stdexcept>

namespace
{
void AppendLE(std::vector<std::uint8_t>& out, std::uint32_t value, unsigned width)
{
    for (unsigned i = 0; i < width; ++i)
        out.push_back(static_cast<std::uint8_t>(value >> (8 * i)));
}
}
namespace comm
{
constexpr std::size_t MapPacketEncoder::kMetadataBytes;
constexpr std::size_t MapPacketEncoder::kAddUpdateBytes;
constexpr std::size_t MapPacketEncoder::kDeleteBytes;
constexpr std::size_t MapPacketEncoder::kCapacityByBudget;
constexpr std::size_t MapPacketEncoder::kMaxBatchOperations;

std::vector<std::uint8_t> MapPacketEncoder::Encode(
    const MapBatchMessage& message, const std::uint8_t source,
    const std::uint8_t destination, const std::uint8_t ttl) const
{
    if (message.operations.empty() || message.operations.size() > kMaxBatchOperations)
        throw std::invalid_argument("Map batch must contain 1..budget-limited operations");

    std::vector<std::uint8_t> payload;
    AppendLE(payload, message.session_id, 4);
    AppendLE(payload, message.active_map_id, 2);
    payload.push_back(static_cast<std::uint8_t>(message.operations.size()));
    for (const auto& point : message.operations)
    {
        payload.push_back(static_cast<std::uint8_t>(point.operation));
        switch (point.operation)
        {
        case MapPointOperation::ADD:
        case MapPointOperation::UPDATE:
            AppendLE(payload, point.map_id, 2);
            AppendLE(payload, point.id, 4);
            AppendLE(payload, static_cast<std::uint16_t>(point.x_cm), 2);
            AppendLE(payload, static_cast<std::uint16_t>(point.y_cm), 2);
            AppendLE(payload, static_cast<std::uint16_t>(point.z_cm), 2);
            break;
        case MapPointOperation::DELETE:
            AppendLE(payload, point.id, 4);
            break;
        default: throw std::invalid_argument("Invalid map operation");
        }
    }
    PacketHeader header;
    header.message_type = MessageType::MAP_BATCH;
    header.source = source;
    header.destination = destination;
    header.sequence = message.sequence;
    header.ttl = ttl;
    return PacketEncoder{}.Encode(header, payload);
}
} // namespace comm
