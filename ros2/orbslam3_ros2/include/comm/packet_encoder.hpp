#ifndef ORBSLAM3_COMM_PACKET_ENCODER_HPP
#define ORBSLAM3_COMM_PACKET_ENCODER_HPP

#include <cstddef>
#include <cstdint>
#include <vector>

namespace comm
{
// V3 routing protocol reservations, not implemented application handlers.
enum class MessageType : std::uint8_t
{
    HEARTBEAT = 0x01, POSE = 0x02, SLAM_STATUS = 0x03, VEHICLE_STATUS = 0x04,
    MAP_BATCH = 0x10, MAP_ADD = 0x11, MAP_UPDATE = 0x12,
    MAP_DELETE = 0x13, MAP_RESET = 0x14,
    MOTION_CMD = 0x20, CMD_STOP = 0x21, CMD_RETURN_HOME = 0x22, CMD_EXPLORE = 0x23,
    ACK = 0x30, NACK = 0x31, RELAY_STATUS = 0x40, LINK_STATUS = 0x41
};

enum class Priority { Emergency, High, Normal, Low };
Priority DefaultPriority(MessageType type);

namespace node_id
{
constexpr std::uint8_t Explorer = 0x01;
constexpr std::uint8_t RelayFirst = 0x10;
constexpr std::uint8_t RelayLast = 0x1F;
constexpr std::uint8_t GroundStation = 0xF0;
constexpr std::uint8_t Broadcast = 0xFF;
}

// Semantic fields only: never memcpy this struct onto the wire.
struct PacketHeader
{
    MessageType message_type{MessageType::MAP_BATCH};
    std::uint8_t source{node_id::Explorer};
    std::uint8_t destination{node_id::GroundStation};
    std::uint32_t sequence{0};
    std::uint8_t ttl{1};
};

class PacketEncoder
{
public:
    static constexpr std::uint8_t kVersion = 0x03;
    static constexpr std::size_t kMaxRadioPacketSize = 200;
    static constexpr std::size_t kHeaderBytes = 13;
    static constexpr std::size_t kCrcBytes = 2;
    static constexpr std::size_t kMaxPayloadBytes =
        kMaxRadioPacketSize - kHeaderBytes - kCrcBytes;

    std::vector<std::uint8_t> Encode(
        const PacketHeader& header,
        const std::vector<std::uint8_t>& payload) const;
};
} // namespace comm
#endif
