#include "comm/packet_encoder.hpp"

#include <stdexcept>

namespace comm
{
// C++14 requires definitions when constexpr members are passed by reference.
constexpr std::uint8_t PacketEncoder::kVersion;
constexpr std::size_t PacketEncoder::kMaxRadioPacketSize;
constexpr std::size_t PacketEncoder::kHeaderBytes;
constexpr std::size_t PacketEncoder::kCrcBytes;
constexpr std::size_t PacketEncoder::kMaxPayloadBytes;

Priority DefaultPriority(const MessageType type)
{
    switch (type)
    {
    case MessageType::CMD_STOP: return Priority::Emergency;
    case MessageType::MOTION_CMD:
    case MessageType::CMD_RETURN_HOME:
    case MessageType::CMD_EXPLORE:
    case MessageType::ACK:
    case MessageType::NACK: return Priority::High;
    case MessageType::MAP_BATCH:
    case MessageType::MAP_ADD:
    case MessageType::MAP_UPDATE:
    case MessageType::MAP_DELETE:
    case MessageType::MAP_RESET: return Priority::Low;
    default: return Priority::Normal;
    }
}

std::vector<std::uint8_t> PacketEncoder::Encode(
    const PacketHeader& header, const std::vector<std::uint8_t>& payload) const
{
    if (payload.size() > kMaxPayloadBytes)
        throw std::invalid_argument("Packet exceeds 200-byte project budget");
    if (header.source == node_id::Broadcast)
        throw std::invalid_argument("Broadcast cannot be a source address");
    // TTL=0 is permitted for reception/local delivery; forwarding policy belongs to gateway.
    std::vector<std::uint8_t> bytes;
    bytes.reserve(kHeaderBytes + payload.size() + kCrcBytes);
    bytes.push_back(0xAA);
    bytes.push_back(0x55);
    bytes.push_back(kVersion);
    bytes.push_back(static_cast<std::uint8_t>(header.message_type));
    bytes.push_back(header.source);
    bytes.push_back(header.destination);
    for (unsigned shift = 0; shift < 32; shift += 8)
        bytes.push_back(static_cast<std::uint8_t>(header.sequence >> shift));
    bytes.push_back(header.ttl);
    bytes.push_back(static_cast<std::uint8_t>(payload.size()));
    bytes.push_back(static_cast<std::uint8_t>(payload.size() >> 8));
    bytes.insert(bytes.end(), payload.begin(), payload.end());

    // CRC-16/CCITT-FALSE, including magic/header and complete payload.
    std::uint16_t crc = 0xFFFF;
    for (const auto byte : bytes)
    {
        crc ^= static_cast<std::uint16_t>(byte) << 8;
        for (unsigned bit = 0; bit < 8; ++bit)
            crc = static_cast<std::uint16_t>((crc & 0x8000)
                ? (crc << 1) ^ 0x1021 : crc << 1);
    }
    bytes.push_back(static_cast<std::uint8_t>(crc));
    bytes.push_back(static_cast<std::uint8_t>(crc >> 8));
    return bytes;
}
} // namespace comm
