#include "comm/v3_ack_decoder.hpp"
namespace
{
std::uint16_t ReadU16(const std::uint8_t* data)
{
    return static_cast<std::uint16_t>(data[0]) | static_cast<std::uint16_t>(data[1]) << 8;
}
std::uint32_t ReadU32(const std::uint8_t* data)
{
    return static_cast<std::uint32_t>(data[0]) | static_cast<std::uint32_t>(data[1]) << 8
        | static_cast<std::uint32_t>(data[2]) << 16 | static_cast<std::uint32_t>(data[3]) << 24;
}
std::uint16_t Crc16CcittFalse(const std::uint8_t* data, std::size_t size)
{
    std::uint16_t crc = 0xFFFF;
    for (std::size_t index = 0; index < size; ++index)
    {
        crc ^= static_cast<std::uint16_t>(data[index]) << 8;
        for (unsigned bit = 0; bit < 8; ++bit)
            crc = static_cast<std::uint16_t>((crc & 0x8000) ? (crc << 1) ^ 0x1021 : crc << 1);
    }
    return crc;
}
}
namespace comm
{
constexpr std::uint8_t V3AckDecoder::kVersion;
constexpr std::uint8_t V3AckDecoder::kAckMessageType;
constexpr std::uint8_t V3AckDecoder::kReceivedStatus;
constexpr std::size_t V3AckDecoder::kHeaderBytes;
constexpr std::size_t V3AckDecoder::kCrcBytes;
constexpr std::size_t V3AckDecoder::kAckPayloadBytes;
constexpr std::size_t V3AckDecoder::kAckFrameBytes;
constexpr std::size_t V3AckDecoder::kMaxPayloadBytes;
V3AckDecodeResult V3AckDecoder::Feed(const std::uint8_t* data, const std::size_t size)
{
    V3AckDecodeResult result;
    if (data != nullptr && size != 0) buffer_.insert(buffer_.end(), data, data + size);
    for (;;)
    {
        std::size_t magic_index = 0;
        while (magic_index + 1 < buffer_.size()
            && (buffer_[magic_index] != 0xAA || buffer_[magic_index + 1] != 0x55)) ++magic_index;
        if (magic_index != 0)
        {
            result.discarded_bytes += magic_index;
            buffer_.erase(buffer_.begin(), buffer_.begin() + magic_index);
        }
        if (buffer_.size() == 1 && buffer_[0] != 0xAA)
        {
            ++result.discarded_bytes; buffer_.clear();
        }
        if (buffer_.size() < kHeaderBytes) break;
        if (buffer_[2] != kVersion)
        {
            ++result.header_errors; buffer_.erase(buffer_.begin()); continue;
        }
        const std::size_t payload_size = ReadU16(buffer_.data() + 11);
        if (payload_size > kMaxPayloadBytes)
        {
            ++result.header_errors; buffer_.erase(buffer_.begin()); continue;
        }
        const std::size_t frame_size = kHeaderBytes + payload_size + kCrcBytes;
        if (buffer_.size() < frame_size) break;
        const std::uint16_t received_crc = ReadU16(buffer_.data() + frame_size - kCrcBytes);
        if (Crc16CcittFalse(buffer_.data(), frame_size - kCrcBytes) != received_crc)
        {
            ++result.crc_errors; buffer_.erase(buffer_.begin()); continue;
        }
        if (buffer_[3] != kAckMessageType) ++result.non_ack_frames;
        else if (payload_size != kAckPayloadBytes || frame_size != kAckFrameBytes) ++result.header_errors;
        else
        {
            V3Ack ack;
            ack.source=buffer_[4]; ack.destination=buffer_[5]; ack.sequence=ReadU32(buffer_.data()+6);
            ack.session_id=ReadU32(buffer_.data()+13); ack.acknowledged_message_type=buffer_[17];
            ack.status=buffer_[18]; result.acknowledgements.push_back(ack);
        }
        buffer_.erase(buffer_.begin(), buffer_.begin() + frame_size);
    }
    return result;
}
}
