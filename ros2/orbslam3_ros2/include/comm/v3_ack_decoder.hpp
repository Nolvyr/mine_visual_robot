#ifndef ORBSLAM3_COMM_V3_ACK_DECODER_HPP
#define ORBSLAM3_COMM_V3_ACK_DECODER_HPP

#include <cstddef>
#include <cstdint>
#include <vector>

namespace comm
{
struct V3Ack
{
    std::uint8_t source{0};
    std::uint8_t destination{0};
    std::uint32_t sequence{0};
    std::uint32_t session_id{0};
    std::uint8_t acknowledged_message_type{0};
    std::uint8_t status{0};
};
struct V3AckDecodeResult
{
    std::vector<V3Ack> acknowledgements;
    std::size_t discarded_bytes{0};
    std::size_t header_errors{0};
    std::size_t crc_errors{0};
    std::size_t non_ack_frames{0};
};
class V3AckDecoder
{
public:
    static constexpr std::uint8_t kVersion = 0x03;
    static constexpr std::uint8_t kAckMessageType = 0x30;
    static constexpr std::uint8_t kReceivedStatus = 0x00;
    static constexpr std::size_t kHeaderBytes = 13;
    static constexpr std::size_t kCrcBytes = 2;
    static constexpr std::size_t kAckPayloadBytes = 6;
    static constexpr std::size_t kAckFrameBytes = 21;
    static constexpr std::size_t kMaxPayloadBytes = 185;
    V3AckDecodeResult Feed(const std::uint8_t* data, std::size_t size);
    V3AckDecodeResult Feed(const std::vector<std::uint8_t>& data)
    {
        return Feed(data.data(), data.size());
    }
    void Reset() { buffer_.clear(); }
private:
    std::vector<std::uint8_t> buffer_;
};
} // namespace comm
#endif
