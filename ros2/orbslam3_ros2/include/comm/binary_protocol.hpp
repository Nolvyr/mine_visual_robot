#ifndef ORBSLAM3_ROS2_BINARY_PROTOCOL_HPP
#define ORBSLAM3_ROS2_BINARY_PROTOCOL_HPP

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "comm/comm_types.hpp"

namespace comm
{

struct BinaryFrameResult
{
    std::vector<std::uint8_t> frame;
    std::size_t payload_bytes{0};
    std::uint16_t crc{0};
};

class BinaryProtocol
{
public:
    static constexpr std::size_t kMaxBatchOperations = 13;
    static constexpr std::size_t kHeaderBytes = 17;
    static constexpr std::size_t kAddUpdateBytes = 13;
    static constexpr std::size_t kDeleteBytes = 5;
    static constexpr std::size_t kCrcBytes = 2;

    BinaryFrameResult BuildMapPointFrame(
        const std::vector<MapPointBatchData>& batch,
        std::uint32_t session_id,
        std::uint32_t sequence,
        std::uint16_t active_map_id) const;

    static std::uint16_t ComputeCrc16CcittFalse(
        const std::vector<std::uint8_t>& data);

    static std::string MakeHexPreview(
        const std::vector<std::uint8_t>& data,
        std::size_t max_bytes = 64);
};

}  // namespace comm

#endif
