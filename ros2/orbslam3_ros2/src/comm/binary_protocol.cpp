#include "comm/binary_protocol.hpp"

#include <algorithm>
#include <iomanip>
#include <sstream>
#include <stdexcept>

namespace
{

constexpr std::uint8_t kMagic1 = 0xAA;
constexpr std::uint8_t kMagic2 = 0x55;
constexpr std::uint8_t kVersion = 0x02;
constexpr std::uint8_t kMapPointBatchMessage = 0x10;

void AppendUint16LE(
    std::vector<std::uint8_t>& buffer,
    const std::uint16_t value)
{
    buffer.push_back(static_cast<std::uint8_t>(value & 0xFFU));
    buffer.push_back(static_cast<std::uint8_t>((value >> 8U) & 0xFFU));
}

void AppendInt16LE(
    std::vector<std::uint8_t>& buffer,
    const std::int16_t value)
{
    AppendUint16LE(buffer, static_cast<std::uint16_t>(value));
}

void AppendUint32LE(
    std::vector<std::uint8_t>& buffer,
    const std::uint32_t value)
{
    buffer.push_back(static_cast<std::uint8_t>(value & 0xFFU));
    buffer.push_back(static_cast<std::uint8_t>((value >> 8U) & 0xFFU));
    buffer.push_back(static_cast<std::uint8_t>((value >> 16U) & 0xFFU));
    buffer.push_back(static_cast<std::uint8_t>((value >> 24U) & 0xFFU));
}

}  // namespace

namespace comm
{

std::uint16_t BinaryProtocol::ComputeCrc16CcittFalse(
    const std::vector<std::uint8_t>& data)
{
    std::uint16_t crc = 0xFFFFU;
    for (const std::uint8_t byte : data)
    {
        crc ^= static_cast<std::uint16_t>(byte) << 8U;
        for (int bit = 0; bit < 8; ++bit)
        {
            crc = (crc & 0x8000U) != 0U
                ? static_cast<std::uint16_t>((crc << 1U) ^ 0x1021U)
                : static_cast<std::uint16_t>(crc << 1U);
        }
    }
    return crc;
}

std::string BinaryProtocol::MakeHexPreview(
    const std::vector<std::uint8_t>& data,
    const std::size_t max_bytes)
{
    std::ostringstream oss;
    oss << std::uppercase << std::hex << std::setfill('0');
    const std::size_t count = std::min(data.size(), max_bytes);
    for (std::size_t i = 0; i < count; ++i)
    {
        if (i != 0) oss << ' ';
        oss << std::setw(2) << static_cast<unsigned int>(data[i]);
    }
    if (data.size() > max_bytes) oss << " ...";
    return oss.str();
}

BinaryFrameResult BinaryProtocol::BuildMapPointFrame(
    const std::vector<MapPointBatchData>& batch,
    const std::uint32_t session_id,
    const std::uint32_t sequence,
    const std::uint16_t active_map_id) const
{
    if (batch.size() > kMaxBatchOperations)
    {
        throw std::invalid_argument("MapPoint batch exceeds 13 operations");
    }

    std::vector<std::uint8_t> payload;
    payload.reserve(kMaxBatchOperations * kAddUpdateBytes);
    for (const MapPointBatchData& point : batch)
    {
        payload.push_back(static_cast<std::uint8_t>(point.operation));
        if (point.operation == MapPointOperation::ADD
            || point.operation == MapPointOperation::UPDATE)
        {
            AppendUint16LE(payload, point.map_id);
            AppendUint32LE(payload, point.id);
            AppendInt16LE(payload, point.x_cm);
            AppendInt16LE(payload, point.y_cm);
            AppendInt16LE(payload, point.z_cm);
        }
        else if (point.operation == MapPointOperation::DELETE)
        {
            AppendUint32LE(payload, point.id);
        }
        else
        {
            throw std::invalid_argument("Unknown MapPoint operation");
        }
    }

    BinaryFrameResult result;
    result.payload_bytes = payload.size();
    result.frame.reserve(kHeaderBytes + payload.size() + kCrcBytes);
    result.frame.push_back(kMagic1);
    result.frame.push_back(kMagic2);
    result.frame.push_back(kVersion);
    result.frame.push_back(kMapPointBatchMessage);
    AppendUint32LE(result.frame, session_id);
    AppendUint32LE(result.frame, sequence);
    AppendUint16LE(result.frame, active_map_id);
    result.frame.push_back(static_cast<std::uint8_t>(batch.size()));
    AppendUint16LE(result.frame, static_cast<std::uint16_t>(payload.size()));
    result.frame.insert(result.frame.end(), payload.begin(), payload.end());
    result.crc = ComputeCrc16CcittFalse(result.frame);
    AppendUint16LE(result.frame, result.crc);
    return result;
}

}  // namespace comm
