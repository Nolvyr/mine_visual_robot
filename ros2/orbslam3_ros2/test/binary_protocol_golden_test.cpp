#include "comm/binary_protocol.hpp"

#include <array>
#include <cstdint>
#include <iostream>
#include <stdexcept>
#include <vector>

int main()
{
    std::vector<comm::MapPointBatchData> batch;
    for (std::uint32_t i = 1; i <= 13; ++i)
    {
        comm::MapPointBatchData point;
        point.operation = comm::MapPointOperation::ADD;
        point.map_id = 2;
        point.id = i;
        point.x_cm = static_cast<std::int16_t>(100 + i);
        point.y_cm = static_cast<std::int16_t>(-200 - static_cast<int>(i));
        point.z_cm = static_cast<std::int16_t>(300 + i);
        batch.push_back(point);
    }

    const auto result = comm::BinaryProtocol{}.BuildMapPointFrame(
        batch, 0x12345678U, 0x01020304U, 2U);

    const std::array<std::uint8_t, 64> expected_prefix{{
        0xAA,0x55,0x02,0x10,0x78,0x56,0x34,0x12,
        0x04,0x03,0x02,0x01,0x02,0x00,0x0D,0xA9,0x00,
        0x01,0x02,0x00,0x01,0x00,0x00,0x00,0x65,0x00,0x37,0xFF,0x2D,0x01,
        0x01,0x02,0x00,0x02,0x00,0x00,0x00,0x66,0x00,0x36,0xFF,0x2E,0x01,
        0x01,0x02,0x00,0x03,0x00,0x00,0x00,0x67,0x00,0x35,0xFF,0x2F,0x01,
        0x01,0x02,0x00,0x04,0x00,0x00,0x00,0x68}};

    bool prefix_matches = result.frame.size() >= expected_prefix.size();
    for (std::size_t i = 0; prefix_matches && i < expected_prefix.size(); ++i)
        prefix_matches = result.frame[i] == expected_prefix[i];

    bool over_limit_threw = false;
    try
    {
        batch.push_back(batch.back());
        comm::BinaryProtocol{}.BuildMapPointFrame(batch, 0, 0, 0);
    }
    catch (const std::invalid_argument&) { over_limit_threw = true; }

    bool invalid_operation_threw = false;
    try
    {
        batch.resize(1);
        batch[0].operation = static_cast<comm::MapPointOperation>(0x7F);
        comm::BinaryProtocol{}.BuildMapPointFrame(batch, 0, 0, 0);
    }
    catch (const std::invalid_argument&) { invalid_operation_threw = true; }

    const bool passed = result.payload_bytes == 169
        && result.frame.size() == 188
        && result.crc == 0x7E05U
        && result.frame[result.frame.size() - 2] == 0x05U
        && result.frame.back() == 0x7EU
        && prefix_matches
        && over_limit_threw
        && invalid_operation_threw;

    std::cout << "payload=" << result.payload_bytes
              << " frame=" << result.frame.size()
              << " crc=0x" << std::hex << result.crc
              << " prefix=" << prefix_matches
              << " over13_throw=" << over_limit_threw
              << " invalid_throw=" << invalid_operation_threw << '\n';
    return passed ? 0 : 1;
}
