#include "comm/map_packet_encoder.hpp"

#include <algorithm>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>

namespace
{
void Check(bool ok, const char* message)
{
    if (!ok) throw std::runtime_error(message);
}
template<class F> void Reject(F function)
{
    try { function(); }
    catch (const std::invalid_argument&) { return; }
    throw std::runtime_error("Expected invalid_argument");
}
std::vector<std::uint8_t> Hex(const char* input)
{
    std::istringstream stream(input);
    unsigned byte;
    std::vector<std::uint8_t> result;
    while (stream >> std::hex >> byte) result.push_back(static_cast<std::uint8_t>(byte));
    return result;
}
}

int main()
{
    try
    {
        comm::MapBatchMessage message;
        message.session_id = 0x12345678;
        message.sequence = 0x01020304;
        message.active_map_id = 2;
        for (int i = 1; i <= 13; ++i)
        {
            comm::MapPointBatchData point;
            point.id = i; point.map_id = 2;
            point.x_cm = 100 + i; point.y_cm = -200 - i; point.z_cm = 300 + i;
            message.operations.push_back(point);
        }
        const comm::MapPacketEncoder encoder;
        const auto bytes = encoder.Encode(message, 1, 0xF0, 3);
        // Independently generated using Python struct + binascii.crc_hqx(init=0xffff).
        const auto golden = Hex(
            "AA 55 03 10 01 F0 04 03 02 01 03 B0 00 78 56 34 12 02 00 0D "
            "01 02 00 01 00 00 00 65 00 37 FF 2D 01 "
            "01 02 00 02 00 00 00 66 00 36 FF 2E 01 "
            "01 02 00 03 00 00 00 67 00 35 FF 2F 01 "
            "01 02 00 04 00 00 00 68 00 34 FF 30 01 "
            "01 02 00 05 00 00 00 69 00 33 FF 31 01 "
            "01 02 00 06 00 00 00 6A 00 32 FF 32 01 "
            "01 02 00 07 00 00 00 6B 00 31 FF 33 01 "
            "01 02 00 08 00 00 00 6C 00 30 FF 34 01 "
            "01 02 00 09 00 00 00 6D 00 2F FF 35 01 "
            "01 02 00 0A 00 00 00 6E 00 2E FF 36 01 "
            "01 02 00 0B 00 00 00 6F 00 2D FF 37 01 "
            "01 02 00 0C 00 00 00 70 00 2C FF 38 01 "
            "01 02 00 0D 00 00 00 71 00 2B FF 39 01 59 46");
        Check(bytes == golden && bytes.size() == 191, "Full V3 golden mismatch");
        Check(comm::MapPacketEncoder::kCapacityByBudget == 13, "Capacity mismatch");
        message.operations.push_back(message.operations.back());
        Reject([&] { encoder.Encode(message); });
        message.operations.resize(3);
        message.operations[1].operation = comm::MapPointOperation::UPDATE;
        message.operations[1].map_id = 0x1234;
        message.operations[1].x_cm = -32768;
        message.operations[1].y_cm = 32767;
        message.operations[2].operation = comm::MapPointOperation::DELETE;
        const auto mixed = encoder.Encode(message);
        const auto expected_records = Hex(
            "01 02 00 01 00 00 00 65 00 37 FF 2D 01 "
            "02 34 12 02 00 00 00 00 80 FF 7F 2E 01 "
            "03 03 00 00 00");
        Check(mixed.size() == 53 && mixed[11] == 38 && mixed[19] == 3,
            "Mixed packet lengths/count mismatch");
        Check(std::equal(expected_records.begin(), expected_records.end(), mixed.begin()+20),
            "Mixed operations/signed values mismatch");
        message.operations.resize(1);
        Check(encoder.Encode(message).size() == 35, "One ADD size mismatch");
        message.operations[0].operation = comm::MapPointOperation::DELETE;
        Check(encoder.Encode(message).size() == 27, "One DELETE size mismatch");
        message.operations[0].operation = static_cast<comm::MapPointOperation>(0x7F);
        Reject([&] { encoder.Encode(message); });
        message.operations.clear();
        Reject([&] { encoder.Encode(message); });

        comm::PacketHeader header;
        const comm::PacketEncoder packet;
        Check(packet.Encode(header, {}).size() == 15, "Empty generic payload mismatch");
        Check(packet.Encode(header, std::vector<std::uint8_t>(185)).size() == 200,
            "200B boundary mismatch");
        Reject([&] { packet.Encode(header, std::vector<std::uint8_t>(186)); });
        header.source = comm::node_id::RelayFirst;
        header.destination = comm::node_id::Broadcast;
        header.ttl = 0;
        const auto routed = packet.Encode(header, {});
        Check(routed[4] == 0x10 && routed[5] == 0xFF && routed[10] == 0,
            "Routing fields mismatch");
        header.source = comm::node_id::Broadcast;
        Reject([&] { packet.Encode(header, {}); });
        Check(comm::DefaultPriority(comm::MessageType::CMD_STOP) == comm::Priority::Emergency
            && comm::DefaultPriority(comm::MessageType::ACK) == comm::Priority::High
            && comm::DefaultPriority(comm::MessageType::POSE) == comm::Priority::Normal
            && comm::DefaultPriority(comm::MessageType::MAP_BATCH) == comm::Priority::Low,
            "Priority reservations mismatch");
        std::cout << "V3 golden=191B CRC=0x4659 capacity=13 mixed=PASS bounds=PASS\n";
        return 0;
    }
    catch (const std::exception& error)
    {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
