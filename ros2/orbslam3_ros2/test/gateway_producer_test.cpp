#include "comm/map_producer.hpp"
#include <chrono>
#include <thread>
#include <stdexcept>

int main(int argc, char** argv)
{
    const std::size_t expected_synced = argc > 1 && std::string(argv[1]) == "--expect-transport-failure" ? 0 : 13;
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("map_producer_test");
    comm::CommMap map;
    comm::MapProducer producer(*node, map);
    const auto wait = [&](const std::function<bool()>& condition) {
        const auto end = std::chrono::steady_clock::now() + std::chrono::seconds(15);
        while (rclcpp::ok() && std::chrono::steady_clock::now() < end)
        {
            rclcpp::spin_some(node);
            if (condition()) return true;
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        return false;
    };
    try
    {
        orbslam3::msg::MapBatch identity;
        identity.source_id=1; identity.session_id=1234; identity.sequence=20; identity.message_type=16;
        orbslam3::msg::TxCompletion completion;
        completion.source_id=1; completion.session_id=1234; completion.sequence=20; completion.message_type=16;
        if (!comm::MapProducer::Matches(completion, identity)) throw std::runtime_error("match failed");
        completion.session_id++;
        if (comm::MapProducer::Matches(completion, identity)) throw std::runtime_error("session mismatch accepted");
        completion.session_id--; completion.sequence++;
        if (comm::MapProducer::Matches(completion, identity)) throw std::runtime_error("seq mismatch accepted");
        completion.sequence--; completion.message_type++;
        if (comm::MapProducer::Matches(completion, identity)) throw std::runtime_error("type mismatch accepted");
        completion.message_type--; completion.source_id++;
        if (comm::MapProducer::Matches(completion, identity)) throw std::runtime_error("source mismatch accepted");
        if (!wait([&] { return producer.ready(); })) throw std::runtime_error("gateway discovery timeout");
        // Allow reverse completion discovery as well.
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        comm::CommMapResult result;
        result.active_map_id=2;
        for (int i=1;i<=13;++i)
        {
            comm::MapPointBatchData p;
            p.id=i; p.map_id=2; p.x_cm=100+i; p.y_cm=-200-i; p.z_cm=300+i;
            result.batch.push_back(p);
        }
        if (!producer.Submit(result,1234,20) || map.synced_size()!=0
            || producer.Submit(result,1234,21)) throw std::runtime_error("pending semantics failed");
        if (!wait([&] { return !producer.pending(); }) || map.synced_size()!=expected_synced)
            throw std::runtime_error("success commit failed");
        result.batch.resize(1);
        result.batch[0].operation=static_cast<comm::MapPointOperation>(0x7f);
        if (!producer.Submit(result,1234,21)) throw std::runtime_error("failed submit");
        if (!wait([&] { return !producer.pending(); }) || map.synced_size()!=expected_synced)
            throw std::runtime_error("failure semantics failed");
        RCLCPP_INFO(node->get_logger(), "[IntegrationTest] PASS transport/failure/pending/identity synced=%zu", map.synced_size());
    }
    catch (const std::exception& error)
    {
        RCLCPP_ERROR(node->get_logger(), "%s", error.what());
        rclcpp::shutdown(); return 1;
    }
    rclcpp::shutdown(); return 0;
}
