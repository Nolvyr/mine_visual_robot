#include "comm/map_producer.hpp"
#include "comm/packet_encoder.hpp"

namespace comm
{
MapProducer::MapProducer(rclcpp::Node& node, CommMap& map) : node_(node), map_(map)
{
    const auto qos = rclcpp::QoS(16).reliable().durability_volatile();
    publisher_ = node_.create_publisher<orbslam3::msg::MapBatch>("/comm/map_batch", qos);
    subscription_ = node_.create_subscription<orbslam3::msg::TxCompletion>(
        "/comm/tx_completion", qos,
        [this](orbslam3::msg::TxCompletion::ConstSharedPtr c) { Complete(*c); });
}
bool MapProducer::Matches(const orbslam3::msg::TxCompletion& c,
                          const orbslam3::msg::MapBatch& r)
{
    return c.source_id == r.source_id && c.session_id == r.session_id
        && c.sequence == r.sequence && c.message_type == r.message_type;
}
bool MapProducer::Submit(const CommMapResult& result, std::uint32_t session, std::uint32_t sequence)
{
    if (pending_ || result.batch.empty() || !ready()) return false;
    orbslam3::msg::MapBatch request;
    request.source_id = node_id::Explorer;
    request.session_id = session;
    request.sequence = sequence;
    request.message_type = static_cast<std::uint8_t>(MessageType::MAP_BATCH);
    request.active_map_id = result.active_map_id;
    for (const auto& p : result.batch)
    {
        orbslam3::msg::MapOperation op;
        op.operation = static_cast<std::uint8_t>(p.operation);
        op.map_id = p.map_id; op.id = p.id;
        op.x_cm = p.x_cm; op.y_cm = p.y_cm; op.z_cm = p.z_cm;
        request.operations.push_back(op);
    }
    // Save before publish; no callback may observe a missing pending identity.
    request_ = request;
    batch_ = result.batch;
    pending_ = true;
    try { publisher_->publish(request_); }
    catch (...) { pending_ = false; batch_.clear(); throw; }
    RCLCPP_INFO(node_.get_logger(),
        "[MapProducer] session=%u seq=%u operations=%zu state=PENDING",
        session, sequence, batch_.size());
    return true;
}
void MapProducer::Complete(const orbslam3::msg::TxCompletion& c)
{
    if (!pending_ || !Matches(c, request_))
    {
        RCLCPP_WARN(node_.get_logger(), "[CommCommit] unmatched completion session=%u seq=%u type=%u",
            c.session_id, c.sequence, c.message_type);
        return;
    }
    if (c.status == orbslam3::msg::TxCompletion::TRANSPORT_SENT)
    {
        const auto result = map_.CommitBatch(batch_);
        RCLCPP_INFO(node_.get_logger(), "[CommCommit] session=%u seq=%u result=SUCCESS synced_now=%zu",
            c.session_id, c.sequence, result.synced_now);
    }
    else if (c.status == orbslam3::msg::TxCompletion::FAILED)
        RCLCPP_WARN(node_.get_logger(), "[CommCommit] session=%u seq=%u result=FAILED commit=NO",
            c.session_id, c.sequence);
    else
    {
        RCLCPP_WARN(node_.get_logger(), "[CommCommit] unknown status=%u ignored", c.status);
        return;
    }
    pending_ = false;
    batch_.clear();
}
}
