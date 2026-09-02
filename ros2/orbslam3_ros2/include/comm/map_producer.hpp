#ifndef COMM_MAP_PRODUCER_HPP
#define COMM_MAP_PRODUCER_HPP
#include "rclcpp/rclcpp.hpp"
#include "orbslam3/msg/map_batch.hpp"
#include "orbslam3/msg/tx_completion.hpp"
#include "comm/comm_map.hpp"

namespace comm
{
// Access only from the node's default mutually-exclusive callback group.
class MapProducer
{
public:
    MapProducer(rclcpp::Node& node, CommMap& map);
    bool Submit(const CommMapResult& result, std::uint32_t session, std::uint32_t sequence);
    bool pending() const { return pending_; }
    bool ready() const
    {
        return publisher_->get_subscription_count() != 0
            && subscription_->get_publisher_count() != 0;
    }
    static bool Matches(const orbslam3::msg::TxCompletion& completion,
                        const orbslam3::msg::MapBatch& request);
private:
    void Complete(const orbslam3::msg::TxCompletion& completion);
    rclcpp::Node& node_;
    CommMap& map_;
    bool pending_{false};
    orbslam3::msg::MapBatch request_;
    std::vector<MapPointBatchData> batch_;
    rclcpp::Publisher<orbslam3::msg::MapBatch>::SharedPtr publisher_;
    rclcpp::Subscription<orbslam3::msg::TxCompletion>::SharedPtr subscription_;
};
}
#endif
