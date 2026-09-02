#include "rclcpp/rclcpp.hpp"
#include "orbslam3/msg/map_batch.hpp"
#include "orbslam3/msg/tx_completion.hpp"
#include "comm/map_packet_encoder.hpp"
#include "comm/transport.hpp"
#include "comm/uart_transport.hpp"

class SerialGateway : public rclcpp::Node
{
public:
    SerialGateway() : Node("serial_gateway_node")
    {
        destination_ = declare_parameter<int>("destination_id", 0xF0);
        ttl_ = declare_parameter<int>("ttl", 3);
        if (destination_ < 0 || destination_ > 255 || ttl_ < 0 || ttl_ > 255)
            throw std::invalid_argument("destination_id and ttl must fit uint8");
        mode_ = declare_parameter<std::string>("transport_mode", "simulated");
        const auto device = declare_parameter<std::string>("serial_device", "");
        const int baud = declare_parameter<int>("baud_rate", 115200);
        const int timeout = declare_parameter<int>("write_timeout_ms", 250);
        if (mode_ == "simulated") transport_.reset(new comm::SimulatedTransport);
        else if (mode_ == "uart")
        {
            try { transport_.reset(new comm::UartTransport(device, baud, timeout)); }
            catch (const std::exception& error)
            {
                RCLCPP_ERROR(get_logger(), "UART unavailable: %s; requests will fail", error.what());
            }
        }
        else throw std::invalid_argument("transport_mode must be simulated or uart");
        const auto qos = rclcpp::QoS(16).reliable().durability_volatile();
        completion_ = create_publisher<orbslam3::msg::TxCompletion>("/comm/tx_completion", qos);
        input_ = create_subscription<orbslam3::msg::MapBatch>("/comm/map_batch", qos,
            [this](orbslam3::msg::MapBatch::ConstSharedPtr message) { Send(*message); });
        RCLCPP_INFO(get_logger(), "Gateway mode=%s device=%s baud=%d timeout=%dms available=%d",
            mode_.c_str(), device.c_str(), baud, timeout, bool(transport_));
    }
private:
    void Send(const orbslam3::msg::MapBatch& request)
    {
        orbslam3::msg::TxCompletion result;
        result.source_id = request.source_id; result.session_id = request.session_id;
        result.sequence = request.sequence; result.message_type = request.message_type;
        result.status = orbslam3::msg::TxCompletion::FAILED;
        try
        {
            if (request.message_type != static_cast<std::uint8_t>(comm::MessageType::MAP_BATCH))
                throw std::invalid_argument("unsupported message type");
            comm::MapBatchMessage message;
            message.session_id = request.session_id; message.sequence = request.sequence;
            message.active_map_id = request.active_map_id;
            for (const auto& op : request.operations)
            {
                comm::MapPointBatchData p;
                p.operation = static_cast<comm::MapPointOperation>(op.operation);
                p.map_id = op.map_id; p.id = op.id;
                p.x_cm = op.x_cm; p.y_cm = op.y_cm; p.z_cm = op.z_cm;
                message.operations.push_back(p);
            }
            const auto packet = encoder_.Encode(message, request.source_id, destination_, ttl_);
            RCLCPP_INFO(get_logger(),
                "[Gateway] session=%u seq=%u type=MAP_BATCH points=%zu packet=%zuB src=0x%02X dst=0x%02X ttl=%d",
                request.session_id, request.sequence, message.operations.size(), packet.size(),
                request.source_id, destination_, ttl_);
            if (transport_ && transport_->Send(packet))
            {
                result.status = orbslam3::msg::TxCompletion::TRANSPORT_SENT;
                RCLCPP_INFO(get_logger(), "[Transport] mode=%s seq=%u bytes=%zu complete local write",
                    mode_.c_str(), request.sequence, packet.size());
            }
            else RCLCPP_ERROR(get_logger(), "[Transport] seq=%u FAILED (unavailable/write failure)", request.sequence);
        }
        catch (const std::exception& error)
        {
            RCLCPP_ERROR(get_logger(), "[Gateway] seq=%u failed: %s", request.sequence, error.what());
        }
        completion_->publish(result);
        RCLCPP_INFO(get_logger(), "[TxCompletion] session=%u seq=%u status=%s",
            result.session_id, result.sequence,
            result.status == result.TRANSPORT_SENT ? "TRANSPORT_SENT" : "FAILED");
    }
    int destination_, ttl_;
    std::string mode_;
    comm::MapPacketEncoder encoder_;
    std::unique_ptr<comm::ITransport> transport_;
    rclcpp::Publisher<orbslam3::msg::TxCompletion>::SharedPtr completion_;
    rclcpp::Subscription<orbslam3::msg::MapBatch>::SharedPtr input_;
};
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SerialGateway>());
    rclcpp::shutdown();
    return 0;
}
