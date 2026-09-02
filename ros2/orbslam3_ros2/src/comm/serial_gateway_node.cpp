#include "rclcpp/rclcpp.hpp"
#include "orbslam3/msg/map_batch.hpp"
#include "orbslam3/msg/tx_completion.hpp"
#include "comm/map_packet_encoder.hpp"
#include "comm/transport.hpp"
#include "comm/uart_transport.hpp"
#include "comm/v3_ack_decoder.hpp"
#include <algorithm>
#include <chrono>
#include <deque>

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
        ack_timeout_ms_ = declare_parameter<int>("ack_timeout_ms", 1000);
        if (ack_timeout_ms_ <= 0 || ack_timeout_ms_ > 60000)
            throw std::invalid_argument("ack_timeout_ms must be 1..60000");
        if (mode_ == "simulated") transport_.reset(new comm::SimulatedTransport);
        else if (mode_ == "uart")
        {
            try
            {
                std::unique_ptr<comm::UartTransport> uart(new comm::UartTransport(device, baud, timeout));
                uart_ = uart.get(); transport_ = std::move(uart);
            }
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
        if (uart_) receive_timer_ = create_wall_timer(std::chrono::milliseconds(10), [this] { PollUart(); });
        RCLCPP_INFO(get_logger(), "Gateway mode=%s device=%s baud=%d timeout=%dms ack_timeout=%dms available=%d",
            mode_.c_str(), device.c_str(), baud, timeout, ack_timeout_ms_, bool(transport_));
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
                if (uart_) outstanding_.push_back(OutstandingAck{request.source_id,
                    static_cast<std::uint8_t>(destination_), request.session_id, request.sequence,
                    request.message_type, std::chrono::steady_clock::now()});
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
    struct OutstandingAck
    {
        std::uint8_t source, destination;
        std::uint32_t session_id, sequence;
        std::uint8_t message_type;
        std::chrono::steady_clock::time_point sent_at;
    };
    void PollUart()
    {
        try
        {
            const auto bytes=uart_->ReadAvailable();
            if (!bytes.empty())
            {
                const auto decoded=ack_decoder_.Feed(bytes);
                if(decoded.discarded_bytes||decoded.header_errors||decoded.crc_errors||decoded.non_ack_frames)
                    RCLCPP_WARN(get_logger(), "[RxV3] bytes=%zu discarded=%zu header_errors=%zu crc_errors=%zu non_ack=%zu",
                        bytes.size(),decoded.discarded_bytes,decoded.header_errors,decoded.crc_errors,decoded.non_ack_frames);
                for(const auto& ack:decoded.acknowledgements) HandleAck(ack);
            }
        }
        catch(const std::exception& error)
        {
            RCLCPP_ERROR(get_logger(),"[RxACK] UART read failed: %s",error.what());
            uart_=nullptr; receive_timer_->cancel();
        }
        ExpireAcks();
    }
    void HandleAck(const comm::V3Ack& ack)
    {
        const auto match=std::find_if(outstanding_.begin(),outstanding_.end(),[&ack](const OutstandingAck& expected){
            return ack.source==expected.destination && ack.destination==expected.source
                && ack.session_id==expected.session_id && ack.sequence==expected.sequence
                && ack.acknowledged_message_type==expected.message_type; });
        if(match==outstanding_.end())
        {
            RCLCPP_WARN(get_logger(),"[RxACK] unmatched src=0x%02X dst=0x%02X session=%u seq=%u type=0x%02X status=0x%02X",
                ack.source,ack.destination,ack.session_id,ack.sequence,ack.acknowledged_message_type,ack.status); return;
        }
        if(ack.status!=comm::V3AckDecoder::kReceivedStatus)
        {
            RCLCPP_WARN(get_logger(),"[RxACK] session=%u seq=%u type=0x%02X unsupported_status=0x%02X",
                ack.session_id,ack.sequence,ack.acknowledged_message_type,ack.status);
            outstanding_.erase(match); return;
        }
        RCLCPP_INFO(get_logger(),"[RxACK] session=%u seq=%u type=0x%02X status=RECEIVED peer=STM32",
            ack.session_id,ack.sequence,ack.acknowledged_message_type);
        outstanding_.erase(match);
    }
    void ExpireAcks()
    {
        const auto now=std::chrono::steady_clock::now();
        while(!outstanding_.empty() && std::chrono::duration_cast<std::chrono::milliseconds>(now-outstanding_.front().sent_at).count()>=ack_timeout_ms_)
        {
            const auto& expired=outstanding_.front();
            RCLCPP_WARN(get_logger(),"[ACK_TIMEOUT] session=%u seq=%u type=0x%02X",expired.session_id,expired.sequence,expired.message_type);
            outstanding_.pop_front();
        }
    }
    int destination_, ttl_;
    int ack_timeout_ms_;
    std::string mode_;
    comm::MapPacketEncoder encoder_;
    std::unique_ptr<comm::ITransport> transport_;
    comm::UartTransport* uart_{nullptr};
    comm::V3AckDecoder ack_decoder_;
    std::deque<OutstandingAck> outstanding_;
    rclcpp::TimerBase::SharedPtr receive_timer_;
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
