#ifndef COMM_TRANSPORT_HPP
#define COMM_TRANSPORT_HPP
#include <cstdint>
#include <vector>
namespace comm
{
class ITransport
{
public:
    virtual ~ITransport() = default;
    virtual bool Send(const std::vector<std::uint8_t>& bytes) = 0;
};
class SimulatedTransport final : public ITransport
{
public:
    bool Send(const std::vector<std::uint8_t>& bytes) override
    {
        return !bytes.empty() && bytes.size() <= 200;
    }
};
}
#endif
