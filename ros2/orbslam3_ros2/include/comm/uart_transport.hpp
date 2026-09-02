#ifndef COMM_UART_TRANSPORT_HPP
#define COMM_UART_TRANSPORT_HPP
#include "comm/transport.hpp"
#include <string>
#include <functional>
#include <vector>
#include <sys/types.h>
#include <termios.h>
namespace comm
{
namespace detail
{
using WriteFunction = std::function<ssize_t(int, const void*, std::size_t)>;
bool WriteAll(int fd, const std::vector<std::uint8_t>& bytes, int timeout_ms,
              const WriteFunction& write_some);
}
// Single owner; Send and Close must not be called concurrently.
class UartTransport final : public ITransport
{
public:
    UartTransport(const std::string& device, int baud_rate, int timeout_ms = 250);
    ~UartTransport() override;
    UartTransport(const UartTransport&) = delete;
    UartTransport& operator=(const UartTransport&) = delete;
    bool Send(const std::vector<std::uint8_t>& bytes) override;
    std::vector<std::uint8_t> ReadAvailable(std::size_t max_bytes = 512);
    void Close() noexcept;
private:
    int fd_{-1};
    int timeout_ms_;
    bool exclusive_{false};
    bool saved_{false};
    termios original_{};
};
}
#endif
