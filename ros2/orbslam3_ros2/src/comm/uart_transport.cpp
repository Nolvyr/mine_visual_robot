#include "comm/uart_transport.hpp"
#include <cerrno>
#include <chrono>
#include <stdexcept>
#include <system_error>
#include <fcntl.h>
#include <poll.h>
#include <sys/file.h>
#include <sys/ioctl.h>
#include <unistd.h>

namespace
{
speed_t Baud(int baud)
{
    switch (baud)
    {
    case 9600: return B9600;
    case 19200: return B19200;
    case 38400: return B38400;
    case 57600: return B57600;
    case 115200: return B115200;
    case 230400: return B230400;
    case 460800: return B460800;
    case 921600: return B921600;
    default: throw std::invalid_argument("unsupported UART baud_rate");
    }
}
void Check(int result, const char* operation)
{
    if (result < 0) throw std::system_error(errno, std::generic_category(), operation);
}
}
namespace comm
{
bool detail::WriteAll(int fd, const std::vector<std::uint8_t>& bytes,
                      int timeout_ms, const WriteFunction& write_some)
{
    if (fd < 0 || bytes.empty() || timeout_ms <= 0) return false;
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
    std::size_t offset = 0;
    while (offset < bytes.size())
    {
        const auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(
            deadline - std::chrono::steady_clock::now()).count();
        if (remaining <= 0) return false;
        pollfd event{fd, POLLOUT, 0};
        const int ready = ::poll(&event, 1, static_cast<int>(remaining));
        if (ready < 0 && errno == EINTR) continue;
        if (ready <= 0 || (event.revents & (POLLERR | POLLHUP | POLLNVAL))) return false;
        if (!(event.revents & POLLOUT)) continue;
        const auto count = write_some(fd, bytes.data() + offset, bytes.size() - offset);
        if (count < 0 && (errno == EINTR || errno == EAGAIN || errno == EWOULDBLOCK)) continue;
        if (count <= 0 || static_cast<std::size_t>(count) > bytes.size()-offset) return false;
        offset += static_cast<std::size_t>(count);
    }
    return true;
}
UartTransport::UartTransport(const std::string& device, int baud_rate, int timeout_ms)
    : timeout_ms_(timeout_ms)
{
    const auto speed = Baud(baud_rate);
    if (device.empty() || timeout_ms <= 0 || timeout_ms > 5000)
        throw std::invalid_argument("UART device required; timeout must be 1..5000ms");
    try
    {
        fd_ = ::open(device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
        Check(fd_, "open UART");
        Check(::flock(fd_, LOCK_EX | LOCK_NB), "lock UART");
        Check(::ioctl(fd_, TIOCEXCL), "exclusive UART");
        exclusive_ = true;
        Check(::tcgetattr(fd_, &original_), "read UART settings");
        saved_ = true;
        auto options = original_;
        ::cfmakeraw(&options);
        options.c_cflag &= ~(CSIZE | PARENB | PARODD | CSTOPB | CRTSCTS);
        options.c_cflag |= CS8 | CLOCAL | CREAD;
        options.c_iflag &= ~(IXON | IXOFF | IXANY);
        options.c_cc[VMIN] = 0; options.c_cc[VTIME] = 0;
        Check(::cfsetispeed(&options, speed), "input baud");
        Check(::cfsetospeed(&options, speed), "output baud");
        Check(::tcsetattr(fd_, TCSANOW, &options), "configure UART 8N1");
    }
    catch (...) { Close(); throw; }
}
UartTransport::~UartTransport() { Close(); }
void UartTransport::Close() noexcept
{
    if (fd_ < 0) return;
    if (saved_) ::tcsetattr(fd_, TCSANOW, &original_);
    if (exclusive_) ::ioctl(fd_, TIOCNXCL);
    ::close(fd_); // Linux close must not be retried after EINTR.
    fd_ = -1; saved_ = false; exclusive_ = false;
}
bool UartTransport::Send(const std::vector<std::uint8_t>& bytes)
{
    if (bytes.empty() || bytes.size() > 200) return false;
    const bool success = detail::WriteAll(fd_, bytes, timeout_ms_,
        [](int fd, const void* data, std::size_t count) { return ::write(fd, data, count); });
    // Fail closed: no automatic reopen or ambiguous replay after a partial send.
    if (!success) Close();
    return success;
}
std::vector<std::uint8_t> UartTransport::ReadAvailable(const std::size_t max_bytes)
{
    if (fd_ < 0 || max_bytes == 0 || max_bytes > 4096) return {};
    std::vector<std::uint8_t> bytes(max_bytes);
    for (;;)
    {
        const ssize_t count = ::read(fd_, bytes.data(), bytes.size());
        if (count > 0) { bytes.resize(static_cast<std::size_t>(count)); return bytes; }
        if (count == 0 || errno == EAGAIN || errno == EWOULDBLOCK) return {};
        if (errno == EINTR) continue;
        const std::system_error error(errno, std::generic_category(), "read UART");
        Close(); throw error;
    }
}
}
