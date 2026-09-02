#include "comm/uart_transport.hpp"
#include "comm/map_packet_encoder.hpp"
#include <pty.h>
#include <unistd.h>
#include <poll.h>
#include <cerrno>
#include <algorithm>
#include <iostream>
#include <stdexcept>

void Check(bool ok, const char* reason) { if (!ok) throw std::runtime_error(reason); }
template<class F> void Reject(F f)
{
    try { f(); } catch (const std::exception&) { return; }
    throw std::runtime_error("expected failure");
}
struct Pty
{
    int master{-1}, slave{-1}; char name[128]{};
    Pty() { Check(openpty(&master,&slave,name,nullptr,nullptr)==0,"openpty"); }
    ~Pty() { if(master>=0) close(master); if(slave>=0) close(slave); }
    std::vector<std::uint8_t> Read(std::size_t size)
    {
        std::vector<std::uint8_t> out(size); std::size_t offset=0;
        while(offset<size)
        {
            pollfd event{master,POLLIN,0};
            Check(poll(&event,1,1000)>0,"read timeout");
            auto n=read(master,out.data()+offset,size-offset);
            Check(n>0,"read failure"); offset+=n;
        }
        return out;
    }
};
int main()
{
    try
    {
        comm::MapBatchMessage message;
        message.session_id=0x12345678; message.sequence=0x01020304; message.active_map_id=2;
        for(int i=1;i<=13;++i)
        {
            comm::MapPointBatchData p; p.id=i; p.map_id=2;
            p.x_cm=100+i; p.y_cm=-200-i; p.z_cm=300+i; message.operations.push_back(p);
        }
        const auto packet=comm::MapPacketEncoder{}.Encode(message,1,0xF0,3);
        Check(packet.size()==191 && packet[189]==0x59 && packet[190]==0x46,"golden CRC");
        Pty pty;
        comm::UartTransport uart(pty.name,115200,100);
        termios settings{};
        Check(tcgetattr(pty.slave,&settings)==0,"getattr");
        Check((settings.c_cflag&CSIZE)==CS8 && !(settings.c_cflag&(PARENB|CSTOPB|CRTSCTS))
            && !(settings.c_iflag&(IXON|IXOFF|IXANY)) && cfgetospeed(&settings)==B115200,"8N1");
        Reject([&] { comm::UartTransport second(pty.name,115200); });
        Check(uart.Send(packet),"Send191");
        Check(pty.Read(191)==packet,"PTY bytes differ from V3 golden encoder");
        const std::vector<std::uint8_t> ack{0xAA,0x55,0x03,0x30,0xF0,0x01,0x04,0x03,0x02,0x01,0x00,0x06,0x00,0x78,0x56,0x34,0x12,0x10,0x00,0x17,0xE1};
        Check(write(pty.master,ack.data(),ack.size())==static_cast<ssize_t>(ack.size()),"write ACK");
        std::vector<std::uint8_t> received;
        for(int attempt=0;attempt<20&&received.empty();++attempt){received=uart.ReadAvailable();if(received.empty())usleep(1000);}
        Check(received==ack,"UART ACK read differs from STM32 golden");
        int calls=0;
        Check(comm::detail::WriteAll(pty.slave,packet,100,
            [&](int fd,const void* bytes,std::size_t n)->ssize_t {
                ++calls;
                if(calls==1) { errno=EINTR; return -1; }
                if(calls==2) { errno=EAGAIN; return -1; }
                return write(fd,bytes,std::min<std::size_t>(n,7));
            }),"partial/EINTR/EAGAIN");
        Check(calls>20 && pty.Read(191)==packet,"partial byte preservation");
        Check(!comm::detail::WriteAll(pty.slave,packet,20,
            [](int,const void*,std::size_t)->ssize_t { errno=EAGAIN; return -1; }),"timeout");
        Check(!comm::detail::WriteAll(pty.slave,packet,100,
            [](int,const void*,std::size_t)->ssize_t { return 0; }),"zero write");
        Check(!comm::detail::WriteAll(pty.slave,packet,100,
            [](int,const void*,std::size_t)->ssize_t { errno=EIO; return -1; }),"EIO");
        Check(!uart.Send({}) && !uart.Send(std::vector<std::uint8_t>(201)),"invalid sizes");
        uart.Close(); uart.Close(); Check(!uart.Send(packet),"send after close");
        { comm::UartTransport reopened(pty.name,115200); }
        { comm::UartTransport reopened(pty.name,115200); }
        Reject([] { comm::UartTransport missing("/dev/nonexistent-codex-uart",115200); });
        Reject([&] { comm::UartTransport bad(pty.name,12345); });
        Reject([&] { comm::UartTransport bad(pty.name,115200,0); });
        Pty unplugged;
        comm::UartTransport failed(unplugged.name,115200,100);
        close(unplugged.master); unplugged.master=-1;
        Check(!failed.Send(packet) && !failed.Send(packet),"disconnect must fail closed");
        std::cout<<"UART PTY PASS: 191B TX + 21B STM32 ACK RX, 8N1, exclusive, partial/EINTR/EAGAIN, timeout, EIO, close, disconnect\n";
        return 0;
    }
    catch(const std::exception& error) { std::cerr<<error.what()<<'\n'; return 1; }
}
