#include "comm/v3_ack_decoder.hpp"
#include <iostream>
#include <stdexcept>
#include <vector>
namespace
{
void Check(bool condition,const char* message){if(!condition)throw std::runtime_error(message);}
std::vector<std::uint8_t> GoldenAck(){return {0xAA,0x55,0x03,0x30,0xF0,0x01,0x04,0x03,0x02,0x01,0x00,0x06,0x00,0x78,0x56,0x34,0x12,0x10,0x00,0x17,0xE1};}
}
int main()
{
 try{
  comm::V3AckDecoder decoder;const auto golden=GoldenAck();auto result=decoder.Feed(golden.data(),7);
  Check(result.acknowledgements.empty(),"fragment decoded too early");result=decoder.Feed(golden.data()+7,golden.size()-7);
  Check(result.acknowledgements.size()==1,"golden ACK not decoded");const auto& ack=result.acknowledgements.front();
  Check(ack.source==0xF0&&ack.destination==0x01&&ack.sequence==0x01020304&&ack.session_id==0x12345678&&ack.acknowledged_message_type==0x10&&ack.status==0,"ACK fields differ from STM32 V3_BuildAck");
  std::vector<std::uint8_t> noisy{0x00,0x7F,0xAA};noisy.insert(noisy.end(),golden.begin(),golden.end());result=decoder.Feed(noisy);
  Check(result.acknowledgements.size()==1&&result.discarded_bytes>=2,"noise resynchronization failed");
  auto corrupt=golden;corrupt[13]^=1;result=decoder.Feed(corrupt);Check(result.acknowledgements.empty()&&result.crc_errors==1,"bad CRC accepted");
  result=decoder.Feed(golden);Check(result.acknowledgements.size()==1,"recovery after CRC error failed");
  auto wrong_version=golden;wrong_version[2]=2;result=decoder.Feed(wrong_version);Check(result.acknowledgements.empty()&&result.header_errors>=1,"wrong version accepted");
  decoder.Reset();std::cout<<"V3 ACK decoder PASS: 21B CRC=0xE117 fragments/noise/errors/recovery\n";return 0;
 }catch(const std::exception& error){std::cerr<<error.what()<<'\n';return 1;}
}
