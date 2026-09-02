# UART V3 ACK extension

This document extends the existing V3 frame without changing its header, byte order, CRC, or MAP_BATCH encoding.

## Generic frame

All multibyte fields are little-endian. A frame is at most 200 bytes.

| Offset | Size | Field |
|---:|---:|---|
| 0 | 2 | magic `AA 55` |
| 2 | 1 | version `03` |
| 3 | 1 | message type |
| 4 | 1 | source node |
| 5 | 1 | destination node |
| 6 | 4 | sequence |
| 10 | 1 | TTL |
| 11 | 2 | payload length |
| 13 | N | payload |
| 13+N | 2 | CRC-16/CCITT-FALSE, low byte first |

CRC covers magic, header, and payload. Parameters: polynomial `0x1021`, initial value `0xFFFF`, no reflection, no final XOR.

## ACK (`message_type=0x30`)

ACK is a normal V3 frame. Its header sequence equals the acknowledged frame sequence. Source and destination are swapped from the acknowledged frame and TTL is zero.

The ACK payload is exactly 6 bytes:

| Payload offset | Size | Field |
|---:|---:|---|
| 0 | 4 | session ID copied from MAP_BATCH payload |
| 4 | 1 | acknowledged message type |
| 5 | 1 | status; `0x00 = RECEIVED` |

Total ACK length is 21 bytes. STM32 sends ACK only after the complete input frame passes magic, version, length, and CRC checks. It never acknowledges ACK frames, preventing ACK loops.

`RECEIVED` means the STM32 accepted an intact V3 frame. It does not mean map application, LoRa forwarding, or durable storage succeeded.

S100 logs matched ACK and timeout for link verification. This extension does not enable retransmission and does not change the existing `TRANSPORT_SENT`/CommMap commit behavior.
