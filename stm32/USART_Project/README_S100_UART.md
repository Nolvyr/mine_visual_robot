# S100 ↔ STM32 USART1 V3 ACK 联调

## 通信配置

STM32F103C8T6 使用 USART1：PA9 TX、PA10 RX，115200、8N1，无流控。RX 使用循环 DMA，字节在主循环中完成 V3 组帧和 CRC 校验；中断回调只搬运字节。

接线前确认双方均为兼容的 TTL 电平，不要连接 VCC：

| S100 | STM32 |
|---|---|
| 已确认 UART TX | PA10 / USART1 RX |
| 已确认 UART RX | PA9 / USART1 TX |
| GND | GND |

## ACK 行为

STM32 仅在帧头、版本、长度及 CRC 全部正确后返回21字节二进制 ACK。ACK 的 seq 与原帧一致，载荷为 session、原消息类型和 RECEIVED 状态。STM32 不回复 ACK 帧，避免确认循环。

`RECEIVED` 只证明 STM32 收到了完整帧并通过 CRC，不代表地图已经应用或 LoRa 已发送。本阶段不自动重发，原有 `TRANSPORT_SENT` 和 CommMap Commit 行为不变。

完整格式见 `docs/uart_v3_ack.md`。

## 固定包测试

停止 Gateway 和所有串口工具，确保目标设备不是 Linux 控制台。把 `tests/send_golden.py` 复制到 S100，然后运行：

```bash
python3 send_golden.py \
  --device /dev/REPLACE_WITH_CONFIRMED_UART \
  --read-ack
```

成功输出：

```text
Driver accepted 191B; waiting for ACK is required to confirm STM32 reception.
[RxACK] session=305419896 seq=16909060 type=0x10 status=RECEIVED
```

只有第二行能确认 STM32 收到。脚本仍接受旧参数名 `--read-debug`，但读取的是二进制 ACK。

## ROS2 节点联调

不要让固定包脚本与 Gateway 同时占用串口。运行：

```bash
source /home/sunrise/ros2_ws/install/setup.bash
ros2 run orbslam3 serial_gateway_node --ros-args \
  -p transport_mode:=uart \
  -p serial_device:=/dev/REPLACE_WITH_CONFIRMED_UART \
  -p baud_rate:=115200 \
  -p write_timeout_ms:=250 \
  -p ack_timeout_ms:=500
```

还需运行现有 MapBatch producer/SLAM。成功闭环应出现：

```text
[Gateway] session=... seq=... type=MAP_BATCH ...
[Transport] mode=uart seq=... bytes=... complete local write
[TxCompletion] session=... seq=... status=TRANSPORT_SENT
[RxACK] session=... seq=... type=0x10 status=RECEIVED
```

若500ms内没有匹配确认：

```text
[ACK_TIMEOUT] session=... seq=...
```

超时可能来自接线、设备名、波特率、STM32固件、CRC或返回通道；当前不会自动重发。

## 调试统计

Keil Watch 中观察 `uart_link_stats`：

- `frames`：CRC正确的接收帧数；
- `ack_sent`：已交给TX DMA的ACK数；
- `ack_dropped`：单槽ACK队列已满时丢弃数；
- `crc_errors/header_errors`：协议错误；
- `uart_errors/overflows/restarts`：串口和缓冲恢复；
- `last_sequence/last_length/last_frame`：最后一帧。

## 验证范围

Windows 测试覆盖 V3 golden、ACK字段/CRC、拆包/粘包、DMA回绕、TX繁忙、超时和错误恢复。Keil工程可直接 Rebuild 并烧录 `MDK-ARM/USART/USART.hex`。软件测试不能代替真实电气和接线验收。