# S100 → STM32 USART1 联调

## 已实现

本工程为 STM32F103C8T6，使用现有 8MHz HSE / 72MHz 系统时钟。
USART1：PA9 TX、PA10 RX，115200、8N1，无软硬件流控。
RX DMA1 Channel5 已改为 Circular，TX DMA1 Channel4 为 Normal。
USART1 和 RX DMA 中断抢占优先级均为 1；TX 为 2。保留 HT/TC 中断。
`USART.ioc` 已同步 RX Circular 配置。旧的 `@文本\r\n`、LED 命令和回显接收已移除。

`uart_link.c` 使用 ReceiveToIdle DMA。512B DMA 缓冲区通过 HT/TC/IDLE 搬运至
2048B 软件环形缓冲区，主循环调用 `UART_Link_Poll()` 组帧。
IDLE 仅表示当前字节到达，不等于完整协议帧。
回调读取实际 DMA 写位置，避免延迟 HT 回调带来的旧位置造成重复读。
不要同时调用 `HAL_UART_Receive_IT()` 或另一个 Receive DMA 函数。
不要在主循环加入长时间阻塞任务；115200 满速下 DMA 半缓冲时间约 22ms，
软件环形缓冲区约能容纳 178ms 数据，不能保证任意长阻塞后仍无丢失。

V3 通用帧：`AA 55 | version=03 | type | src | dst | seq:u32 | ttl:u8 |
payload_len:u16 | payload | crc:u16`，多字节小端。13B Header，最大185B payload，
总长15–200B。CRC-16/CCITT-FALSE：初值 FFFF、多项式1021，覆盖 magic/header/payload，
CRC 低字节在前。解析器支持拆包、粘包、噪声重同步、版本/长度/CRC 拒绝。
未完成帧在软件接收队列为空且250ms无新数据时清空；UART 错误或队列溢出后
丢弃损坏流并重启 RX DMA，不会把损坏帧当作成功帧。

本阶段只接收通用 V3 帧并校验、显示长度/序号/HEX，**不执行地图操作、运动命令，
不实现 ACK、重试或 LoRa 转发**。目的地址 F0 的地图帧也会接收用于联调。
未来业务可覆盖弱函数 `UART_Link_OnFrame()`，回调在主循环执行，帧指针仅在回调期间有效。

## 编译和烧录

打开 `MDK-ARM/USART.uvprojx`，选择 USART Target，Rebuild，再用自己的 ST-Link 下载。
工程已包含 `uart_link.c` 与 `v3_protocol.c`，无需手动添加。
本次生成的固件为 `MDK-ARM/USART/USART.hex`，编译记录为 `build_uart.log`。
未执行烧录，未验证真实 S100 接线。OLED 保留初始化及静态提示，不参与接收结果判断。
若 CubeMX 重新生成工程，确认这两个自定义源文件仍在 Keil 工程中。

## 接线与日志

先根据 S100 官方板卡资料确认实际 UART 引脚、设备映射及双方电平兼容性，
必要时使用电平转换；不要将 RS232 或 5V 电平直接接入。不猜测 S100 引脚或 tty 编号。
断电接线，双方共地，不连接两块板子的 VCC。

| S100 | STM32 |
|---|---|
| 已确认的 UART TX | PA10 / USART1_RX |
| GND | GND |
| 已确认的 UART RX（可选读日志） | PA9 / USART1_TX |

默认 `UART_LINK_DEBUG_HEX=1`：通过 PA9 异步输出 ASCII 长度/序号/CRC OK 和整帧 HEX。
可将 PA9 接到电平兼容的 USB-TTL **RX** 观察，USB-TTL GND 共地；不要让两个 TX 并接。
也可接回 S100 RX 并用下方测试脚本读取；现有 ROS Gateway 未实现日志接收，
不要期待其自动显示 STM32 返回内容。

HEX 日志约为原始数据的3倍，同波特率下无法逐帧全量输出满速输入。
TX 忙时仅跳过本帧日志，接收继续，`debug_dropped` 计数增加。
连续吞吐测试或未来双向二进制通信前，设 `UART_LINK_DEBUG_HEX=0` 后重新编译。
调试器 Watch 可观察 `uart_link_stats`：frames、rx_bytes、crc_errors、header_errors、
uart_errors、overflows、restarts、timeout_resets、debug_dropped、last_length、last_sequence、last_frame。

## 实机验收

1. 停止占用该串口的 Gateway/串口终端；确认它不是 Linux 调试控制台。
2. 将 `tests/send_golden.py` 复制到 S100，只依赖 Python 标准库。
3. 使用已确认的设备名运行（下面的路径是占位符，必须替换）：

   ```bash
   python3 send_golden.py --device /dev/REPLACE_WITH_CONFIRMED_UART --read-debug
   ```

4. 若接了 PA9 回 S100 RX，应看到 `V3 len=191 seq=16909060 CRC OK`，
   HEX 起始 `AA 55 03 10 01 F0 04 03 02 01`，末尾 `59 46`。
   若独立 USB-TTL 观察日志，可去掉 `--read-debug`。
   STM32 `frames` 应增加1、`last_length=191`，last_frame 与脚本输出逐字节一致。
5. 单帧确认后退出脚本，再运行 ROS Gateway，不要让脚本与 Gateway 同时占用串口：

   ```bash
   source /home/sunrise/ros2_ws/install/setup.bash
   ros2 run orbslam3 serial_gateway_node --ros-args \
     -p transport_mode:=uart \
     -p serial_device:=/dev/REPLACE_WITH_CONFIRMED_UART \
     -p baud_rate:=115200 -p write_timeout_ms:=250
   ```

   Gateway 本身不产生地图，仍需现有 MapBatch producer / SLAM 节点。
   Linux 的 TRANSPORT_SENT 不是 STM32 接收确认。

## 软件测试（不等同硬件验收）

在工程上级目录 PowerShell 运行：

```powershell
gcc -std=c99 -Wall -Wextra -Werror -I USART/Core/Inc USART/Core/Src/v3_protocol.c USART/tests/test_v3.c -o USART/tests/test_v3.exe
./USART/tests/test_v3.exe
gcc -std=c99 -Wall -Wextra -Werror -I USART/Core/Inc USART/Core/Src/v3_protocol.c USART/tests/test_uart_link.c -o USART/tests/test_uart_link.exe
./USART/tests/test_uart_link.exe
```

协议测试使用 S100 实际 golden 数据和固定 CRC 尾字节，覆盖所有拆分点、粘包、
损坏CRC、错误版本、超长字段、15/200B边界、重置与长噪声。
HAL mock 测试执行实际 uart_link.c，覆盖 DMA 回绕/连续输入/延迟HT、
日志忙、超时、UART 错误、溢出与启动失败恢复。mock 无法验证电气、
真实中断时序、DMA 总线或 STM32 是否实际收到字节。
