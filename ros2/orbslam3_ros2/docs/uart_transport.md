# Linux/S100 UART 接入

默认仍为 simulated，不打开设备。只有 serial_gateway_node 创建 UartTransport。
CommMap、MapProducer、V3 Encoder 和 STM32 端没有修改。

每个终端先加载：

```bash
source /opt/ros/humble/setup.bash
source /home/sunrise/ros2_ws/install/setup.bash
```

先确认实际设备、接线与串口占用；下例 `/dev/ttyS2` **仅示例，未确认对应板端引脚**。
STM32 RX 与 S100 TX 交叉连接并共地；电平、复用与波特率按板卡资料确认，
不要连接 RS232 电平，不要选择系统 console/getty 所在口。无需改整机权限或关闭未知服务。

```bash
ros2 run orbslam3 serial_gateway_node --ros-args \
  -p transport_mode:=uart -p serial_device:=/dev/ttyS2 \
  -p baud_rate:=115200 -p write_timeout_ms:=250 \
  -p destination_id:=240 -p ttl:=3
```

回退：

```bash
ros2 run orbslam3 serial_gateway_node --ros-args -p transport_mode:=simulated
```

Stereo 沿用已有相机、标定、词典与图像 remap，使用 `comm_mode:=gateway_v3`。
`comm_mode:=legacy_v2` 是旧进程内模拟链路，不会经本网关输出 UART。

支持波特率：9600/19200/38400/57600/115200/230400/460800/921600。
8N1、raw、无 RTS/CTS 和 XON/XOFF。flock + TIOCEXCL 防止一般重复打开；
不能据此证明没有提前打开的进程或特权程序，请先人工确认设备占用。

## 发送/失败语义

- O_NONBLOCK；poll 等待可写；逐次累计 write 返回长度。
- EINTR/EAGAIN 继续当前帧，受单次总超时约束；不是协议重发。
- 完整字节被 Linux 驱动接受才返回 true → TRANSPORT_SENT。
- 不使用可能无限阻塞的 tcdrain；成功不证明最后一个字节已经出 TX，更不是 STM32 ACK。
- 写入0、超时、断开、系统错误返回 false 并关闭设备，之后请求均 FAILED。
- 打开/配置失败时网关保持在线返回 FAILED；不会偷偷回退 simulated。
- 无自动重连或失败帧重放；修复硬件后手动重启网关。
- 部分发送失败不能撤回已写字节；STM32 后续 parser 必须有重同步能力。
- Close 幂等，析构恢复原 termios、解除独占并关闭 fd。

本轮 Send 在网关互斥 callback 内执行，最多等待 write_timeout_ms（默认250ms），
不会阻塞独立 Stereo 进程；但会延迟本网关其他回调。后续控制命令/优先级接入前，
需将串口发送移入网关专属有界 worker，不能据此声称已实现实时控制发送。

## 验证步骤

1. 先运行 CTest 的 UART 伪终端和既有 V3 golden 测试。
2. 确认 STM32 同波特率8N1；本阶段只接收缓存并打印长度和HEX，不执行运动指令。
3. 单独启动一个 uart 网关及现有 Stereo+RealSense。
4. 选择日志中 points=13 的全 ADD 包，S100 应记录 packet=191B、complete local write。
5. STM32 按流累计，不能把一次中断/一次read当作一帧；核对 AA 55 03 10、长度与全部HEX。
6. V3头偏移11..12是小端 Payload Length，整包长度=13+PayloadLength+2。
7. 对应实际 session/seq 核对。固定 golden session=12345678、seq=01020304、ttl=3
   的13 ADD为191B，CRC=4659，尾字节59 46；实际运行不同会话/序列会改变CRC。

本轮不烧写 STM32，不实现 parser/E200/ACK/重试/YOLO。
伪终端成功只证明 Linux字节路径，不证明物理 S100→STM32 已打通。
