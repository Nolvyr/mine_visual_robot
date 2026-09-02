# 通信网关接口与 Packet V3（2026-08-31）

## 本轮边界及实际运行状态

基线：`7e76671` / `comm-pending-v6.1-validated`。本轮不改 ROS Topic、不接串口、
不修改 CommMap 算法，不改变 V2 字节格式，不启动网关或算法空节点。

当前运行链：Stereo callback → CommMap::Update → BinaryProtocol V2 →
CommWorker 模拟发送 → TxCompletion → 下一通信周期按序列匹配 CommitBatch。
最多一个 in-flight；Worker 不访问 ORB 对象。`synced_now` 表示按当前模拟发送
成功结果已提交的状态，**不是远端 PC 已收到的证明**。

新增 `comm_packet_encoder` 静态库，独立于 ROS/ORB/UART。
`MapPacketEncoder` 接受 CommMap 产生的普通数据；`PacketEncoder` 只理解头字段和字节。
本轮通过单元测试打通地图数据到 V3 Packet；没有替换 Stereo 的 V2 实机基准路径。
V3 与 V2 不兼容，不能交给旧解析器。没有在 V3 内嵌整个 V2 帧/双重 CRC。

检查结果：项目节点/CommMap 无 UART 打开操作。工作区发现 `/home/sunrise/uart2-40pin.dtso`
设备树使能文件，但未找到独立 UART 测试源程序；本轮没有重跑物理串口测试。
200B 是用户指定的设计预算，不是本轮核实过的 E200 硬件/空中 MTU。

## 模块与未来网关接入

```text
算法 / CommMap
  → ROS 语义消息（未来定义 msg）
  → serial_gateway_node（唯一 UART 所有者）
  → MapPacketEncoder（地图元数据与增量记录）
  → PacketEncoder（路由头 + payload + CRC）
  → UART Transport（未来；仅 bytes/length）
```

`MapBatchMessage` 是普通 C++ 值结构，**不是 ROS msg，也不是线协议结构体**：
session_id:uint32、sequence:uint32、active_map_id:uint16、operations:MapPointBatchData[]。
序列由生产方分配并保持到 Completion；源/目的地址/TTL 是网关策略参数。
`MapPointBatchData` 中 raw/previous/delta 等调试字段不序列化。

未来从 `CommMapResult.batch` 创建上述消息，网关验证并调用：

```cpp
comm::MapBatchMessage message;
message.session_id = session_id;
message.sequence = sequence;
message.active_map_id = result.active_map_id;
message.operations = result.batch;
auto bytes = comm::MapPacketEncoder{}.Encode(message, 0x01, 0xF0, 1);
```

不要让算法节点打开串口。网关只负责编码、解码、传输与队列；运动仲裁属于
decision_node。未来跨进程接入必须将 Completion 返回生产方，匹配
`producer/source + session_id + sequence` 后 Commit；不能因 publish 成功就 Commit。
当前进程内 sequence-only Completion 不能直接当成跨进程完整关联接口。
网关断开/重启、接收者重启、超时与端到端确认语义是后续任务，尚未实现。

## Topic 名称约定（不创建空 Topic）

| Topic | 生产方 → 消费方 | 内容约定 |
|---|---|---|
| /slam/pose | SLAM → decision/gateway | 时间戳、坐标系、位姿 |
| /slam/status | SLAM → decision/gateway | 跟踪状态、会话 |
| /detection/objects | YOLO → decision | 类别、置信度、检测框、图像时间戳 |
| /obstacle/info | 深度避障 → decision | 左中右距离、有效性、时间戳 |
| /control/latest_cmd | decision → gateway | 最新控制意图、有效期；非自主串口决策 |
| /remote/command | gateway → decision | PC 任务命令、来源、命令标识 |
| /stm32/status | gateway → decision | 底盘、电池、故障与时间戳 |
| /comm/map_batch | CommMap 所属节点 → gateway | 会话、序列、active_map、操作数组 |
| /comm/link_status | gateway → 监控/decision | 链路状态、队列与错误计数 |
| /comm/tx_completion | gateway → 地图生产方 | 拟定：来源、会话、序列、发送结果 |

上述字段只是后续 ROS msg 设计约定，未注册消息类型或 QoS。
现有 `/orbslam3/pose`、`/orbslam3/tracking_state`、`/orbslam3/path`、
`/orbslam3/map_points`、`/orbslam3/comm_map_points` 保持原名；未来用 remap/适配接入，
不要现在将 `/slam/*` 宣称为已发布 Topic。

## V3 字节布局（全为显式序列化，小端）

| 字节偏移 | 长度 | 字段 |
|---|---|---|
| 0 | 2 | Magic AA 55 |
| 2 | 1 | Version 03（旧协议为 02） |
| 3 | 1 | Message Type；混合地图批次为 10 |
| 4 | 1 | Source |
| 5 | 1 | Destination |
| 6 | 4 | Sequence uint32 LE |
| 10 | 1 | TTL |
| 11 | 2 | Payload Length uint16 LE |
| 13 | L | Payload |
| 13+L | 2 | CRC16 LE |

Header=13B；CRC-16/CCITT-FALSE：poly=1021，init=FFFF，refin/refout=false，xorout=0。
覆盖 Magic 起至 Payload 末尾，不包含 CRC 自身。最大通用 Payload=185B，Packet=200B。
未知消息类型可作为不透明 Payload 编码；这不代表该类型业务已实现。

MAP_BATCH Payload：session_id(4) + active_map_id(2) + operation_count(1) + records。
Session 是地图身份上下文，不能仅靠路由地址替代；无 active map 用 FFFF。

记录沿用 V2：

- ADD/UPDATE：op(1) + map_id(2) + point_id(4) + x_cm(2) + y_cm(2) + z_cm(2) = **13B**。
- DELETE：op(1) + point_id(4) = **5B**；PointID 身份以 Session 为范围。
- op 为 01/02/03；XYZ 为已有 int16 厘米量化，负值以补码小端发送。

一个 ADD 整包=13+7+13+2=35B。
13 ADD/UPDATE 整包=13+7+169+2=**191B**（Payload=176B）。
14 ADD/UPDATE 整包=204B，拒绝。
最坏情况容量=floor((200-13-2-7)/13)=**13 条**。
纯 DELETE 理论容量=floor(178/5)=35 条，但本轮统一安全上限仍为13条，不改变调度器。
上限由常量算出并与安全上限13取最小；超限拒绝，绝不静默截断已准备 Batch。
空地图 Batch 和非法 op 拒绝；通用 Packet 可有空 Payload。

固定输入 session=12345678、seq=01020304、active_map=2、src=1、dst=F0、ttl=3，
第 i 条 ADD id=i、x=100+i、y=-200-i、z=300+i（i=1..13）：
191B，CRC=**4659**，尾部 **59 46**。测试完整比较191字节。
旧 V2 Golden 仍为188B、CRC=7E05，两者不要混用。

## 扩展保留（仅枚举，不实现调度和业务）

节点：探索车01；中继10..1F；地面站F0；广播FF（只可作目的地址）。
TTL=0 可编码作本地交付；未来转发应拒绝继续转发0，递减 TTL 后重新算 CRC。
没有路由、去重、自动中继选择或广播风暴抑制实现。

类型：HEARTBEAT=01、POSE=02、SLAM_STATUS=03、VEHICLE_STATUS=04；
MAP_BATCH=10、MAP_ADD=11、MAP_UPDATE=12、MAP_DELETE=13、MAP_RESET=14；
MOTION_CMD=20、CMD_STOP=21、CMD_RETURN_HOME=22、CMD_EXPLORE=23；
ACK=30、NACK=31、RELAY_STATUS=40、LINK_STATUS=41（数值均为十六进制）。
只有 MAP_BATCH 混合操作 Payload 已实现；独立 MAP_ADD/UPDATE/DELETE 类型仅预留。

Priority：STOP 为 Emergency，控制命令/ACK/NACK 为 High，位姿/状态/心跳为 Normal，
地图为 Low。Priority 不在线上传输，不改变 V6.0 FIFO；真正网关需做优先级调度。
未实现 UART、STM32 parser、YOLO、避障、decision、ACK重传或 PC 显示。
