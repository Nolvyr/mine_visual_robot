# ROS2 Gateway 模拟链路

本轮新增真正的 `/comm/map_batch` 和 `/comm/tx_completion`，消息类型分别为
`orbslam3/msg/MapBatch` 和 `orbslam3/msg/TxCompletion`，在当前包内生成，避免大规模拆包。
MapOperation 只包含 op/map_id/id/int16 XYZ；数组上限13。无 C++ 内存布局透传。

默认 Stereo `comm_mode=gateway_v3`：CommMap → MapProducer（ROS publish）→
serial_gateway_node → MapPacketEncoder → V3 → ITransport/SimulatedTransport →
ROS completion → MapProducer → CommitBatch。

MapProducer 为 Stereo 和无相机集成测试共用的生产组件。所有操作与 completion
回调均属于节点默认 mutually-exclusive group；不会在 Worker 内修改 CommMap。
先保存 pending 再 publish；publish 失败回滚 pending；匹配 source_id、session_id、
sequence、message_type 且 status=TRANSPORT_SENT 才提交。FAILED 清 pending 不提交；
不匹配/未知状态保留 pending。单 in-flight；没有重试、ACK、超时重发。

启动前未发现网关输入订阅及 completion 发布者时不提交发送请求。
发送过程中网关退出或 completion 丢失时 pending 会保留：**不是自动恢复链路**。
本阶段只支持一个网关；启动多个网关会重复处理消息，不应这样部署。
Completion 不带认证；仅适用于可信 ROS domain。source 固定01；多生产者以后须分配唯一来源。

TRANSPORT_SENT 只表示当前 Transport 成功（本轮只是模拟），不证明 STM32/PC 收到。
网关参数 destination_id 默认240(0xF0)、ttl默认3；只预留路由信息，不进行转发。
模拟 Transport 是立即返回的同步接口；未来真实 UART 必须在网关内部处理阻塞、
部分写、关闭、队列和失败，不应直接把长时间阻塞写塞入 ROS callback。

## 启动

先在每个终端加载环境：

```bash
source /opt/ros/humble/setup.bash
source /home/sunrise/ros2_ws/install/setup.bash
```

网关：

```bash
ros2 run orbslam3 serial_gateway_node --ros-args -p destination_id:=240 -p ttl:=3
```

保持现有 RealSense 启动方式和图像 remap。Stereo 沿用已有词典/标定文件：

```text
ros2 run orbslam3 stereo <vocabulary> <settings> false --ros-args -p comm_mode:=gateway_v3
```

回退（无需网关，不删除 V2 编码/Worker）：

```text
ros2 run orbslam3 stereo <vocabulary> <settings> false --ros-args -p comm_mode:=legacy_v2
```

## 无相机跨进程测试

```bash
ctest --test-dir /home/sunrise/ros2_ws/build/orbslam3 -R gateway_ros_integration -V
```

脚本启动独立 gateway 和 map_producer_test 进程；ROS_DOMAIN_ID=189，localhost-only，
不与默认 domain 的实机通信。测试使用真实共享 MapProducer、真实 V3 Encoder 和 CommMap
Commit 实现，但 batch 是固定输入，**不是 D435i/ORB Atlas 实机验证**。
涵盖13条 ADD=191B、提交前零状态、单 pending、身份四字段拒绝匹配、编码失败不提交。
其余既有 CTest 验证 V2 Golden、Worker 与 Commit 行为。

完整测试输出由 CTest 写入 build/orbslam3/Testing/Temporary/LastTest.log。
本轮不启动相机、不打开任何 UART，不实现 E200、STM32 parser、YOLO 或运动控制。
