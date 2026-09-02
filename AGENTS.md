# Mine Visual Robot Project

## Project structure

- `ros2/`: ROS2 and ORB-SLAM3 nodes running on RDK S100.
- `stm32/`: STM32 firmware and Keil project.
- `protocol/`: Shared communication protocol.

## Local Windows task boundary

This Windows/local Codex task owns only the STM32 implementation.

1. Read `ros2/` and `protocol/` only to understand existing behavior, packet fields, tests, and requirements.
2. Modify files only under `stm32/` unless the user explicitly authorizes a broader scope in the current conversation.
3. Do not add, edit, delete, format, restore, stage, or commit files under `ros2/`.
4. Do not implement the S100/ROS2 side locally. Describe required ROS2 changes for the separate remote S100 task instead.
5. Treat `protocol/` as read-only. If the STM32 work requires a protocol change, stop that part and report the proposed change for the remote/shared-protocol task.
6. Never make a local ROS2 change merely to make an STM32 test pass.
7. Before reporting completion, run `git diff --name-only -- ros2 protocol` and confirm that this task introduced no changes there. Do not discard pre-existing user changes.
8. Stage or commit only intended `stm32/` files. Keep IDE state, generated build files, `build/`, `install/`, and `log/` out of commits.

## STM32 implementation rules

- Preserve the existing V3 frame layout, version, session, sequence, routing fields, payload length, and CRC unless an explicitly approved shared-protocol change says otherwise.
- Keep interrupt and DMA callbacks short. Perform parsing and business logic in main-loop context.
- Prefer small functions, clear names, explicit byte order, bounds checks, and comments that explain protocol or concurrency constraints.
- Verify STM32 changes with host-side protocol tests and a Keil rebuild when available.
- Clearly separate software verification from real S100-to-STM32 hardware verification.

## Hardware

- S100: Ubuntu 22.04, ROS2 Humble.
- STM32: STM32F103C8T6, USART communication.
