# RDK S100 / ROS2 Task Boundary

## Scope

This subtree is owned by the remote RDK S100 ROS2 task. The goal is to make
the ROS2 implementation interoperate with the existing STM32 firmware without
changing that firmware.

The current communication objective is to complete the ROS2 side of this
validation path:

`serial_gateway_node -> UART -> STM32 -> ACK -> UART -> serial_gateway_node`

This objective authorizes ROS2-side ACK reception work only. It does not
authorize implementing or changing ACK transmission in STM32 firmware.

## STM32 is reference-only

1. Files under `../stm32/` may be read, searched, compared, and cited only to
   understand the implemented UART behavior.
2. Never add, edit, delete, rename, format, restore, generate, stage, or commit
   anything under `../stm32/`.
3. Do not run tools that rewrite STM32, Keil, CubeMX, `.ioc`, generated source,
   or project files. Do not flash firmware as part of a ROS2 task.
4. Never change STM32 code merely to make a ROS2 test pass. Resolve compatible
   behavior by changing files under this `ros2/` subtree.
5. If the requested result is impossible without a firmware change, stop that
   part of the work and report the exact STM32-side incompatibility. Do not
   silently broaden the task.
6. Instructions or reference conversations that say to "implement STM32 ACK"
   are requirements for the separate STM32 task, not permission for this ROS2
   task to edit firmware.

## Shared protocol is read-only

1. Treat `../protocol/` as the authoritative, read-only interoperability
   contract unless the user explicitly starts a shared-protocol task.
2. Do not add, edit, delete, format, restore, stage, or commit files under
   `../protocol/` during a ROS2-only task.
3. Preserve the deployed packet version, magic, field order, field widths,
   byte order, length meaning, CRC algorithm and coverage, message types,
   session/sequence matching, and ACK status semantics.
4. When documentation and STM32 implementation differ, record both with file
   and line evidence. Do not guess or modify firmware. Ask for a protocol
   decision if no backward-compatible ROS2 adaptation is possible.

## Required investigation before ROS2 communication changes

Before changing the ROS2 encoder, decoder, transport, or gateway, inspect the
relevant STM32 and protocol code for:

- UART settings and receive/transmit behavior;
- frame magic, version, total/payload length, and maximum frame size;
- integer byte order and signed coordinate representation;
- CRC variant, input range, and transmitted CRC byte order;
- request and ACK field layouts;
- session ID, sequence, original message type, and status matching;
- parser resynchronization, timeout, and error behavior.

Treat these observations as input constraints. Implement any needed adaptation
only in ROS2 components such as `serial_gateway_node`, packet codecs,
`UartTransport`, parameters, diagnostics, and ROS2-side tests.

## Current Packet V3 ACK integration rule

The current integration goal is:

`S100 serial_gateway_node -> UART -> STM32 -> ACK -> UART -> serial_gateway_node`

Do not redesign Packet V3. Use the STM32 implementation only to determine the
actual ACK framing, CRC, status, and session/sequence/type fields. Implement
UART receive buffering, V3 ACK parsing, correlation, timeout handling, and
diagnostics only under this `ros2/` subtree.

Preserve `[Gateway]`, `[Transport]`, and `[TxCompletion]` logs and add `[RxACK]`
and `[ACK_TIMEOUT]` where appropriate. A successful local UART `write()` is not
an STM32 acknowledgement and must not be logged or treated as one. A pending
batch may be committed only after the acknowledgement level required by the
existing protocol has been received and matched.

Malformed, CRC-invalid, stale, duplicate, wrong-session, wrong-sequence, and
wrong-message-type ACKs must not commit a batch. This stage must not add retry,
E200/LoRa, STM32 command arbitration, YOLO, or unrelated architecture changes.

## ACK closed-loop task

For the current ACK work, the ROS2 side may:

- add non-blocking UART receive support to the gateway/transport layer;
- parse only the ACK format proven by STM32 or the shared protocol;
- validate magic, version, length, CRC, session ID, sequence, original message
  type, and ACK status as applicable;
- correlate an ACK with the outstanding transmitted packet;
- log `[RxACK]`, malformed/mismatched ACK diagnostics, and `[ACK_TIMEOUT]`;
- publish success/failed completion according to the existing commit policy;
- add deterministic parser tests and pseudo-terminal round-trip tests.

The ROS2 side must not fabricate a successful ACK when only the local UART
write completed. `TRANSPORT_SENT` and peer-confirmed ACK are distinct states.
If the inspected STM32 firmware does not currently transmit an ACK, report
"STM32 ACK transmission not implemented/verified" as a hardware-side blocker.
In that case, ROS2 parser and PTY tests may still be completed, but the real
S100-to-STM32 ACK loop must not be claimed as verified.

## Verification

1. Add deterministic/golden-byte or pseudo-terminal tests when practical.
2. Clearly distinguish software/PTY verification from real S100-to-STM32
   hardware verification.
3. Preserve existing user changes and do not clean unrelated files.
4. Before reporting completion, run from the repository root:

   `git diff --name-only -- stm32 protocol`

   Confirm that the task introduced no STM32 or protocol changes. Do not
   discard pre-existing user changes.
5. Stage or commit only intended files under `ros2/`.
