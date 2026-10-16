# Lyrical integration evidence

> **Historical micro-ROS implementation:** the active transport is now direct
> USB serial. See [current serial validation](../../../docs/DIRECT_SERIAL_VALIDATION.md). The results below
> describe the earlier micro-ROS path and do not validate the serial replacement.

Inspected upstream source on the user's session date, **2026-10-16**. Version
compatibility is established at source/dependency level, by the executed
Cortex-M4 cross-build, and by live communication between this STM32 and the
matching Lyrical Agent on the laptop. The native Lyrical interface package built
successfully and all three custom message type hashes match the embedded
generation. Pi-to-STM32 deployment remains untested.

| Component | Selected source |
|---|---|
| Official CubeMX utilities, rolling integration glue | `3eb71ecebd57c166f0f4367cc2b0b1c4fe4b3234` |
| Official `micro_ros_setup`, Lyrical dependency manifest | `e64204663fb734e24b5de3cd04fb03d3188468d9` |
| Official micro-ROS Agent, Lyrical | `8cf77415e64a9a95ace719f14a020b7a0745e19e` |
| Micro XRCE-DDS client and Agent | Both **v3.0.2** |
| Micro-CDR | **v2.0.2**, from the Lyrical client manifest |
| STM32 HAL/device/CMSIS/FreeRTOS/USB | CubeF4 **1.28.1** component revisions in `dependencies.json` |
| GNU Arm compiler actually executed | **14.2.1**, Cortex-M4/Thumb, `fpv4-sp-d16`, hard float |

The official CubeMX utility repository has foxy through kilted and rolling
branches, but no Lyrical branch at this audit. Its releases page has no releases.
The Lyrical static-library-builder Docker tag query returned HTTP 404. Therefore
no Humble/Jazzy/Kilted binary or an unverified “Lyrical CubeMX release” is used.
The pinned utility glue is paired with the official Lyrical source manifest and
built locally. Rolling here identifies the platform utility commit, **not** the
ROS client package distribution: every client repository is pinned in
`micro_ros/client.repos`.

Primary evidence:

- [Official CubeMX integration instructions](https://github.com/micro-ROS/micro_ros_stm32cubemx_utils/blob/3eb71ecebd57c166f0f4367cc2b0b1c4fe4b3234/README.md)
  specify static library linkage, custom HAL transport, allocator/time support
  and more than 10 KiB of micro-ROS task stack.
- [Official Lyrical client manifest](https://github.com/micro-ROS/micro_ros_setup/blob/e64204663fb734e24b5de3cd04fb03d3188468d9/config/client_uros_packages.repos)
  selects Micro XRCE-DDS Client 3.0.2 and the Lyrical micro-ROS packages.
- [Official Lyrical Agent SuperBuild](https://github.com/micro-ROS/micro-ROS-Agent/blob/8cf77415e64a9a95ace719f14a020b7a0745e19e/micro_ros_agent/cmake/SuperBuild.cmake)
  requires Micro XRCE-DDS Agent 3.0.2 and links the host Fast DDS/Fast CDR.

The source lock also preserves the Lyrical setup's temporary pre-rosidl-buffer
pin (`ros2/rosidl` `5f4ace0288ecf942307ed62b9239ab5986884676`) and its temporary
eProsima Lyrical rclc fork. Those are explicit upstream integration decisions,
not accidental references to an older ROS client distribution.

## Adaptations checked into this project

- Lyrical's `RCL_MICROROS=ON` configuration replaces obsolete RCL switches in
  the older utility metadata. XRCE custom stream framing is enabled explicitly.
- The XRCE 3 transport write callback takes `const uint8_t *`; the application
  uses that signature. The older utility transport callback does not compile
  unchanged against XRCE 3 with current GCC.
- The official FreeRTOS time implementation is staged with `clockid_t` in its
  `clock_gettime` signature to match newlib 4.6. The syscall time fallback uses
  this same RTOS monotonic clock; epoch conversion comes from Agent time sync.
- The official custom heap is staged with a separate 48 KiB micro-ROS heap and
  explicit fixes for calloc failure/overflow, malloc size overflow and
  realloc(pointer, 0). The FreeRTOS kernel heap is 8 KiB.
- USB CDC follows the official custom-transport approach with bounded static
  TX storage, overflow detection, bounded waits and an ISR/task RX ring.
  The unmodified utility USB ring can overwrite unread data and its TX buffer
  lifetime is insufficient after a timeout; that transport is not compiled.
- The native builder filters host DDS/type-support discovery while retaining
  native Python/CMake generators. Firmware libraries are all ARM archives.
  The project uses MRI archive assembly to preserve duplicate object names.

The full client build completed with 45 packages. The matching native Lyrical
Agent also compiled on the laptop with XRCE Agent 3.0.2 and bundled spdlog
1.9.2 (the host system-logger override fails with the installed fmt headers).
The dependency download was made shallow without changing its pinned tag.
The first live attempt exposed that newlib `srand()` allocates 24 bytes of
internal state during RMW initialization; the initial rejecting `_sbrk` stub
caused an assert/reset. The updated firmware supplies a bounded, separately
reserved 1 KiB libc heap. After reflashing, the board established the session, created all four publishers
and the subscription, and delivered 99 status frames in five seconds. One Agent
restart recovered telemetry (144 frames in eight seconds), without resetting
the MCU, arming, changing zero duty, or reducing reported free micro-ROS heap. Upstream warnings include
unused parameters, zero-sized disabled service/client pools, and an unused
action-client initializer; no actions/services are exposed by this application.
None of this is a claim that the board, transport timings, Pi installation or
physical actuator behavior have passed hardware validation.
