# SVEA Low-Level Interface (Zephyr)

Firmware for the SVEA low-level interface running Zephyr 4.1.0 on the MikroE Clicker 4 for STM32F745VGT6. It bridges vehicle actuators, power management, sensors, and a micro-ROS agent on the host PC.

## Highlights
- Servo control for steering, throttle/ESC, gearbox, and both differential locks with watchdog supervision and throttle slew limiting.
- SBUS input on USART2 with override/mute support and RC state export to ROS.
- micro-ROS publishers for IMU, encoder odometry, INA3221 rail telemetry, battery state, and RC feedback plus subscribers for drive commands.
- TI BQ769x0 battery management integration (alert, wake, SoC indication, peak tracking).
- Devicetree overlays pin out the Clicker 4 peripherals (PWM, I²C, GPIO) used by the application.

## Repository Layout
- `Projects/svea-lli/` – Zephyr application (sources, headers, Kconfig, overlays).
- `modules/` – extra Zephyr modules (micro-ROS transport, vendor HALs, BMS helpers).
- `drivers/` – local driver glue compiled into the app.
- `west-manifest/west.yml` – locks Zephyr v4.1.0 and companion modules.
- `entrypoint` – helper script for container-based workflows (runs `west update` and exports env).

## Prerequisites
- Zephyr SDK 0.16.x (or matching GCC toolchain) and CMake ≥ 3.20.
- Python 3.10+ with `west` installed (`pip install --upgrade west`).
- ROS 2 Iron (or newer) with the `micro_ros_agent` package on the host computer.
- MikroE CodeGrip Suite to flash the Clicker 4 board.

## Setup
1. Clone this repository and `cd` into it.
2. Optional: `python -m venv .venv && source .venv/bin/activate`.
3. `pip install --upgrade west cmake ninja`.
4. `west init -l .` (first time only).
5. `west update`.
6. `west zephyr-export`.
7. `. zephyr/zephyr-env.sh` each new shell.

## Build
```sh
west build -b clicker4_stm32f7 Projects/svea-lli --pristine
```
Artifacts: `build/zephyr/zephyr.hex` (flash), `build/zephyr/zephyr.elf` (debug). Drop `--pristine` for incremental rebuilds.

## Flash (CodeGrip GUI)
1. Open CodeGrip Suite and connect to the Clicker 4.
2. Program → Browse → select `build/zephyr/zephyr.hex`.
3. Enable Erase + Verify; press Write.
4. Reset/power‑cycle; the board appears as USB CDC (`/dev/ttyACM*`).

## ROS Topics
| Topic | Direction | Type | Notes |
| --- | --- | --- | --- |
| `/lli/ctrl/steering` | Subscribe | `std_msgs/msg/Int8` | −127…127 maps to 1000–2000 µs.
| `/lli/ctrl/throttle` | Subscribe | `std_msgs/msg/Int8` | −127…127 maps to 1000–2000 µs (same as steering).
| `/lli/ctrl/high_gear` | Subscribe | `std_msgs/msg/Bool` | High gear request.
| `/lli/ctrl/diff` | Subscribe | `std_msgs/msg/Bool` | Differential lock (front+rear).
| `/lli/remote/steering` | Publish | `std_msgs/msg/Int8` | RC steer mirrored to ROS.
| `/lli/remote/throttle` | Publish | `std_msgs/msg/Int8` | RC throttle mirrored to ROS.
| `/lli/remote/high_gear` | Publish | `std_msgs/msg/Bool` | RC gear switch state.
| `/lli/remote/override` | Publish | `std_msgs/msg/Bool` | `true` when RC override/mute is active.
| `/lli/remote/connected` | Publish | `std_msgs/msg/Bool` | RC link health.
| `/lli/sensor/imu` | Publish | `sensor_msgs/msg/Imu` | ISM330DLC data.
| `/lli/sensor/encoders` | Publish | `geometry_msgs/msg/TwistWithCovarianceStamped` | Wheel odom → twist.
| `/lli/sensor/ina3221` | Publish | `std_msgs/msg/Float32MultiArray` | `[V, I, P]` for ESC, 12 V, 5 V.
| `/lli/battery/state` | Publish | `sensor_msgs/msg/BatteryState` | Pack voltage, current, SOC, flags.

No services/actions are exposed.

## Hardware Wiring Summary
Use this with `Projects/svea-lli/boards/clicker4_stm32f7.overlay`, the official MikroE schematic, and `wiring-guide-by-li-chen.pdf`. Safest is to cross‑reference each overlay pin with the schematic before wiring.

### Actuators (servo rail)
| Function | MCU pin / timer | Board ref | Note |
| --- | --- | --- | --- |
| Steering | `PE5 / TIM9_CH1` | Servo `P3` | 5 V rail.
| Throttle/ESC | `PE6 / TIM9_CH2` | Servo `P4` | Ramp limited.
| Gear | `PB9 / TIM4_CH4` | Servo `P2` | 3‑position.
| Diff front | `PB5 / TIM3_CH2` | PB5 pad | Use servo 5 V/GND.
| Diff rear | `PA2 / TIM5_CH3` | Servo `P5` | 5 V rail.

### Remote control
| Signal | MCU pin | Board ref | Note |
| --- | --- | --- | --- |
| SBUS | `PA3 / USART2_RX` | mikroBUS1 `RX` | Inverted SBUS; power from 5 V/GND.

### Sensors (I²C2 + GPIO)
| Peripheral | MCU pins | Board ref | Note |
| --- | --- | --- | --- |
| ISM330DLC IMU | `PB10` SCL, `PB11` SDA, `PC15` RST, optional `PD0` INT | mikroBUS1 `SCL/SDA/RST/INT` | Enable INT in overlay if wired.
| INA3221 | `PB10` SCL, `PB11` SDA | mikroBUS1 `SCL/SDA` | 5 V supply.
| Wheel enc L | `PB2` | PB2 pad | 5 V/GND supply.
| Wheel enc R | `PC7` | PC7 pad | Debounced in firmware.

### Battery management (I²C1 + control)
| Signal | MCU pin | Board ref | Note |
| --- | --- | --- | --- |
| BMS I²C SCL | `PB6` | `MB2_TX / PB6` | Pull‑ups on BMS.
| BMS I²C SDA | `PB7` | `MB23_RX / PB7` | —
| ALERT | `PD6` | `D6` | Active‑high.
| PCHG gate | `PD2` | `D2` | Drives BQ76200.
| TS1 wake | `PD5` | `D5` | Used by `wake_chip.c`.
| SoC button | `PE10` | `BTN1` | Optional.

For anything else, match overlay pins to the schematic/silkscreen before wiring. The PDF is a quick reference during bring‑up.

## Development Tips
- `west build -t menuconfig` for quick Kconfig tweaks; keep overlays under `Projects/svea-lli/boards/`.
- Console logs: USB CDC at 115200 baud (`minicom -D /dev/ttyACM0 -b 115200`).
- Recommended VS Code extension: Zephyr IDE (Mylonics): https://marketplace.visualstudio.com/items?itemName=mylonics.zephyr-ide

## micro-ROS Agent Reminder
Start the agent on the host before launching ROS nodes:
```sh
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 -b 115200
```
Adjust `/dev/ttyACM*` if your system enumerates differently.
