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
2. *(Optional)* Create a virtual environment: `python -m venv .venv && source .venv/bin/activate`.
3. Install build utilities: `pip install --upgrade west cmake ninja`.
4. Initialise west in-place (once): `west init -l .`.
5. Fetch modules: `west update`.
6. Export Zephyr CMake packages: `west zephyr-export`.
7. Source the environment when opening a new shell: `. zephyr/zephyr-env.sh`.

## Build
```sh
west build -b clicker4_stm32f7 Projects/svea-lli --pristine
```
Artifacts land in `build/zephyr/` (`zephyr.hex` for flashing, `zephyr.elf` for debugging). Skip `--pristine` for incremental rebuilds or add `-DDTC_OVERLAY_FILE=...` if you introduce additional overlays.

## Flash (CodeGrip GUI)
1. Open CodeGrip Suite and connect to the Clicker 4.
2. Choose *Program* → *Browse* and pick `build/zephyr/zephyr.hex`.
3. Enable *Erase before programming* and *Verify after programming*.
4. Click *Write*. After success, reset or power-cycle the board – it enumerates as USB CDC (`/dev/ttyACM*`).

## ROS Topics
| Topic | Direction | Type | Notes |
| --- | --- | --- | --- |
| `/lli/ctrl/steering` | Subscribe | `std_msgs/msg/Int8` | Desired steering command (−127…127 → 1000–2000 µs).
| `/lli/ctrl/throttle` | Subscribe | `std_msgs/msg/Int8` | Desired throttle command (symmetrical range).
| `/lli/ctrl/high_gear` | Subscribe | `std_msgs/msg/Bool` | High gear request.
| `/lli/ctrl/diff` | Subscribe | `std_msgs/msg/Bool` | Differential lock request (front+rear).
| `/lli/remote/steering` | Publish | `std_msgs/msg/Int8` | RC steer input mirrored to ROS.
| `/lli/remote/throttle` | Publish | `std_msgs/msg/Int8` | RC throttle input.
| `/lli/remote/high_gear` | Publish | `std_msgs/msg/Bool` | RC gear switch state.
| `/lli/remote/override` | Publish | `std_msgs/msg/Bool` | `true` when RC override/mute is active.
| `/lli/remote/connected` | Publish | `std_msgs/msg/Bool` | RC link health flag.
| `/lli/sensor/imu` | Publish | `sensor_msgs/msg/Imu` | ISM330DLC data with covariance.
| `/lli/sensor/encoders` | Publish | `geometry_msgs/msg/TwistWithCovarianceStamped` | Wheel odometry converted to vehicle twist.
| `/lli/sensor/ina3221` | Publish | `std_msgs/msg/Float32MultiArray` | `[V, I, P]` triplets for ESC, 12 V, 5 V rails.
| `/lli/battery/state` | Publish | `sensor_msgs/msg/BatteryState` | Pack voltage, current, SOC, alarms.

The firmware currently exposes no services or actions.

## Hardware Wiring Summary
Use this summary together with `Projects/svea-lli/boards/clicker4_stm32f7.overlay`, the Clicker 4 schematic, and `wiring-guide-by-li-chen.pdf`. Those documents are the authoritative references; the safest approach is to cross-check each net name in the schematic against the overlay before wiring.

### Actuators (servo rail, per overlay comments)
| Function | Overlay node | MCU pin / timer | Board reference | Notes |
| --- | --- | --- | --- | --- |
| Steering servo | `steeringservo` | `PE5 / TIM9_CH1` | Servo header `P3` | 5 V servo rail, PWM 1–2 ms.
| Throttle / ESC | `throttleesc` | `PE6 / TIM9_CH2` | Servo header `P4` | Throttle ramp limiting active.
| Gear actuator | `gearservo` | `PB9 / TIM4_CH4` | Servo header `P2` | 3-position gearbox servo.
| Front diff lock | `diffservo` | `PB5 / TIM3_CH2` | PB5 test pad (see wiring guide) | Route with 5 V/GND per actuator harness.
| Rear diff lock | `diffservorear` | `PA2 / TIM5_CH3` | Servo header `P5` | Shares 5 V servo rail.

### Remote control
| Signal | Overlay alias | MCU pin | Board reference | Notes |
| --- | --- | --- | --- | --- |
| SBUS data | `&usart2` / `sbus0` | `PA3 / USART2_RX` | mikroBUS slot 1 `RX` | Inverted input, powered from 5 V/GND on the same slot.

### Sensors (I²C2 and GPIO)
| Peripheral | Overlay node | MCU pin(s) | Board reference | Notes |
| --- | --- | --- | --- | --- |
| IMU (ISM330DLC) | `ism330dlc@6a` | `PB10` (SCL), `PB11` (SDA), optional `PD0` (INT), `PC15` (RST) | mikroBUS 1 `SCL/SDA/INT/RST` | Enable INT in overlay if you route it.
| INA3221 rail monitor | `ina3221@40` | `PB10`, `PB11` | mikroBUS 1 `SCL/SDA` | Shares bus with IMU, powered from 5 V.
| Wheel encoder – left | `wheel_left_gpios` | `PB2` | Digital header pad for PB2 | Supply from 5 V/GND per sensor datasheet.
| Wheel encoder – right | `wheel_right_gpios` | `PC7` | Digital header pad for PC7 | Debounce handled in firmware.

### Battery management (I²C1 + control signals)
| Signal | Overlay property | MCU pin | Board reference | Notes |
| --- | --- | --- | --- | --- |
| BMS I²C SCL | `&i2c1` | `PB6` | Header labelled MB2_TX / PB6 | Pull-ups reside on BMS board.
| BMS I²C SDA | `&i2c1` | `PB7` | Header labelled MB23_RX / PB7 | — |
| BMS ALERT | `alert-gpios` | `PD6` | Digital header `D6` | Active-high alarm.
| BMS PCHG gate | `pchg-gpios` | `PD2` | Digital header `D2` | Drives BQ76200 pre-charge FET.
| BMS TS1 wake | `ts1_wake_gpios` | `PD5` | Digital header `D5` | Used by `wake_chip.c` to wake the pack.
| SoC button input | `soc_button_gpios` | `PE10` | User button `BTN1` | Optional charge indicator trigger.

For any additional peripherals, match the MCU pins defined in the overlay with the silkscreen or schematic pins before wiring. The `wiring-guide-by-li-chen.pdf` illustrates expected harnessing – refer to it during bring-up.

## Development Tips
- `west build -t menuconfig` to adjust configuration quickly; keep custom overlays under `Projects/svea-lli/boards/`.
- Console logs appear on the USB CDC port at 115200 baud (`minicom -D /dev/ttyACM0 -b 115200`).
- `west build -t run` launches `west flash`/`openocd` with the current build if you switch to a debug probe.
- Recommended VS Code extension: [Zephyr IDE (Mylonics)](https://marketplace.visualstudio.com/items?itemName=mylonics.zephyr-ide) for west tasks, Kconfig/devicetree browsing, and serial consoles.

## micro-ROS Agent Reminder
Start the agent on the host **before** launching ROS nodes:
```sh
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 -b 115200
```
Adjust `/dev/ttyACM*` if your system enumerates differently.

