# Teensy 4.0 zenoh-pico publisher (Zephyr)

Minimal demo app that publishes a heartbeat message via zenoh-pico over serial.

## Build

```sh
west build -b teensy40 Projects/teensy40_zenoh_pub --pristine
```

For the custom Clicker 4 STM32F7 board:

```sh
west build -b clicker4_stm32f7 Projects/teensy40_zenoh_pub --pristine -- -DBOARD_ROOT=$PWD
```

For Seeed XIAO ESP32-C3:

```sh
west build -b xiao_esp32c3 Projects/teensy40_zenoh_pub --pristine
```

## Runtime Notes

- Publishes to keyexpr: `demo/teensy40/heartbeat`
- Publish period: `100 ms`
- Zephyr-side serial locator format: `serial/<uart_device_name>#baudrate=<baud>`

## UART selection via overlay

For `teensy40`, the app uses `boards/teensy40.overlay` and binds zenoh serial to
the `zenoh-uart` alias:

- `zenoh-uart = &lpuart4`
- `lpuart4` enabled at `921600` baud
- console/shell stays on `lpuart6` at `921600`

On Teensy 4.0 this maps to UART4 pins configured in Zephyr pinctrl:

- TX: pin 8
- RX: pin 7

If you want UART2 instead (pins 14/15), change `zenoh-uart` to `&lpuart2` and
match router baudrate accordingly.

For `clicker4_stm32f7`, the app uses `boards/clicker4_stm32f7.overlay`:

- `zenoh-uart = &usart2`
- `usart2` set to `230400` baud

For `xiao_esp32c3`, the app uses `boards/xiao_esp32c3.overlay`:

- `zenoh-uart = &uart0`
- `uart0` set to `230400` baud

## Router Example

```sh
zenohd --no-multicast-scouting \
  -l 'tcp/127.0.0.1:7447' \
  -l 'serial//dev/cu.usbserial-0001#baudrate=921600;exclusive=false'
```

## Troubleshooting

If zenoh router logs:

`Failed to accept link before deadline (10000ms)`

after it already received Serial INIT, the serial framing link is up but the
transport handshake did not complete. Check:

1. TX/RX are cross-connected both ways (MCU TX -> adapter RX and MCU RX -> adapter TX).
2. Common ground is connected.
3. UART pins match the selected overlay alias (`lpuart4` => pins 8/7 by default).
4. Router baudrate matches overlay `current-speed`.
