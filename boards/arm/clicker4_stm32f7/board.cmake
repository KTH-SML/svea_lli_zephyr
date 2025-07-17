# SPDX-License-Identifier: Apache-2.0

board_runner_args(jlink "--device=STM32F745VGT6" "--speed=4000")

board_runner_args(openocd
  --cmd-pre-init "source [find interface/stlink.cfg]"
  --cmd-pre-init "adapter speed 1800"
  --cmd-pre-init "source [find target/stm32f7x.cfg]"
  --cmd-pre-init "adapter srst delay 100"
)

# board_runner_args(openocd --cmd-pre-init "source [find interface/stlink.cfg]")
# board_runner_args(openocd --cmd-pre-init "source [find target/stm32f7x.cfg]")

include(${ZEPHYR_BASE}/boards/common/openocd.board.cmake)

