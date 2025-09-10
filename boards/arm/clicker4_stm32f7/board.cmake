board_runner_args(openocd
  "--cmd-pre-init=source [find interface/cmsis-dap.cfg]"
  "--cmd-pre-init=adapter speed 2000"
  "--cmd-pre-init=transport select swd"
  "--cmd-pre-init=source [find target/stm32f7x.cfg]")
include(${ZEPHYR_BASE}/boards/common/openocd.board.cmake)
