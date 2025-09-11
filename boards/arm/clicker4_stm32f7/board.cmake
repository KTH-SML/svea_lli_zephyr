board_runner_args(openocd
  "--cmd-pre-init=source [find interface/cmsis-dap.cfg]"
  "--cmd-pre-init=transport select swd"
  "--cmd-pre-init=adapter speed 400"                 # be conservative on first attach
  "--cmd-pre-init=reset_config srst_only srst_nogate connect_assert_srst"
  "--cmd-pre-init=source [find target/stm32f7x.cfg]"
  "--cmd-pre-init=cortex_m reset_config sysresetreq" # use SYSRESETREQ for init
)
include(${ZEPHYR_BASE}/boards/common/openocd.board.cmake)
