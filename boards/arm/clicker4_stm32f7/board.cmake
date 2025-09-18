board_runner_args(pyocd
  "--target=stm32f745vg"
  "--frequency=10000"
  "--flash-opt=-O connect_mode=under-reset"
)

include(${ZEPHYR_BASE}/boards/common/pyocd.board.cmake)
include(${ZEPHYR_BASE}/boards/common/openocd.board.cmake)
