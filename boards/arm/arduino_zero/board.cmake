# Copyright (c) 2017 Google LLC.
# SPDX-License-Identifier: Apache-2.0

#include(${ZEPHYR_BASE}/boards/common/openocd.board.cmake)
#include(${ZEPHYR_BASE}/boards/common/bossac.board.cmake)
board_set_flasher_ifnset(bossac)
board_finalize_runner_args(bossac "--bossac=/usr/local/bin/bossac")