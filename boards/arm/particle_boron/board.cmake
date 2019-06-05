#
# Copyright (c) 2018, Peter Bigot Consulting, LLC
#
# SPDX-License-Identifier: Apache-2.0

board_runner_args(nrfjprog "--nrf-family=NRF52")
board_runner_args(nrfjprog "--softreset")

include(${ZEPHYR_BASE}/boards/common/nrfjprog.board.cmake)
