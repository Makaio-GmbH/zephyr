cmake_minimum_required(VERSION 3.8.2)
include($ENV{ZEPHYR_BASE}/cmake/app/boilerplate.cmake NO_POLICY_SCOPE)
project(nrf52_power_mgr)

FILE(GLOB app_sources src/main.c)
target_sources(app PRIVATE ${app_sources})
zephyr_sources_ifdef(CONFIG_PM_CONTROL_APP src/device.c src/power.c)