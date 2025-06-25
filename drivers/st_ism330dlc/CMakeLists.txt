## SVEA Note.
##
## This is a copy/rewrite from the ISM330DHCX driver that exist in Zephyr tree.
## 
## Path: 
## zephyr/drivers/sensor/st/ism330dhcx
## 
## Author:
## Kaj Munhoz Arfvidsson


# ST Microelectronics ISM330DLC 6-axis IMU sensor driver
#
# Copyright (c) 2020 STMicroelectronics
#
# SPDX-License-Identifier: Apache-2.0
#
zephyr_library()

zephyr_library_sources(ism330dlc.c)
zephyr_library_sources(ism330dlc_i2c.c)
zephyr_library_sources(ism330dlc_spi.c)
zephyr_library_sources_ifdef(CONFIG_ISM330DLC_SENSORHUB  ism330dlc_shub.c)
zephyr_library_sources_ifdef(CONFIG_ISM330DLC_TRIGGER    ism330dlc_trigger.c)

zephyr_library_include_directories(${ZEPHYR_BASE}/drivers/sensor/st/stmemsc)
