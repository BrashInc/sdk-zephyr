# Kconfig - nRF91 ORION board configuration
#
# Copyright (c) 2019 Nordic Semiconductor ASA
#
# SPDX-License-Identifier: LicenseRef-BSD-5-Clause-Nordic

target_include_directories(app PRIVATE .)

if(CONFIG_BOARD_ORION_NRF9160 AND NOT DEFINED CONFIG_MCUBOOT)
target_sources(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/board_secure.c)
endif()

if(CONFIG_BOARD_ORION_NRF9160NS)
target_sources(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/board_nonsecure.c)
endif()
