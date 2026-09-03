# Copyright 2026 NXP
#
# SPDX-License-Identifier: Apache-2.0

zephyr_library_named(synapse_zenoh_px4)
zephyr_library_sources(src/main.c)
zephyr_library_link_libraries(px4_msgs)