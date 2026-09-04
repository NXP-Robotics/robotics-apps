# Copyright 2026 NXP
#
# SPDX-License-Identifier: Apache-2.0

if(DEFINED WEST_TOPDIR)
  set(MICROCDR_DIR "${WEST_TOPDIR}/modules/lib/microcdr")
else()
  set(MICROCDR_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../../../modules/lib/microcdr")
endif()

if(NOT EXISTS "${MICROCDR_DIR}/include/ucdr/microcdr.h")
  message(FATAL_ERROR
    "Micro-CDR sources not found at ${MICROCDR_DIR}. "
    "Run 'west update' after adding the microcdr project to west.yml.")
endif()

zephyr_library_named(px4_microcdr)
file(GLOB MICROCDR_SRCS
  "${MICROCDR_DIR}/src/c/common.c"
  "${MICROCDR_DIR}/src/c/types/*.c"
)
zephyr_library_sources(${MICROCDR_SRCS})
zephyr_library_include_directories(${MICROCDR_DIR}/include)
zephyr_library_include_directories(${CMAKE_CURRENT_SOURCE_DIR}/microcdr_shim/include)
zephyr_include_directories(${MICROCDR_DIR}/include)
zephyr_include_directories(${CMAKE_CURRENT_SOURCE_DIR}/microcdr_shim/include)

zephyr_library_named(px4_msgs)
zephyr_library_sources(
  src/encode_sensor_optical_flow.c
  src/encode_distance_sensor.c
  src/encode_sensor_gps.c
  src/encode_sensor_mag.c
)
zephyr_library_include_directories(include)
zephyr_include_directories(include)
zephyr_library_link_libraries(px4_microcdr)