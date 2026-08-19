#-------------------------------------------------------------------------------
# NXP Robotics SDK - ESC Passthrough
#
# Copyright (c) 2025 CogniPilot Foundation
# Copyright 2025 NXP
# SPDX-License-Identifier: Apache-2.0
#
# ESC passthrough for programming AM32/BLHeli ESCs using BLHeli Suite,
# ESC Configurator (esc-configurator.com), am32.ca, etc. Implements the
# MSP + BLHeli 4-way interface + AM32 bootloader protocol over USB CDC-ACM,
# talking to the ESCs over a single-wire half-duplex FlexIO UART.
#
# The FlexIO-UART transport (src/nxp_flexio_uart.c + its devicetree binding in
# dts/bindings/) is carried app-privately: it is NOT a shared Zephyr driver
# module. Per-board FlexIO instance/pins live in boards/<board>.overlay, so the
# same application builds unchanged for every supported board.

cmake_minimum_required(VERSION 3.13.1)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(esc_passthrough LANGUAGES C)

target_compile_options(app PRIVATE -Wall -Wextra -Wno-unused-parameter -Werror)

set(SOURCE_FILES
  src/main.c
  src/nxp_flexio_uart.c
  )

target_sources(app PRIVATE ${SOURCE_FILES})
