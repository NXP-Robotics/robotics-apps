/*
 * Copyright 2026 NXP
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#if defined(CONFIG_SPINALI_CORE_COMMON_BOOT_BANNER)
#include <stdio.h>
#endif

#include <spinali/core/common.h>

LOG_MODULE_REGISTER(io_exp_gnss_boot_banner, CONFIG_MCXN_T1_ROS2_IO_EXP_LOG_LEVEL);

static int io_exp_gnss_boot_banner_sys_init(void)
{
	LOG_INF("MCXN-T1 ROS2 IO-EXP %d.%d.%d", CONFIG_SPINALI_VERSION_MAJOR,
		CONFIG_SPINALI_VERSION_MINOR, CONFIG_SPINALI_VERSION_PATCH);
#if defined(CONFIG_SPINALI_CORE_COMMON_BOOT_BANNER)
	printf("%s", banner_spinali);
#endif
	return 0;
};

SYS_INIT(io_exp_gnss_boot_banner_sys_init, APPLICATION, 0);

// vi: ts=4 sw=4 et
