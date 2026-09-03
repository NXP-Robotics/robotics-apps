# SPDX-License-Identifier: Apache-2.0

add_subdirectory_ifdef(CONFIG_SPINALI_SYNAPSE_TOPIC topic)
add_subdirectory_ifdef(CONFIG_PX4_MSGS px4_msgs)
add_subdirectory_ifdef(CONFIG_SYNAPSE_ZENOH_PX4 zenoh_px4)
