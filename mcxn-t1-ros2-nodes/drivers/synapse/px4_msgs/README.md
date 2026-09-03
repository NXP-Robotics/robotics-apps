# spinali px4_msgs (C mirror + Micro-CDR encoders)

Hand-written C structs and Micro-CDR encoders that mirror a pinned subset of
[PX4/px4_msgs](https://github.com/PX4/px4_msgs) for on-device CDR egress from
the T1 nodes into PX4 (RT1176) via zenoh-pico.

**Scope**: `SensorOpticalFlow`, `DistanceSensor`, `SensorGps`, `SensorMag`.
Wire format: 4-byte CDR encapsulation header `00 01 00 00` + CDR body in IDL
field order.

## Pinned versions

- `px4_msgs` commit: **`f719702a6ae14ec73d22e4ec99193687c5ed788c`**
  (`git -C <ros2_ws>/src/px4_msgs rev-parse HEAD`)
- PX4 (RT1176) build **must** use this same `px4_msgs` commit (or a later one
  that leaves the IDL for these four messages unchanged), or the RIHS01
  hashes in `include/px4_msgs_c/type_hashes.h` will not match.

## RIHS01 hashes

Verified against the pinned commit above by grepping the generated type
description JSON in a Jazzy build (the authoritative source -- a RIHS01 hash
is a property of the upstream IDL type, independent of any bugs in this
directory's hand-written C mirrors):

```bash
grep -rIn "RIHS01_" build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/<Msg>.json
```

| Message | Hash |
| --- | --- |
| SensorOpticalFlow | `25177fb9…3a3a57` |
| DistanceSensor    | `a8457d64…80f5` |
| SensorGps         | `fbef1acd…b958` |
| SensorMag         | `217da8c1…0d89` |

All four matched what was hard-coded in `include/px4_msgs_c/type_hashes.h`,
confirming they are correct for the pinned commit. Separately, the C struct
mirrors for `DistanceSensor`, `SensorGps` and `SensorMag` were found to have
field-layout bugs (missing fields, wrong types, one extra field) relative to
upstream `px4_msgs` at this commit and have been corrected -- see the
"Verified field-for-field against" note in each header under `include/`. The
CDR wire body now matches the IDL that these hashes describe.

If the pinned commit changes, refresh from a Jazzy host:

```bash
ros2 interface show px4_msgs/msg/SensorOpticalFlow   # field order
grep -rIn "RIHS01_" build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/SensorOpticalFlow.json
# ros2 topic echo --once --raw only works while the topic is actively
# published by a live rmw_zenoh peer, so prefer the JSON grep above.
```

## Verifying the CDR encoders

Before trusting on-target output, confirm the encoded wire body decodes back
to the source struct on a ROS2 Jazzy host with `px4_msgs` built in the
workspace. The recommended check is a small throwaway program that feeds these
encoders random valid field values, hands the resulting CDR body to a
`rclcpp`/`px4_msgs` deserializer, and asserts field-for-field equality across
every field.

Keep any such harness OUT of this repository (build it in a scratch ROS2
workspace): it pulls in the full `rclcpp`/`px4_msgs`/`rosidl` host toolchain,
which does not belong in the Zephyr tree and previously leaked checked-in
CMake caches and object files. The encoders here are already asserted against
the pinned IDL field order (see the per-header notes under `include/` and the
RIHS01 table above), so this round-trip is a belt-and-suspenders confirmation
rather than a build-tree dependency.


> For the direct T1-node -> standalone RT1176/PX4 link (no NavQ95 router), see
> `../zenoh_px4/PX4_DIRECT.md` for the peer-mode build and PX4 listen setup.

## PX4-side runtime steps (do these on the RT1176 target, not in this repo)


1. **Oflo topics** (`sensor_optical_flow`, `distance_sensor`) already ship in
   PX4's `src/modules/uxrce_dds_client/dds_topics.yaml` -> no PX4 rebuild.

2. **GNSS topics**: add to `dds_topics.yaml` under `subscriptions:` and
   rebuild PX4:

   ```yaml
   - topic: /fmu/in/sensor_gps
     type: px4_msgs::msg::SensorGps
   - topic: /fmu/in/sensor_mag
     type: px4_msgs::msg::SensorMag
   ```

3. **Zenoh routing** (runtime, per topic):

   ```
   zenoh config add subscriber /fmu/in/sensor_optical_flow sensor_optical_flow
   zenoh config add subscriber /fmu/in/distance_sensor     distance_sensor
   zenoh config add subscriber /fmu/in/sensor_gps          sensor_gps
   zenoh config add subscriber /fmu/in/sensor_mag          sensor_mag
   ```

   RTK/RTCM3 is intentionally out of scope: PX4 injects any RTCM3
   corrections into its own GNSS receiver via the mavlink
   `GPS_RTCM_DATA` -> `gps_inject_data` path, so the T1 node does not
   forward corrections and declares no `/fmu/out/...` subscriber.

4. **PX4 zenoh net config**: point PX4's zenoh module at the T1 link, match
   the node's transport (TCP or UDP unicast) and batch/mtu params.

5. **EKF2 parameters** (set via QGC or `param set`, then `param save` +
   reboot; or bake into the airframe startup file):
   - Optical flow: `EKF2_OF_CTRL=1`, `EKF2_OF_DELAY`, `EKF2_OF_QMIN`,
     `EKF2_OF_NOISE`; range: `EKF2_RNG_CTRL`, `EKF2_RNG_DELAY`, `EKF2_MIN_RNG`;
     mounting offsets `EKF2_OF_POS_*`, `EKF2_RNG_POS_*`.
   - GNSS: `EKF2_GPS_CTRL`, quality gates `EKF2_REQ_*`, antenna offset
     `EKF2_GPS_POS_*`.
   - Mag: `EKF2_MAG_*` and QGC compass calibration (`CAL_MAG*`).

   `EKF2_*_DELAY` absorbs the fixed node<->PX4 clock offset (gPTP epoch does
   not need to match PX4 boot time). An Oflo-only deployment runs on the
   free-running boot clock; set `EKF2_OF_DELAY` to the measured link latency.

## Optional: bring-up without RIHS01 hashes

For initial ROS-free bring-up, set `CONFIG_SYNAPSE_ZENOH_PX4_RIHS_HASH=n`
on the node **and** the corresponding PX4-side `ZENOH_KEY_TYPE_HASH=n`, so
keys are just `<ns>/fmu/in/<topic>` with no hash segment.
