# Optical Flow T1 node

Zephyr application for the `mr_mcxn_t1` (MCXN947) T1 node. It streams the
onboard optical-flow and ranging sensors, encodes each sample as a CDR
`px4_msgs` payload, and publishes it over zenoh-pico into PX4's uORB
(`/fmu/in/...`) for EKF2 fusion.

## Sensors / topics published

- **Optical flow** — PixArt PAA3905 -> `sensor_optical_flow`
- **Distance** — AFBR-S50 (argus) -> `distance_sensor`
- IMU (ICM45686) is streamed on-node; mag is disabled in this app.

The CDR egress bridge is `drivers/synapse/zenoh_px4`; message encoders live in
`drivers/synapse/px4_msgs`.

## Link modes

The node connects to PX4 in one of two topologies, chosen at runtime from the
console (the CDR payload is identical either way):

- **client** — connect to a **NavQ95** running Ubuntu + a zenoh router
  (`rmw_zenohd`); the node shows up in the ROS2 graph.
- **peer** (power-on default) — direct unicast link to a **standalone RT1176
  running PX4** (no router). Here the node is the LISTENER and PX4 connects to
  it. See `../../drivers/synapse/zenoh_px4/PX4_DIRECT.md` for the PX4-side
  connect / subscriber / EKF2 setup.

The mode + locator are runtime parameters (`zenoh_px4 config net peer|client
<ip>:<port>`); `prj.conf` only supplies the power-on
defaults (peer, `tcp/0.0.0.0:7447`). In peer mode the locator is the node's
LOCAL bind address; point PX4 at the node's own static IP `10.58.0.12/16`.

## Build

A single build command; peer/client is selected later from the console.

```bash
west build --sysbuild -b mr_mcxn_t1/mcxn947/cpu0 app/optical_flow -p always
```

Flash with:
```bash
LinkServer flash MCXN947 load --addr 0x10014000 /tmp/zephyr.signed.bin
```

