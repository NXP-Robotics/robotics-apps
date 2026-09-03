# IO-EXP-GNSS T1 node

Zephyr application for the `mr_mcxn_t1` (MCXN947) T1 node. It is derived from the
`gnss` app and reads the onboard GNSS receiver, encodes each fix as a CDR
`px4_msgs` payload, and publishes it over zenoh-pico into PX4's uORB
(`/fmu/in/...`) for EKF2 fusion.

The only functional difference from the `gnss` app is the bus the F9P receiver
is wired to: this app uses **FlexComm2 LPUART** instead of FlexComm7.

## Sensors / topics published

- **GNSS** -> `sensor_gps` (published at 25 Hz)
- IMU (ICM45686) is streamed on-node; mag and optical-flow egress are disabled
  in this app.

The CDR egress bridge is `drivers/synapse/zenoh_px4`; message encoders live in
`drivers/synapse/px4_msgs`. RTCM3/RTK corrections are out of scope (PX4 injects
those into its own receiver via the mavlink `GPS_RTCM_DATA` path).

## GNSS bus / pinout

The NEO-F9P-15B GNSS is connected to the MCU over **FlexComm2 LPUART**:

- `FC2_P0_PIO4_0` -> RXD
- `FC2_P1_PIO4_1` -> TXD

`flexcomm2_lpspi2` is explicitly disabled so only the LPUART is active on FC2.
The console/shell/mcumgr UART remains on `flexcomm1_lpuart1`.

## Link modes

The node connects to PX4 in one of two topologies, chosen at runtime from the
console (the CDR payload is identical either way):

- **client** - connect to a **NavQ95** running Ubuntu + a zenoh router
  (`rmw_zenohd`); the node shows up in the ROS2 graph.
- **peer** (power-on default) - direct unicast link to a **standalone RT1176
  running PX4** (no router). Here the node is the LISTENER and PX4 connects to
  it. See `../../drivers/synapse/zenoh_px4/PX4_DIRECT.md` for the PX4-side
  connect / subscriber / EKF2 setup.

The mode + locator are runtime parameters (`zenoh_px4 config net peer|client
<ip>:<port>`); `prj.conf` only supplies the power-on
defaults (peer, `tcp/0.0.0.0:7447`). In peer mode the locator is the node's
LOCAL bind address; point PX4 at the node's own static IP `10.58.0.13/16`.

## Build

A single build command; peer/client is selected later from the console.

```bash
west build --sysbuild -b mr_mcxn_t1/mcxn947/cpu0 app/io_exp_gnss -p always
```

Flash with:
```bash
LinkServer flash MCXN947 load --addr 0x10014000 /tmp/zephyr.signed.bin
 ```
