# Direct T1-node -> standalone RT1176/PX4 link (peer mode)

The `zenoh_px4` bridge encodes each ZROS sensor sample as a CDR `px4_msgs`
payload and publishes it on `<domain>/fmu/in/<topic>/...` over zenoh-pico. It
supports two link topologies, selected at runtime from the console (the CDR
wire payload is byte-identical in both):


- **Client (router path)** — connects to a NavQ95 running Ubuntu + a zenoh
  router (`rmw_zenohd`). The node appears in the ROS2 graph and PX4 (or any
  ROS2 consumer) reaches it through the router.
- **Peer (direct path)** — a plain zenoh-pico unicast link straight to a
  standalone RT1176 running PX4's embedded zenoh module. No router, no ROS2
  graph discovery; only the CDR payload matters. In this topology **the T1
  node is the LISTENER and PX4 is the CONNECTOR** (see "Why the node listens"
  below).

This document covers the **peer / direct** path.

## Build

There is a single build command per node:

```bash
# Optical Flow
west build -b mr_mcxn_t1/mcxn947/cpu0 app/optical_flow

# GNSS
west build -b mr_mcxn_t1/mcxn947/cpu0 app/gnss
```

The relevant `prj.conf` power-on defaults are:

| Kconfig | Value | Why |
| --- | --- | --- |
| `..._MODE_PEER` | `y` | default topology: plain unicast link, no router |
| `..._LOCATOR` | `tcp/0.0.0.0:7447` | the LOCAL address the node LISTENs on |
| `..._RMW_COMPAT` | `y` | compiled in; emitted only in CLIENT mode (see note) |
| `..._RIHS_HASH` | `y` | must match PX4's `ZENOH_KEY_TYPE_HASH` build option |

`..._RMW_COMPAT` is compiled in unconditionally but the rmw_zenoh liveliness
tokens and per-sample attachments are emitted at runtime only when the session
is in CLIENT mode (router path). In PEER mode PX4 ignores them, so they are
suppressed to save bandwidth. There is nothing to toggle at build time.

## Runtime net config

The mode + locator are runtime parameters set from the shell, mirroring PX4's
own `zenoh config net peer|client <ip>:<port>`. The `..._MODE_*` choice and
`..._LOCATOR` value in `prj.conf` only supply the power-on defaults.


```
uart:~$ zenoh_px4 config show
mode:    peer
locator: tcp/0.0.0.0:7447
running: 1

# Direct peer path: node LISTENs on a LOCAL bind address
uart:~$ zenoh_px4 config net peer 0.0.0.0:7447
staged peer tcp/0.0.0.0:7447 (run `zenoh_px4 restart` to apply)

# Router path: node CONNECTs to a remote zenoh router
uart:~$ zenoh_px4 config net client 10.58.0.1:7447
staged client tcp/10.58.0.1:7447 (run `zenoh_px4 restart` to apply)

# Apply (explicit; changes are NOT live)
uart:~$ zenoh_px4 restart
restarted (client tcp/10.58.0.1:7447)
```

Changes live in RAM only: they are NOT persisted, so a `config net` override is
lost on reboot and the build defaults are restored. They are staged only; the
session is reopened on the explicit `restart`.

**Semantic caveat (peer mode).** Unlike PX4, where `net peer <ip>:<port>` is the
REMOTE connect target, here peer mode makes the node the LISTENER, so the
`<ip>:<port>` is this node's LOCAL bind/listen address (e.g. `0.0.0.0:7447` or
the node's own `10.58.0.12:7447`). `net client <ip>:<port>` matches PX4: it is
the remote router address the node connects to. The command composes
`tcp/<ip>:<port>` for you.

Enabling `..._RUNTIME_CONFIG` only adds the shell commands and the RAM-resident
config; it does not touch flash and requires no `storage_partition`, so the
stock MCUboot slot geometry is unchanged.


## Why the node listens

PX4's zenoh module glue (`ZENOH::setupSession()`) only ever inserts the zenoh
`MODE` and `CONNECT` config keys; it never inserts `LISTEN`. So a stock PX4
can only be a *connector* in peer mode -- it cannot open a listening peer
transport. To make a direct peer link work without patching PX4, the roles are
reversed from the usual embedded pattern: **the T1 node is the LISTENER and PX4
connects to it.**

zenoh-pico supports this on the node side via `Z_FEATURE_UNICAST_PEER` plus the
`LISTEN` config key (both enabled by default in the pinned zenoh-pico). The
driver inserts `Z_CONFIG_LISTEN_KEY` (instead of `CONNECT`) whenever
`..._MODE_PEER` is set, binding to `..._LOCATOR`.

## Network

The T1 nodes use static IPs on the `10.58.0.0/16` subnet:

- optical_flow: `10.58.0.12`
- gnss: `10.58.0.13`

The node LISTENs on `..._LOCATOR`. Leave it at `tcp/0.0.0.0:7447` to accept on
all interfaces, or pin it to the node's own address (`tcp/10.58.0.12:7447` for
optical_flow, `tcp/10.58.0.13:7447` for gnss) to bind a single interface.

Give the RT1176 T1 port an address on the same subnet and point PX4's peer
CONNECT locator at the node's address (`tcp/<node_ip>:7447`).

## PX4 / RT1176 side (not in this repo)

The T1 node is the listener; **PX4 must be the connector.**

1. **zenoh module: connect to the node.** Configure PX4's zenoh module to
   connect (peer mode) to the node's T1 address, e.g.
   `tcp/10.58.0.12:7447` (optical_flow) or `tcp/10.58.0.13:7447` (gnss), so
   PX4's `z_open`/CONNECT reaches the node's listener. Match the transport
   (TCP) and batch/mtu params.

2. **Match the key parameters.**
   - `ZENOH_DOMAIN_ID` == `CONFIG_SYNAPSE_ZENOH_PX4_DOMAIN_ID` (both
     default 0).
   - `ZENOH_KEY_TYPE_HASH` == `CONFIG_SYNAPSE_ZENOH_PX4_RIHS_HASH`
     (both default `y`). A mismatch on either silently breaks the key match.

3. **Declare the CDR subscribers** (runtime, per topic):
   ```
   zenoh config add subscriber /fmu/in/sensor_optical_flow sensor_optical_flow
   zenoh config add subscriber /fmu/in/distance_sensor     distance_sensor
   zenoh config add subscriber /fmu/in/sensor_gps          sensor_gps
   zenoh config add subscriber /fmu/in/sensor_mag          sensor_mag
   ```
   (Add only the topics the chosen app publishes.)

4. **dds_topics.yaml.** `sensor_optical_flow` and `distance_sensor` ship stock
   (no PX4 rebuild). `sensor_gps` / `sensor_mag` must be added under
   `subscriptions:` and PX4 rebuilt — see `../px4_msgs/README.md`.

5. **EKF2 parameters** for fusion (optical flow, range, GNSS, mag) — see the
   PX4-side runtime steps in `../px4_msgs/README.md`.

## Bring-up tip

For an initial ROS-free bring-up, set `CONFIG_SYNAPSE_ZENOH_PX4_RIHS_HASH=n`
on the node **and** `ZENOH_KEY_TYPE_HASH=n` on PX4, so keys are just
`<ns>/fmu/in/<topic>` with no hash segment to match. Turn both back on once the
link is confirmed.
