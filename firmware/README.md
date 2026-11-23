# E1 Nano firmware — CF-Drone + E1 Zenoh link

The flight firmware is the original **CF-Drone** (Cherglow, `external/CF-Drone`, a git submodule pinned
at commit `83ebe61`), with no changes to its source code. The `src_e1/` folder adds the **E1 Zenoh link**
(zenoh-pico 1.10.1), which places the drone directly on the Zenoh network used by ROS 2
(`rmw_zenoh_cpp`). The script `tools/prepare_src.py` copies CF-Drone into `.e1_src/`, adds
`src_e1/` and inserts five one-line hooks (each located by an anchor and verified):

| file           | hook                       | purpose                                                   |
|----------------|----------------------------|-----------------------------------------------------------|
| `CF-Drone.ino` | `setupZenohLink()`         | starts the Zenoh task on core 0                           |
| `CF-Drone.ino` | `zenohLinkApply()`         | applies received commands (before `estimate/control`)     |
| `CF-Drone.ino` | `zenohLinkSnapshot()`      | copies the telemetry (after `sendMotors`)                 |
| `cli.ino`      | `zenohLinkCommand(...)`    | `zn ...` console commands                                 |
| `cli.ino`      | `zenohLinkLog(buf)`        | mirrors the console to `e1/<id>/log`                      |

## Build and flash

```bash
git submodule update --init --recursive
cd firmware
pio run -e e1_nano                 # ESP32-S3 SuperMini (E1 Nano v1.5Y)
pio run -e e1_nano -t upload       # hold BOOT while plugging in USB the first time
pio device monitor
```

Without PlatformIO installed: `docker compose run --rm firmware pio run` (see `docker/Dockerfile.firmware`).
zenoh-pico invokes `cmake` during the build (already included in the Docker image).

## Verification without an ESP32

`test/host_check/run.sh` compiles `zenoh_link.ino` on the PC against the real zenoh-pico (Unix platform) and
CF-Drone headers, with minimal Arduino/FreeRTOS stubs. It catches API errors without the ESP32 toolchain.

## Configuration (serial console, Web RC or `e1-cli console`)

```text
sta MyNetwork MyPassword        # lab WiFi (station mode)
p WIFI_MODE 2                   # 2 = STA (in the code: W_AP=1, W_STA=2)
zn id 3                         # this drone becomes e1/3
zn connect tcp/192.168.1.100:7447   # IP of the PC running the Zenoh router (rmw_zenohd)
zn timeout 0.5                  # failsafe: no AUTO commands for 0.5 s -> descend()
reboot
zn                              # status: connected, rx/tx/errors, IP
```

Before the first flight, run the CF-Drone IMU calibration (`ca`, then `ps`), as described on the project page.

## Behavior

* **Cores:** the CF-Drone flight loop stays on core 1. Zenoh runs in a FreeRTOS task on core 0
  and exchanges data with the loop through a spinlock-protected mailbox. The network never blocks the control loop.
* **Telemetry:** `e1/<id>/state` at 50 Hz (76 bytes). The console is mirrored to `e1/<id>/log`. Discovery
  uses the liveliness token `e1/<id>/alive`.
* **Commands:** `e1/<id>/cmd/{attitude,manual,motors,arm,mode,param,console}` and the same under `e1/all/cmd/*` (broadcast).
  `cmd/attitude` has the same semantics as the CF-Drone MAVLink `SET_ATTITUDE_TARGET`, already in FLU.
* **Failsafes:** all CF-Drone failsafes remain active (battery, inverted, RC loss). In addition, in AUTO, if the
  command stream stops for `ZN_TIMEOUT`, the drone calls `descend()`. Moving the sticks (RC/Web RC) takes the drone out of AUTO.
* **Reconnection:** if WiFi or the router goes down, the task reopens the session automatically.

## License

CF-Drone is GPL-3.0 with an additional commercial-restriction clause from its author (see `external/CF-Drone/LICENSE`).
The combined firmware (`src_e1/` + CF-Drone) is subject to those terms: academic, non-commercial use. The rest
of the repository (ROS 2 packages, simulator and documentation) is Apache-2.0.
