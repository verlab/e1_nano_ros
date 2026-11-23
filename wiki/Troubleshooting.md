# Troubleshooting

| Symptom | Likely cause |
|---|---|
| `e1-cli list` empty | router not running, wrong IP/port in `zn connect`, WiFi in AP mode (`p WIFI_MODE 2`), firewall blocking 7447/TCP |
| `zn` shows `err` increasing | protocol/zenoh version mismatch: update the firmware and `e1_core` together |
| `ros2 topic list` empty | `RMW_IMPLEMENTATION=rmw_zenoh_cpp` or `rmw_zenohd` missing |
| drone descends on its own mid-flight | `cmd_timeout`: saturated WiFi or stopped controller; see `/e1_<id>/state.cmd_timeout` |
| drift in hover | trims `CTL_TRIM_ROLL/PITCH`, `ca` calibration, propellers swapped |
| slow simulator | `--record` renders on the CPU; use `examples/render_demo.py` (offline) |
