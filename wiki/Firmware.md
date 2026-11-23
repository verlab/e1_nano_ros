# Firmware

CF-Drone by Cherglow (submodule, unmodified) + **E1 Zenoh link** (`firmware/src_e1/zenoh_link.hpp`, zenoh-pico 1.10.1).

| Console command | Effect |
|---|---|
| `zn` | status (connected, id, router, rx/tx/errors, IP) |
| `zn id <n>` | drone id (1..254) → `e1/<n>` |
| `zn connect tcp/<ip>:7447` | router |
| `zn timeout <s>` | command-loss failsafe (default 0.5 s) |
| `zn on` / `zn off` | enables or disables the link |
| `sta <ssid> <password>` + `p WIFI_MODE 2` | WiFi in station mode (required) |

Build: `pio run -e e1_nano` · Flash: `-t upload` · Docker: `docker compose run --rm firmware pio run`.
