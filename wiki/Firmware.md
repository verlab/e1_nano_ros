# Firmware

CF-Drone (submódulo, sem modificações) + **E1 Zenoh link** (`firmware/src_e1/zenoh_link.ino`, zenoh-pico 1.10.1).

| Comando de console | Efeito |
|---|---|
| `zn` | status (conectado, id, roteador, rx/tx/erros, IP) |
| `zn id <n>` | id do drone (1..254) → `e1/<n>` |
| `zn connect tcp/<ip>:7447` | roteador |
| `zn timeout <s>` | failsafe de perda de comandos (padrão 0,5 s) |
| `zn on` / `zn off` | liga ou desliga o link |
| `sta <ssid> <senha>` + `p WIFI_MODE 2` | WiFi em modo estação (necessário) |

Build: `pio run -e e1_nano` · Gravar: `-t upload` · Docker: `docker compose run --rm firmware pio run`.
