# Solução de problemas

| Sintoma | Causa provável |
|---|---|
| `e1-cli list` vazio | roteador parado, IP/porta errados no `zn connect`, WiFi em AP (`p WIFI_MODE 2`), firewall bloqueando a 7447/TCP |
| `zn` mostra `err` subindo | versão de protocolo/zenoh diferente: atualize firmware e `e1_core` juntos |
| `ros2 topic list` vazio | falta `RMW_IMPLEMENTATION=rmw_zenoh_cpp` ou o `rmw_zenohd` |
| drone desce sozinho no meio do voo | `cmd_timeout`: WiFi saturado ou controlador parado; veja `/e1_<id>/state.cmd_timeout` |
| deriva em hover | trims `CTL_TRIM_ROLL/PITCH`, calibração `ca`, hélices invertidas |
| simulador lento | `--record` faz renderização em CPU; use `examples/render_demo.py` (offline) |
