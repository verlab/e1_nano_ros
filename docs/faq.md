# FAQ

**O drone não aparece em `e1-cli list`.**
Rode `zn` no console: o modo WiFi precisa ser STA (`p WIFI_MODE 2`), o IP do roteador precisa estar certo
e a porta 7447/TCP liberada no firewall do PC. Confira também que a versão do zenoh-pico é 1.10.x, igual
à do roteador.

**`ros2 topic list` não mostra nada.**
Falta `export RMW_IMPLEMENTATION=rmw_zenoh_cpp` ou o roteador (`ros2 run rmw_zenoh_cpp rmw_zenohd`).

**Posso usar Fast DDS/Cyclone no resto do sistema?**
Pode. O `e1_bridge` fala Zenoh com os drones pela biblioteca Python, de forma independente do RMW. Para
um enxame grande, porém, recomendamos `rmw_zenoh` em todo o sistema.

**E o micro-ROS?** Veja [Escolha do middleware](middleware.md).

**Quero voar sem mocap.** O voo manual funciona pelo Web RC do CF-Drone ou por `cmd/manual`. Voo
autônomo por posição precisa de algum posicionamento externo (mocap, UWB, câmera + ArUco); o
`e1_bridge` aceita qualquer `PoseStamped` em `mocap_topic`.

**Por que o simulador "sabe" a atitude exata?** O estimador complementar do CF-Drone não foi portado;
usamos a atitude verdadeira com ruído nos sensores. Para estudar o estimador, porte `estimate.ino` para
`e1_core.firmware`.

**Funciona com o E1 MINI (ESP32 clássico)?** O firmware sim (`pio run -e e1_mini`), mas o modelo 3D e as
constantes físicas são do Nano.
