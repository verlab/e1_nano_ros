# 3. Firmware e primeiro voo (manual)

## 3.1 Montagem

Siga o guia original do projeto ([oshwhub](https://oshwhub.com/songge8/project_fnvbukbz)). PCB de 1,2 mm,
ESP32-S3 SuperMini por cima da placa, bateria LiPo 1S por baixo presa com velcro, motores 0716 nos anéis de
borracha. **Sentido dos motores** (vistos de cima): frente-esquerda e trás-direita no sentido horário;
frente-direita e trás-esquerda no anti-horário.

## 3.2 Gravar

```bash
git submodule update --init --recursive
cd firmware
pio run -e e1_nano -t upload        # segure BOOT ao conectar o USB na primeira gravação
pio device monitor                  # 115200
```

Sem PlatformIO: `docker compose run --rm --device /dev/ttyACM0 firmware pio run -t upload`.

## 3.3 Calibração (uma vez)

No monitor serial (ou no Web RC do CF-Drone em `http://192.168.4.1` → Depurar):

```text
ca            # calibração de 6 posições do IMU, siga as instruções
ps            # roll/pitch próximos de 0 => ok
mfl / mfr / mrl / mrr   # testa cada motor SEM hélices
```

## 3.4 Primeiro voo manual

Com o firmware de fábrica (modo AP `Drone_WiFi` / `12345678`), use o Web RC em `http://192.168.4.1`
para um voo curto e confirme que tudo funciona antes de passar ao Zenoh.

## 3.5 Entrar na rede do laboratório (Zenoh)

```text
sta VerlabWiFi senha123
p WIFI_MODE 2
zn id 1
zn connect tcp/192.168.1.100:7447     # IP do PC com o roteador
reboot
zn                                    # "connected", rx/tx subindo
```

No PC:

```bash
ros2 run rmw_zenoh_cpp rmw_zenohd     # ou: docker compose up router
e1-cli list                           # -> drones: 1
e1-cli status
```

Daqui em diante o drone aparece para o ROS exatamente como no simulador.
Próximo: [voo real com captura de movimento](04-voo-real.md).
