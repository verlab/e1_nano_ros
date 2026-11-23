# 3. Firmware and first flight (manual)

## 3.1 Assembly

Follow the original project guide by Cherglow ([OSHWHub](https://oshwhub.com/songge8/project_fnvbukbz)). 1.2 mm PCB,
ESP32-S3 SuperMini on top of the board, 1S LiPo battery underneath attached with velcro, 0716 motors in the
rubber rings. **Motor spin directions** (seen from above): front-left and rear-right clockwise;
front-right and rear-left counterclockwise.

## 3.2 Flashing

```bash
git submodule update --init --recursive
cd firmware
pio run -e e1_nano -t upload        # hold BOOT while plugging in USB for the first upload
pio device monitor                  # 115200
```

Without PlatformIO: `docker compose run --rm --device /dev/ttyACM0 firmware pio run -t upload`.

## 3.3 Calibration (once)

In the serial monitor (or in the CF-Drone Web RC at `http://192.168.4.1` → Debug):

```text
ca            # 6-position IMU calibration, follow the instructions
ps            # roll/pitch close to 0 => ok
mfl / mfr / mrl / mrr   # tests each motor WITHOUT propellers
```

## 3.4 First manual flight

With the factory firmware (AP mode `Drone_WiFi` / `12345678`), use the Web RC at `http://192.168.4.1`
for a short flight and confirm that everything works before moving on to Zenoh.

## 3.5 Joining the lab network (Zenoh)

```text
sta VerlabWiFi password123
p WIFI_MODE 2
zn id 1
zn connect tcp/192.168.1.100:7447     # IP of the PC running the router
reboot
zn                                    # "connected", rx/tx increasing
```

On the PC:

```bash
ros2 run rmw_zenoh_cpp rmw_zenohd     # or: docker compose up router
e1-cli list                           # -> drones: 1
e1-cli status
```

From this point on, the drone appears to ROS exactly as in the simulator.
Next: [real flight with motion capture](04-real-flight.md).
