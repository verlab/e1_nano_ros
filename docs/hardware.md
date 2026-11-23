# Hardware and 3D model

The E1 Nano hardware and the CF-Drone flight firmware are open-source designs by Cherglow
([songge8 on OSHWHub](https://oshwhub.com/songge8/project_fnvbukbz)).

## E1 Nano v1.5Y

| Item | Value | Source |
|---|---|---|
| Flight controller | ESP32-S3 SuperMini (ESP32-S3FH4R2) | official specification |
| IMU | MPU-6500 (SPI) | BOM |
| Wheelbase | **50.0 mm** (motor-to-motor diagonal) | PCB (measured) = specification |
| PCB | 50.9 × 50.9 × 1.2 mm, X frame with rounded arms | outline extracted from the PCB |
| Motor hole | Ø 8.5 mm (motor + rubber ring) | PCB |
| Drivers | 4× AO3416 + 4× B5819W (brushed motors) | BOM |
| Regulator | ME6118A33 (SOT-89) | BOM |
| Legs | 4 boards from the same panel, vertical, 18.6 mm below the board | panel outline |
| Mass | ~25 g with battery | official specification |
| Motors | 0716 coreless (Ø7 × 16 mm) | **estimated** from photos and the motor hole |
| Propellers | 31 mm, 4 blades | **estimated** from photos |
| Battery | 1S LiPo ~250 mAh, under the board, attached with velcro | **estimated** |
| Max. thrust/motor | 0.216 N (≈22 gf) @ ~38 000 rpm | **estimated**: measure and adjust `max_thrust` |

## Model

![E1 Nano 3D model](assets/e1_nano_model.png)

The 3D model was generated from the actual PCB data (outline, holes, pads, silkscreen and position of
each component). The files are in `e1_description/`:

| File | Use |
|---|---|
| `urdf/e1_nano.urdf` | ROS 2 (`package://`), gz-sim MulticopterMotorModel + IMU plugins |
| `urdf/e1_nano_standalone.urdf` | relative paths (PyBullet, Isaac...) |
| `urdf/e1_nano_genesis.urdf` | Genesis `gs.morphs.Drone` (`<properties kf km>`, massless propellers) |
| `mjcf/e1_nano.xml` | MuJoCo with thrust actuators and sensors |
| `meshes/visual/*.stl` | meshes in meters, one per material |
| `meshes/e1_nano_full.glb` | colored model (Blender / viewers) |
| `config/e1_nano.yaml` | physical parameters |

Masses and inertias are computed from the meshes, with densities scaled so that the total is 25.1 g. The center
of mass lies 1.3 mm below the PCB mid-plane.

## Motor order and spin direction

```
        front (x)
   FL (3) CW    FR (2) CCW
          \    /
           \  /
           /  \
          /    \
   RL (0) CCW   RR (1) CW
```

Indices = CF-Drone `motors[]` (`MOTOR_REAR_LEFT = 0` ...). Directions as seen from above, following the mixer
(`control.ino`).
