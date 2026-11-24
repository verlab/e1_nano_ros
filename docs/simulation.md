# Simulation

## MuJoCo (default)

* **Physics:** 1 kHz. **Firmware:** 500 Hz (CF-Drone port). **Telemetry:** `state` 50 Hz, `pose` 100 Hz.
* **Motors:** duty × (V/4.2) → normalized speed with τ = 20 ms; thrust = 0.216 N · speed².
  Reaction torque = 0.005 m × thrust.
* **Sensors:** MuJoCo gyroscope and accelerometer with Gaussian noise (0.01 rad/s; 0.05 m/s²). The attitude
  is ground truth, since the CF-Drone estimator has not been ported.
* **Battery:** 250 mAh, linear curve from 4.15 V to 3.3 V with a 0.05 V/A sag. Current ≈ 1.6 A · Σduty².
* **Collisions:** board, arms, ESP32/velcro stack, battery, motors and legs (primitives). Drones collide
  with each other and with the ground.
* **Multi-drone:** the single-drone MJCF is replicated with the prefix `d<id>_`.

```bash
python3 -m e1_core.sim.cli -h
e1-sim -n 8 --layout circle --spacing 0.6 --viewer
e1-sim -n 3 --record flight.mp4 --duration 30
e1-sim -n 1 --speed 0          # as fast as possible (≈12× real time for 1 drone on a laptop)
```

## Suspended cargo (MuJoCo)

The optional `cargo` scene adds a 10 g free box and four physical tethers, each with a maximum length of
30 cm. The tethers are unilateral constraints: they pull when taut and go slack otherwise. The existing
swarm scene remains the default; the cargo scene currently requires exactly four drones in a grid layout
and is not supported by the Genesis backend.

On Linux with the Docker demo image built and X11 access enabled:

```bash
docker compose build sim
xhost +local:docker
E1_N=4 docker compose up -d router bridge controller
E1_N=4 docker compose run --rm \
  -e DISPLAY="$DISPLAY" \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  sim ros2 run e1_core e1-sim -n 4 --scene cargo --layout grid --viewer
```

In another terminal, ask the swarm controller to lift and then land the drones:

```bash
docker compose run --rm tools ros2 service call /e1/takeoff_all e1_msgs/srv/Takeoff "{height: 1.5, duration: 10.0}"
docker compose run --rm tools ros2 service call /e1/land_all e1_msgs/srv/Land "{duration: 3.0}"
```

The built-in formation mission is intentionally omitted: it moves drones apart and is not designed for
the shared cargo. Adjust the load and tether length with `--box-mass` and `--tether-length`, respectively.

## Genesis

`--backend genesis [--device gpu|cpu]`. Same firmware and same protocol. See [tutorial 6](tutorial/06-genesis.md).

## Included validation

`e1_core/test/test_sim.py`:

* hover + 0.6 m square: tracking error < 10 cm along the whole path;
* 4 drones at 0.5 m; without commands, the firmware descends and disarms on its own (failsafe);
* emergency stop by broadcast;
* `test_zenoh.py`: takeoff and landing of 2 drones over real Zenoh.

## Known limitations

* No ground effect, inter-drone *downwash*, rotor drag or wind.
* Motor thrust and time constant are estimates: identify your own (`cmd/motors` +
  scale) and adjust `MAX_THRUST`/`MOTOR_TAU`.
