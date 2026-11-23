# 6. Genesis and reinforcement learning

The Genesis backend uses the same Python firmware and the same protocol:

```bash
pip install genesis-world torch        # GPU recommended
python3 -m e1_core.sim.cli -n 4 --backend genesis --device gpu --mode peer --listen tcp/127.0.0.1:7447 --connect ""
```

For RL, use the URDF `e1_description/urdf/e1_nano_genesis.urdf` with `gs.morphs.Drone`:

```python
import genesis as gs
gs.init(backend=gs.gpu)
scene = gs.Scene(sim_options=gs.options.SimOptions(dt=0.002))
scene.add_entity(gs.morphs.Plane())
drone = scene.add_entity(gs.morphs.Drone(
    file='e1_description/urdf/e1_nano_genesis.urdf',
    propellers_link_name=('rotor_rear_left', 'rotor_rear_right', 'rotor_front_right', 'rotor_front_left'),
    propellers_spin=(-1, 1, -1, 1)))     # reaction torque sign = -(physical direction)
scene.build(n_envs=4096)                 # parallel environments
drone.set_propellers_rpm(rpm)            # (n_envs, 4), kf = 1.496e-10 N/rpm^2
```

!!! note "Spin sign in Genesis"
    Genesis applies `spin * km * rpm²` as a torque **on the body**. For this reason we pass the sign of the
    reaction torque, `-(physical direction)`. With this convention the CF-Drone mixer behaves identically in MuJoCo and in Genesis
    (verified with a yaw step).

A suggested starting point: train policies that output the same command as the firmware (quaternion + thrust).
They run on the real drone without changes, published on `e1/<id>/cmd/attitude`.
