# 6. Genesis e aprendizado por reforço

O backend Genesis usa o mesmo firmware em Python e o mesmo protocolo:

```bash
pip install genesis-world torch        # GPU recomendada
python3 -m e1_core.sim.cli -n 4 --backend genesis --device gpu --mode peer --listen tcp/127.0.0.1:7447 --connect ""
```

Para RL, use o URDF `e1_description/urdf/e1_nano_genesis.urdf` com `gs.morphs.Drone`:

```python
import genesis as gs
gs.init(backend=gs.gpu)
scene = gs.Scene(sim_options=gs.options.SimOptions(dt=0.002))
scene.add_entity(gs.morphs.Plane())
drone = scene.add_entity(gs.morphs.Drone(
    file='e1_description/urdf/e1_nano_genesis.urdf',
    propellers_link_name=('rotor_rear_left', 'rotor_rear_right', 'rotor_front_right', 'rotor_front_left'),
    propellers_spin=(-1, 1, -1, 1)))     # sinal do torque de reação = -(sentido físico)
scene.build(n_envs=4096)                 # ambientes paralelos
drone.set_propellers_rpm(rpm)            # (n_envs, 4), kf = 1.496e-10 N/rpm^2
```

!!! note "Sinal de rotação no Genesis"
    O Genesis aplica `spin * km * rpm²` como torque **no corpo**. Por isso passamos o sinal do torque de
    reação, `-(sentido físico)`. Com isso o mixer do CF-Drone funciona igual no MuJoCo e no Genesis
    (verificado com um degrau de guinada).

Um ponto de partida: treine políticas que emitem o mesmo comando do firmware (quaternion + empuxo).
Elas rodam no drone real sem mudanças, publicadas em `e1/<id>/cmd/attitude`.
