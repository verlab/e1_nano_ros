# Simulação

## MuJoCo (padrão)

* **Física:** 1 kHz. **Firmware:** 500 Hz (porte do CF-Drone). **Telemetria:** `state` 50 Hz, `pose` 100 Hz.
* **Motores:** duty × (V/4,2) → rotação normalizada com τ = 20 ms; empuxo = 0,216 N · rot².
  Torque de reação = 0,005 m × empuxo.
* **Sensores:** giroscópio e acelerômetro do MuJoCo com ruído gaussiano (0,01 rad/s; 0,05 m/s²). A atitude
  é a verdade de campo, já que o estimador do CF-Drone não foi portado.
* **Bateria:** 250 mAh, curva linear de 4,15 V a 3,3 V com queda de 0,05 V/A. Corrente ≈ 1,6 A · Σduty².
* **Colisões:** placa, braços, pilha ESP32/velcro, bateria, motores e pernas (primitivas). Drones colidem
  entre si e com o chão.
* **Multi-drone:** o MJCF de um drone é replicado com prefixo `d<id>_`.

```bash
python3 -m e1_core.sim.cli -h
e1-sim -n 8 --layout circle --spacing 0.6 --viewer
e1-sim -n 3 --record voo.mp4 --duration 30
e1-sim -n 1 --speed 0          # tão rápido quanto possível (≈12× o tempo real para 1 drone num laptop)
```

## Genesis

`--backend genesis [--device gpu|cpu]`. Mesmo firmware e mesmo protocolo. Veja o [tutorial 6](tutorial/06-genesis.md).

## Validação incluída

`e1_core/test/test_sim.py`:

* hover + quadrado de 0,6 m: erro de rastreamento < 10 cm durante todo o percurso;
* 4 drones a 0,5 m; sem comandos, o firmware desce e desarma sozinho (failsafe);
* parada de emergência por broadcast;
* `test_zenoh.py`: decolagem e pouso de 2 drones através do Zenoh real.

## Limitações conhecidas

* Sem efeito solo, *downwash* entre drones, arrasto do rotor ou vento.
* O empuxo e a constante de tempo dos motores são estimativas: identifique os seus (`cmd/motors` +
  balança) e ajuste `MAX_THRUST`/`MOTOR_TAU`.
