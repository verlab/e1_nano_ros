# Contribuindo

* Estilo: PEP 8 (`flake8 --max-line-length 140`), ROS 2 Python (ament_python).
* Testes: `cd e1_core && python3 -m pytest -q` precisa passar. A CI também roda `colcon build` + `colcon test`
  e compila o firmware.
* Protocolo: qualquer mudança em `protocol.py` deve ser espelhada em `firmware/src_e1/e1_protocol.h`
  (os `static_assert` de tamanho ajudam). Incremente `VERSION` se quebrar a compatibilidade.
* Documentação: `docker compose --profile tools up docs` → http://localhost:8000.
* Commits: mensagens no imperativo (`bridge: publica odom com velocidade`).
