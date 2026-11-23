# Contributing

* Style: PEP 8 (`flake8 --max-line-length 140`), ROS 2 Python (ament_python).
* Tests: `cd e1_core && python3 -m pytest -q` must pass. CI also runs `colcon build` + `colcon test`
  and builds the firmware.
* Protocol: any change to `protocol.py` must be mirrored in `firmware/src_e1/e1_protocol.h`
  (the size `static_assert`s help). Increment `VERSION` when compatibility breaks.
* Documentation: `docker compose --profile tools up docs` → http://localhost:8000.
* Commits: imperative-mood messages (`bridge: publish odom with velocity`).
