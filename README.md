# Robot Control Sim

Lightweight C++ simulator of a PID-controlled robot in 2D. Includes a Kalman filter for smoothing noisy sensor readings.

Build:

```bash
cmake -S . -B build && cmake --build build
```

Binary: `bin/robot_sim`

TODO: add a `pybind` wrapper
