# High-Performance Field-Oriented Control (FOC) Drive Engine for an EV Traction Motor

A high-fidelity mathematical simulation framework modeling a high-power **Permanent Magnet Synchronous Motor (PMSM)** driven by Field-Oriented Control vector tracking and **Space Vector Pulse Width Modulation (SVPWM)**.

## Project Architecture
```text
ev-pmsm-foc-drive/
├── README.md                  # Detailed system documentation
├── requirements.txt           # Dependency requirements
├── notebooks/
│   └── pmsm_foc_simulation.ipynb   # Interactive cloud execution engine
└── src/                       # Pure mathematical modular source code
    ├── motor_model.py         # State-space continuous ODE motor engine
    ├── controllers.py         # Clarke/Park matrices & anti-windup PI blocks
    └── inverter.py            # SVPWM duty synthesis & switching sector logic
