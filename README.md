# Dynamic Programming for Optimal Control

Finite-horizon **dynamic-programming (DP) optimal control** worked through three classic systems — a mass–damper, a single-link manipulator, and an inverted pendulum. A clean, example-driven reference for solving optimal-control problems by value iteration over a discretized state space.

## Overview

Each example defines a state grid, an input grid, a stage cost, and a terminal cost, then solves for the optimal control policy by dynamic programming and traces the optimal trajectory from a given initial condition. It's a hands-on companion to the theory: you can see exactly how discretization, horizon length, and cost shaping change the result.

## Examples

| Folder | System | Task |
|--------|--------|------|
| `mass-damper/` | 1 kg mass, damping 0.1 | Move x: 0 → 0.5 m in 1 s and stop, bounded force (±4 N) |
| `single-link-manipulator/` | Single-link arm | Drive position/velocity to target over a 2 s horizon |
| `inverted-pendulum/` | Pendulum | Fixed-horizon optimal stabilization |

Each folder contains its own `ex02_dpa_*.m` entry script, Simulink model, and system-specific functions.

## Run it

```matlab
% 1) Install the yadpf framework and add it to your MATLAB path (see Attribution)
% 2) From any example folder, in MATLAB:
ex02_dpa_mass          % (mass-damper)  — or ex02_dpa_singleLink / ex02_dpa_Pendulum
```

Each script solves the DP problem, traces the optimal trajectory, and plots the optimal states and control input.

## Requirements

MATLAB with **Simulink**, plus the open-source **yadpf** framework on your path (it provides the `yadpf_solve` / `yadpf_trace` DP solver — see Attribution). No other toolboxes needed for the core examples.

## Attribution

The DP solver utilities (`yadpf_solve`, `yadpf_trace`, and helpers) are from **yadpf — Yet Another Dynamic Programming Framework** by Auralius Manurung ([GitHub](https://github.com/auralius/yadpf)), used under its original license. The problem formulations, cost design, tuning, and analysis are my own.

## Author

**Oluwaseun A. Adekoya** — Robotics Engineer & PhD Candidate, University of Cincinnati. License: MIT (my code).
