# Tilted-Rotor Allocation Optimization and Simulation

This repository contains MATLAB code to optimize the rotor orientations (normals `N`) of an 8-rotor system across multiple center-of-mass (CoM) configurations. The optimization improves controllability by acting on the allocation matrix using multi-start `fmincon`. Subsequently, a trajectory tracking simulation compares three control strategies under different CoM shifts.

## Entry Points

The project contains **two main mlx scripts** that need to be executed:

1. **Optimization script** 
   Runs the multi-start optimization of rotor orientations across the CoM configurations and generates the result files in `res/`.

2. **Simulation script** 
   Loads the optimized geometries and runs the trajectory tracking simulations to compare the three cases.

These are the **only `.m` files that need to be executed manually**.

All other MATLAB functions used throughout the scripts are stored in the helper directories:

- `lib/`
- `lib/omnimorph-helper/`
- `lib/math-helper/`
- `plot/`

These folders contain utility functions for:
- allocation matrix computation
- geometric transformations
- optimization helpers
- plotting and visualization

The scripts automatically add these folders to the MATLAB path using `addpath(...)`, so no manual setup is required beyond running the main scripts.

---

## What the code does

### 1. Optimization 
For each CoM configuration:
- Builds the geometry (rotor positions relative to the current CoM)
- Runs MultiStart optimization (`fmincon`) with two objectives (standard and log-volume)
- Extracts the best solution and builds `N_all`
- Produces analysis plots (drone views, performance degradation, plane fitting, etc.)

### 2. Simulation & Control Comparison
Loads the optimized geometries and simulates a predefined trajectory (from `omnirotor_params.m`) for each CoM shift. Three cases are compared:
- **Case A (Adaptive):** controller and plant both use the optimized geometry for the current CoM.
- **Case B (Fixed normals, CoM‑dependent positions):** normals kept at nominal (CoM #1), rotor positions follow current CoM.
- **Case C (Fully fixed nominal geometry):** both normals and positions are nominal, but plant/trajectory is evaluated at the shifted CoM.

The simulation computes:
- Tracking errors (RMSE position & orientation)
- Control energy (sum of squared inputs)
- Maximum actuator commands (force and speed)
- Condition number and minimum singular value of the allocation matrix
- Visualizations: time histories, commanded speeds, thrust, force/torque polytopes, and an animated 3D trajectory.

---

## Requirements

- MATLAB
- Optimization Toolbox
- Project helper functions located in:
  - `lib/`
  - `lib/omnimorph-helper/`
  - `lib/math-helper/`
  - `plot/`
- (Optional) VideoWriter for animation (built-in)

---

## Folder structure
project_root/
├── run_pipeline.m 				% optimization script
├── run_simulation_comparison.m 		% simulation & comparison script
├── omnirotor_params.m 			% system & trajectory parameters
├── lib/ 					% core helper functions
├── plot/ 					% plotting utilities
├── res/ 					% saved .mat results (optimization output)
└── figures/ 					% generated figures



---

## The parameter `k`: selecting the dynamic formulation

Both the optimization and simulation scripts use a variable `k` to choose between two alternative dynamic formulations for the allocation matrix:

- **`k = 1`** → **No Drag formulation**: the moment equation does **not** include the rotor spin term (simplified model).
- **`k = 2`** → **Classical formulation**: the moment equation **includes** the rotor spin term (full model).

Make sure to set `k` at the beginning of both scripts before running them.

---

