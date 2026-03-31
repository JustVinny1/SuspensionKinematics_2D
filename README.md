# 2D Double-Wishbone Suspension Kinematics Solver

A MATLAB tool that computes and visualises the kinematic behaviour of a planar double-wishbone suspension over a range of vertical wheel travel.

This 2D solver is a learning exercise to develop and validate the numerical methods (Newton-Raphson with finite-difference Jacobians, constraint-based formulation) before extending them to a full 3D suspension kinematics solver.

## What it computes

- **Camber angle** vs wheel travel
- **Track width change** vs wheel travel
- **Roll-centre height** vs wheel travel (via instant-centre construction)
- **Animated mechanism** showing wishbone links, upright, contact patch, and roll centre

## How to use

1. Open `SuspensionKinematics_2D.m` in MATLAB.
2. Edit the **USER INPUTS** section to define your suspension geometry (chassis hardpoints, ball joint positions, wheel centre, contact patch offset) and sweep parameters.
3. Run the script.

Results are printed to the command window and displayed across three plots and an animation.

## File structure

| File | Description |
|------|-------------|
| `SuspensionKinematics_2D.m` | Main script -- geometry definition, sweep, plotting, animation |
| `upright_points.m` | Kinematic transformation from generalised coordinates to world-frame positions |
| `constraints.m` | Link-length constraint residuals (squared-length formulation) |
| `numerical_jacobian.m` | 2x2 Jacobian via central finite differences |
| `solve_travel_step.m` | Newton-Raphson solver for a single travel step |
| `find_roll_centre.m` | Instant-centre and roll-centre height calculation |

## Method

The suspension is modelled as two rigid links (upper and lower wishbones) connecting chassis hardpoints to an upright that translates and rotates in 2D. The unknowns are lateral displacement (`dy`) and camber angle (`alpha`) for a prescribed vertical travel (`dz`).

A Newton-Raphson iteration drives the two link-length constraint residuals to zero at each travel step. The Jacobian is evaluated numerically using central finite differences. The sweep starts from the design position (dz = 0) in both directions so the solver always works from a small perturbation.

## Units

All lengths are in **millimetres**, all angles in **radians** internally (displayed in degrees on plots).

## AI Usage Disclosure

AI tools were used as a research and development aid in this project:

- **Theory and guidance:** AI assisted with explaining mathematical concepts, suggesting implementation approaches, and debugging
- **Code authorship:** All code was tested and validated by me. AI did not generate complete scripts or functions
- **Comments and documentation:** AI assisted with standardising code comments across function files at to write documentation for the project.
- **Responsibility:** I take full responsibility for the correctness, structure, and content of this project
