# Inverse-Kinematics-of-6-DOF-Manipulator-witha-spherical-Wrist

## 6‑DOF Manipulator IK (DH‑Based) + Arduino Control

This repository contains:
- A MATLAB inverse kinematics (IK) implementation using Denavit–Hartenberg (DH) parameters
- An Arduino Mega sketch (`IK_function.ino`) that runs the same IK on a real 6‑DOF manipulator driven by stepper motors

## Quick Summary
- IK uses DH parameters and closed‑form trigonometry (position + orientation)
- Orientation uses yaw/pitch/roll with `R60 = Rz * Ry * Rx`
- Arduino sketch takes serial input (x, y, z, yaw, pitch, roll), computes IK, and moves the arm

## Repo Structure
- `matlab/` – MATLAB implementation and tests
- `arduino/IK_function/` – Arduino Mega sketch
- `docs/` – diagrams, DH tables, wiring (optional)
- `cad/` – STL / drawing exports (optional)

## Kinematics Overview
**Link lengths (mm)**
- MATLAB: `l1=112`, `l2=265`, `l3=210`, `l4=130`, `lx=50`
- Arduino: `l1=114`, `l2=265`, `l3=218`, `l4=130`, `lx=20`

**Orientation**
- Yaw, pitch, roll (degrees in Arduino input)
- Rotation order: `Rz * Ry * Rx`

**Wrist center**

Pw = Pg - l4 * R60(:,3)


## MATLAB: How to Run
1. Open `matlab/Kinematics.m`
1. Set goal position and orientation at the top of the script
1. Run the script
1. It prints desired vs. computed positions and orientation

## Arduino: How to Run
**Hardware**
- Arduino Mega
- 6 stepper drivers
- 1 servo (gripper)

**Pins**
- Joint 1: dir `48`, step `49`
- Joint 2: dir `27`, step `26`
- Joint 3: dir `53`, step `52`
- Joint 4: dir `35`, step `34`
- Joint 5: dir `39`, step `38`
- Joint 6: dir `43`, step `42`
- Servo: pin `11`

**Steps**
1. Open `arduino/IK_function/IK_function.ino` in Arduino IDE
1. Upload to Arduino Mega
1. Open Serial Monitor at `9600`
1. Enter:
   - `x`, `y`, `z` (mm)
   - `yaw`, `pitch`, `roll` (degrees)

## Known Assumptions / Caveats
- `yg` offset compensation is applied in Arduino:
  - if `yg < 0`: `yg = yg - 90`
  - else: `yg = yg + 50`
- Gear ratios are applied to convert angles to motor steps:
  - `g1=40, g2=50, g3=50, g4=300, g5=9, g6=25`
- MATLAB and Arduino link lengths currently differ; ensure they match if you want consistent results.

## Credits
Designed and implemented by Fawad Mehboob

