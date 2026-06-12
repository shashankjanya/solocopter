# Single-Propeller Thrust-Vectored UAV (Solocopter) Simulation

## Overview
This project focuses on the modeling, simulation, and control system design for a Single-Propeller Tail-Sitter UAV (solocopter). Unlike traditional multirotors that rely on differential thrust, this vehicle achieves 6-DOF control using a single propulsion unit and four aerodynamic control flaps submerged in the propeller slipstream. 

The primary goal of this repository is to develop a non-linear dynamics model in MATLAB/Simulink and design multiple control architectures suitable for eventual deployment on the PX4 autopilot stack.

## Current Status
This project is currently in the **Modeling and Simulation Phase**. 
- [x] Rigid Body Kinematics & Dynamics (6-DOF)
- [x] Propeller Slipstream, Induced Velocity, and Control Forces Modeling
- [x] Trim State Calculation
- [x] Cascaded PID Control Architecture (Simulink)
- [ ] Incremental Nonlinear Dynamic Inversion (INDI) Control Architecture (Simulink)
- [ ] PX4 Software-In-The-Loop (SITL) Integration

## Fundamentals

### 1. The Plant Model
Standard multirotor equations do not apply directly to this vehicle. The forces and moments are generated via thrust vectoring. The core physics model includes:
* **Slipstream Dynamics:** Control surface effectiveness is scaled dynamically based on propeller induced velocity (Momentum Theory). Control authority drops at low throttle, requiring non-linear compensation.
* **Anti-Torque Trim:** The single propeller generates a massive reactive yaw torque. The default state of the vehicle requires a collective deflection of all four flaps just to maintain a zero-yaw hover.
* **Cross-Coupling:** The aerodynamic model accounts for the complex coupling between roll/pitch moments and lateral translations caused by flap deflection.

### 2. Controller Design
* **Control Allocation Matrix:** Developed based on the dynamic inversion of the control force equations to map virtual torques to physical actuator deflections.
* **Cascaded PID Control Architecture:** The baseline control logic mimics the PX4 flight stack architecture to ensure future compatibility:
    * **Outer Loop:** Attitude Proportional (P) controller converting angle errors to target body rates.
    * **Inner Loop:** Rate PID controller converting rate errors to virtual torque commands. 
    * **Gain Scheduling:** PID outputs are inversely scaled by the throttle command to maintain consistent control authority across the flight envelope.

## How to Run
1. Clone this repository.
2. Open MATLAB and navigate to the project directory.
3. Run the initialization script (`matlab/init.m`) to load the UAV mass, inertia, and aerodynamic coefficients into the workspace.
4. Open the Simulink model (`simulink/controller_tuning.slx`).
5. Run the simulation to view the solocopter fly a simple demo path. 

## Acknowledgments
The mechanical topology and conceptual inspiration for this control scheme draw from active developments in medical and cargo delivery eVTOLs (e.g., Maple Aviation).
