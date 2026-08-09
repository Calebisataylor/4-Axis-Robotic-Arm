## **Overview**

A 4-axis, cable driven, robotic arm, designed and built from scratch to demonstrate capabilities in: 

- Embedded systems
- Circuit design
- 3D CAD design (fusion 360)
- Motor control
- Real-time systems integration

- 
## Quick Specs

- **Kinematics/Motion:** 4 Degrees of Freedom (Base yaw, shoulder pitch, elbow pitch, wrist pitch) plus an independent 4-finger gripper.
- **Optimized Cycloidal Gearboxes:** Adapted and tuned open-source 30:1 and 20:1 ratio gearboxes. Adjusted the 180° phased dual-disc design to minimize vibration and correct offset dimensions in the eccentric shaft.
- **CAD Modifications:** Modified the base models in Fusion 360, specifically: the eccentricity, the output part, the gearbox casing, and the offset holes in each of the discs that hold the output pins.
- **Embedded Architecture**: Dual-ESP32 control scheme; the master MCU handes the real time motion/collision bounds, the slave MCU controls the gripper/finger actuation over a one way RX-TX link.
