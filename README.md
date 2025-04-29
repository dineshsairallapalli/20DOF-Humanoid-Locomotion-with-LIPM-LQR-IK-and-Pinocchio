# 🤖 20DOF Humanoid Locomotion with LIPM, LQR, IK, and Pinocchio

This project demonstrates Humanoid walking simulation using MuJoCo and Pinocchio, featuring ZMP-based locomotion planning, LQR CoM control, swing foot trajectory generation, and inverse kinematics for dynamic walking of a Robotis OP3 robot.  It integrates:

- 🧠 **Linear Inverted Pendulum Model (LIPM)** for dynamic walking
- 🧮 **LQR control** for Center of Mass (CoM) stabilization
- 🦶 **Swing foot trajectory generation** via cubic Hermite splines
- 🔧 **Inverse Kinematics (IK)** solved using Pinocchio and BFGS optimization
- 🎮 Simulated in **MuJoCo** with live visualization


---

## ⚙️ Dependencies

Make sure the following are installed:

```bash
pip install numpy scipy pin mujoco
