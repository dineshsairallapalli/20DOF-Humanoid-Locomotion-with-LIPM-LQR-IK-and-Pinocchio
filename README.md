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

Additionally:

✅ MuJoCo >= 2.3 installed and licensed

✅ Python >= 3.8

✅ Visual display (for mujoco.viewer)

✅ Pinocchio library installed (pip install pin)

🚀 How to Run
Clone the repository:

git clone https://github.com/dineshsairallapalli/20DOF-Humanoid-Locomotion-with-LIPM-LQR-IK-and-Pinocchio.git
cd 20DOF-Humanoid-Locomotion-with-LIPM-LQR-IK-and-Pinocchio

Set MuJoCo path: Edit the XML path in walking_simulation.py:

xml = r"absolute\path\to\scene.xml"
