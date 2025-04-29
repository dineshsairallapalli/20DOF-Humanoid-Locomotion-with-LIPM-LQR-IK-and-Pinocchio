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


pip install numpy scipy pin mujoco

Additionally:

✅ MuJoCo >= 2.3 installed and licensed

✅ Python >= 3.8

✅ Visual display (for mujoco.viewer)

✅ Pinocchio library installed (pip install pin)

---

🚀 How to Run
Clone the repository:

git clone https://github.com/dineshsairallapalli/20DOF-Humanoid-Locomotion-with-LIPM-LQR-IK-and-Pinocchio.git
cd 20DOF-Humanoid-Locomotion-with-LIPM-LQR-IK-and-Pinocchio

---

Set MuJoCo path: Edit the XML path in walking_simulation.py:

xml = r"absolute\path\to\scene.xml"

Run the simulation:

python walking_op3.py

The simulation will launch a MuJoCo viewer window and execute the planned walking gait in real-time.

🧠 Methodology
🧩 1. Footstep Planning
Define a time-based sequence of support phases and target footstep locations.

🔄 2. ZMP Reference Generator
Creates a piecewise affine ZMP trajectory based on the current walking phase.

⚖️ 3. LIPM + LQR CoM Controller
Simulates CoM motion using:

A 2D linear inverted pendulum model

LQR to stabilize CoM to ZMP

🦿 4. Swing Foot Trajectory
A cubic Hermite spline generates a smooth foot swing path with a configurable mid-lift height.

🔧 5. Inverse Kinematics with Pinocchio
Uses frame-level IK with cost minimization (BFGS) to compute robot joint angles that meet CoM and foot position constraints.

🧪 Features
Full walking sequence with alternating support

Modular code for ZMP control and foot trajectory design

Compatible with MuJoCo’s native scene and physics

Lightweight Euler integration for real-time simulation

**Known Limitations**
Not integrated with a whole-body QP solver or external disturbances

Assumes flat ground and pre-defined foot targets

IK uses numerical optimization (may require tuning for convergence)

📌 Future Work
Terrain adaptation and dynamic replanning

Whole-body control using QP solvers

ROS2 integration and hardware deployment

Vision/IMU sensor integration for feedback

📄 License
This project is released under the MIT License.
