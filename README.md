# 🤖 20DOF Humanoid Locomotion with LIPM, LQR, IK, and Pinocchio

This project demonstrates **humanoid walking simulation** using **MuJoCo** and **Pinocchio**, featuring ZMP-based locomotion planning, LQR-based CoM control, swing foot trajectory generation, and inverse kinematics for dynamic walking of a **Robotis OP3** robot. It integrates:

- 🧠 **Linear Inverted Pendulum Model (LIPM)** for dynamic walking
- 🧮 **LQR control** for Center of Mass (CoM) stabilization
- 🦶 **Swing foot trajectory generation** via cubic Hermite splines
- 🔧 **Inverse Kinematics (IK)** solved using Pinocchio and BFGS optimization
- 🎮 Simulated in **MuJoCo** with live visualization

---

## ⚙️ Dependencies

Install the required Python libraries:

```bash
pip install numpy scipy pin mujoco
Additional Requirements:

✅ MuJoCo >= 2.3 installed and licensed

✅ Python >= 3.8

✅ Display capability (for mujoco.viewer)

✅ Pinocchio installed (pip install pin)

🚀 How to Run
Clone the repository:

bash
Copy
Edit
git clone https://github.com/dineshsairallapalli/20DOF-Humanoid-Locomotion-with-LIPM-LQR-IK-and-Pinocchio.git
cd 20DOF-Humanoid-Locomotion-with-LIPM-LQR-IK-and-Pinocchio
Edit the MuJoCo XML path in walking_simulation.py:

python
Copy
Edit
xml = r"absolute\\path\\to\\scene.xml"
Run the simulation:

bash
Copy
Edit
python walking_op3.py
This will launch a MuJoCo viewer window and execute the planned walking gait in real-time.

🧠 Methodology
🧩 1. Footstep Planning
Define a sequence of walking phases (e.g., single support, double support) and target foot positions over time.

🔄 2. ZMP Reference Generator
Generates a piecewise affine Zero Moment Point (ZMP) trajectory to maintain balance and guide CoM movement.

⚖️ 3. LIPM + LQR CoM Controller
Simulates CoM dynamics using:

A 2D Linear Inverted Pendulum Model (LIPM)

Linear Quadratic Regulator (LQR) for stabilizing the CoM over the ZMP

🦿 4. Swing Foot Trajectory
Generates swing foot motions using cubic Hermite splines with a configurable lift height and duration.

🔧 5. Inverse Kinematics with Pinocchio
Solves joint configurations using BFGS optimization to match desired foot placements and CoM reference positions.

🧪 Features
Full walking sequence with alternating stance/swing phases

Modular control for ZMP, CoM, and foot trajectory

Simulation-ready with real-time visualization in MuJoCo

Lightweight control loop based on Euler integration

🧱 Known Limitations
Not integrated with full-body QP solvers or external disturbance handling

Currently assumes a flat walking surface

IK may require tuning for convergence in edge cases

📌 Future Work
Support for terrain variation and online footstep replanning

Whole-body balancing and dynamic QP solvers

ROS2 integration and deployment on hardware

Vision or IMU-based feedback loop integration

📄 License
This project is licensed under the MIT License. See the LICENSE file for details.
