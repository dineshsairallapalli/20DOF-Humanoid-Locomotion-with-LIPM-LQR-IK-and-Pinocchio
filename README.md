**Walking Controller for Robotis OP3 in MuJoCo + Pinocchio**

Welcome to the Robotis OP3 walking controller demo! This repository provides a pedagogical yet extensible whole-body walking framework for the OP3 humanoid in MuJoCo, combining:

* A Linear Inverted Pendulum Model (LIPM) with LQR for real-time Center of Mass (CoM) stabilization
* Cubic‐Hermite swing‐foot splines for smooth stepping
* Pinocchio‐based inverse kinematics (IK) solve via BFGS

---

## 📖 Abstract

We implement a simplified zero‐moment‐point walking algorithm:

1. **Footstep Planner**: sequences timed single‐ and double‐support phases with target foot placements.
2. **ZMP Reference**: piecewise‐affine trajectory under stance foot (and interpolation in double support).
3. **CoM Controller**: LQR‐regulated LIPM at fixed height h tracks the ZMP reference.
4. **Swing Trajectory**: 3D foot arcs using cubic‐Hermite interpolation.
5. **Inverse Kinematics**: BFGS optimizer enforces ankle pose constraints via Pinocchio.
6. **Simulation Loop**: real‐time MuJoCo stepping and viewer.

This codebase is intended for learning, extension, and benchmarking simple walking behaviors on small humanoids.

---

## 💡 Core Concepts

### Zero‐Moment Point (ZMP)

The ZMP is the point on the ground where the net moment of the inertial and gravitational forces has no component along the supporting surface. Keeping the ZMP inside the support polygon (the convex hull of contact points) ensures static stability. During single support, the polygon reduces to the stance foot; during double support, it spans both feet.

### Linear Inverted Pendulum Model (LIPM)

We approximate the robot as a point mass at constant height **h**, connected to the ground via massless legs. The horizontal dynamics simplify to:

```
dd x = omega^2 * (x - p_zmp)
omega = sqrt(g / h)
```

where **x** is the CoM horizontal position and **p\_zmp** is the reference ZMP. This model allows fast, real‐time planning of CoM motions.

### Linear Quadratic Regulator (LQR)

We compute an optimal state‐feedback gain **K** to minimize the infinite‐horizon cost:

```
J = ∫0^∞ [ (x - x_ref)^T Q (x - x_ref) + u^T R u ] dt
```

subject to the continuous dynamics ẋ = A x + B u + B p\_zmp.  The solution involves the Continuous Algebraic Riccati Equation:

```
A^T P + P A - P B R^{-1} B^T P + Q = 0
```

Once **P** is found, the optimal gain is:

```
K = R^{-1} B^T P
```

### Cubic Hermite Splines for Swing‐Foot Trajectory

To generate a smooth foot arc, we interpolate linearly in the XY‐plane and apply a cubic bump in Z:

```
z(s) = 4 * H * s * (1 - s),    s ∈ [0,1]
```

This ensures zero vertical velocity at the beginning (s=0) and end (s=1), reaching peak height **H** at the midpoint.

### Inverse Kinematics via BFGS

We enforce desired ankle frame poses by minimizing:

```
E(q) = ∑ || log( M_i(q)⁻¹ * M_des_i ) ||^2
```

where **M\_i(q)** is the SE(3) pose of frame **i**, and **M\_des\_i** is its target. Gradients are computed via finite differences (or optionally via Pinocchio’s analytic Jacobians), and the BFGS quasi-Newton method iteratively updates **q** for fast local convergence.

---

## 🚀 Quick Start

### 1. Clone & Setup

```bash
git clone https://github.com/yourusername/op3-walking-demo.git
cd op3-walking-demo
```

### 2. Install MuJoCo & Dependencies

1. **MuJoCo**: Download MuJoCo 2.X from **[https://mujoco.org](https://mujoco.org)**. Follow the platform‐specific install guide. Ensure your `mjkey.txt` license is in `~/.mujoco/` and that the MuJoCo `bin/` is on your `PATH` (Windows) or `LD_LIBRARY_PATH` (Linux).
2. **Python Env**: (Recommended: Conda)

   ```bash
   conda env create -f environment.yml
   conda activate op3-walk
   ```

   Or via pip:

   ```bash
   pip install -r requirements.txt
   ```

### 3. Run the Controller

```bash
python scripts/walk_controller.py \
  --scene scene.xml \
  --urdf urdf/op3.urdf \
  --dt 0.01 \
  --log data/run_$(date +%Y%m%d_%H%M%S).csv
```

* `--dt`: control timestep (s)
* `--log`: CSV path for recording $t, x_com, xdot_com, zmp_ref$

Upon launch, a MuJoCo GUI window appears. The OP3 will step forward according to the hard‐coded footstep sequence in `walk_controller.py`.

---

## 🔍 Detailed Modules

### 1. Footstep Planner (`scripts/utils.py: FootSteps`)

* **Data**: `phases = [(duration, support, target), …]`

  * `support`: `'none'` = double support, `'left'`/`'right'` = single support
  * `target`: 2D position `[x, y]` for the swing foot landing.
* **API**:

  * `add_phase(duration, support, target)`
  * `get_phase(t) → ((dur,sup,target), τ)`
  * `get_support_positions(t) → (stance_pos, next_swing_pos)` (computes settled foot positions automatically)

### 2. ZMP Reference Generator (`ZmpRef`)

* **Definition**: Zero Moment Point under stance foot:

  $$
    r_{zmp}(t) = \begin{cases}
      p_{stance}, & \text{single support}\\
      (1-\alpha) p_{L} + \alpha p_{R}, & \text{double support, }\alpha=τ/dur
    \end{cases}
  $$
* **Usage**: callable object `zmp_ref(t) → [x_zmp, y_zmp]`.

### 3. CoM Controller (LIPM + LQR) (`ComRef`)

* **Dynamics** (planar decoupled x/y):

  $$
    \dot x = v,\quad \dot v = \omega^2 (x - r_{zmp}),\quad \omega=\sqrt{g/h}.
  $$
* **LQR Design**: solve CARE

  $$
    A = \begin{pmatrix}0 & 1\\
                 \omega^2 & 0\end{pmatrix},\quad
    B = \begin{pmatrix}0\\ -\omega^2\end{pmatrix},
  $$

  with cost $x^T Q x + u^T R u$, leading to gain $K$.
* **Integration**: Euler step with feedback

  $$
    u = -K(x - x_{ref}),
    x_{k+1} = x_k + (A x_k + B\,r_{zmp} + B\,u)\,dt.
  $$
* **CLI Params**: height $h$, mass (optional), Q/R weights adjustable.

### 4. Swing‐Foot Trajectory (`SwingFootTrajectory`)

* Interpolation parameter $s \in [0,1]$.
* Horizontal: linear $(1 - s) p_0 + s p_1$.
* Vertical: cubic bump $z(s) = 4H s(1 - s)$ ensures smooth lift/lower.

### 5. Inverse Kinematics (`inverse_kinematics`)

* **Targets**: list of `(frame_id, SE3_desired)` for stance & swing ankles.
* **Cost**: $\sum ||\log( M_{error} )||^2$.
* **Gradient**: finite‐difference (optionally substitute with `pin.computeFrameJacobian`).
* **Optimizer**: SciPy BFGS, max iterations default 5.
* **Outputs**: joint vector $q^*$.

### 6. Logging & Visualization

* **CSV Logging**: headers `t, x_com, xdot_com, zmp_x, zmp_y` saved when `--log` is provided.
* **Post‐processing**: use `scripts/utils.py` to plot CoM vs. ZMP over time (matplotlib).
* **MuJoCo Markers**: optional XML edits in `scene.xml` to show spheres at ZMP and CoM in real time (see `<body name="marker_zmp">`).

---

## ⚙️ Command‐Line Arguments

Run `python scripts/walk_controller.py --help` for:

```text
--scene   PATH   MuJoCo XML scene file        [default: scene.xml]
--urdf    PATH   Pinocchio URDF model         [default: urdf/op3.urdf]
--dt      FLOAT  Control timestep (s)         [default: 0.01]
--plan    PATH   Footstep plan JSON           [optional]
--log     PATH   CSV file to record {t,x_com,xdot,zmp}
--h       FLOAT  CoM height (m)               [default: 0.3]
--maxiter INT    IK BFGS max iterations       [default: 5]
```

You can externalize the footstep plan into a JSON and load it at runtime.

---

## 🔧 Dependencies & Versions

|     Package     | Version | Notes                     |
| :-------------: | :-----: | :------------------------ |
|     `numpy`     |  ≥1.22  | Arrays, numerics          |
|     `scipy`     |   ≥1.8  | CARE solver, optimization |
|   `pinocchio`   |   ≥2.5  | Kinematics & SE(3) math   |
|     `mujoco`    |   2.X   | Physics engine            |
| `mujoco.viewer` |   2.X   | Real‐time visualization   |
|   `matplotlib`  |   ≥3.5  | Plotting post‐run data    |

Install all via:

```bash
pip install -r requirements.txt
```

Or with Conda: `conda env create -f environment.yml`

---

## 🛠️ Troubleshooting

* **MuJoCo key not found**: place `mjkey.txt` in `~/.mujoco/` and set `LD_LIBRARY_PATH`.
* **Pinocchio import error**: ensure `pinocchio` and its eigen/boost dependencies are installed.
* **Actuator mapping**: joint names in MuJoCo must match URDF; adjust prefixes in `walk_controller.py`.

---

## 🚀 Extensions & Research Directions

* Replace finite‐difference IK with analytic Jacobian solves.
* Incorporate whole‐body torque optimization (inverse dynamics).
* Add push recovery via step modification.
* Extend to 3D LIPM or nonlinear pendulum dynamics.
* Integrate reflexive ankle or arm swing controllers.

---

## 🙏 Acknowledgments & References

* Pratt, Jerry, and Goswami. “Capture Point: A Step toward Humanoid Push Recovery.” RSS 2006.
* Kajita et al., “Biped walking pattern generation by using preview control of zero‐moment point” ICRA 2003.
* Pinocchio: [https://stack-of-tasks.github.io/pinocchio/](https://stack-of-tasks.github.io/pinocchio/)
* MuJoCo: [https://mujoco.org](https://mujoco.org)

---

## 🤝 Contributing

Contributions, issues, and feature requests are welcome! Please use GitHub issues and pull requests.

1. Fork the repo
2. Create a branch (`git checkout -b feature/YourFeature`)
3. Commit your changes (`git commit -m 'Add feature'`)
4. Push and open a PR

---

## 📄 License

This project is released under the **MIT License** – see [LICENSE](LICENSE).

Happy walking! 🚶
