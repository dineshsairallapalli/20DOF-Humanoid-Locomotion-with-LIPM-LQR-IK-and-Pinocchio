import os
import numpy as np
import pinocchio as pin
from scipy.linalg import solve_continuous_are
from scipy.optimize import minimize
import mujoco as mj
import mujoco.viewer as viewer

# ------------------------------------------------------
# 1) FootSteps: define the footstep sequence
# ------------------------------------------------------
class FootSteps:
    def __init__(self, left0, right0):
        self.phases = []    # list of (duration, support, target)
        self.left0 = np.array(left0)
        self.right0 = np.array(right0)
        self.times = [0.]

    def add_phase(self, duration, support, target=None):
        self.phases.append((duration, support, np.array(target) if target is not None else None))
        self.times.append(self.times[-1] + duration)

    def get_phase_index(self, t):
        # find i s.t. times[i] <= t < times[i+1]
        for i in range(len(self.times)-1):
            if self.times[i] <= t < self.times[i+1]: return i
        return len(self.times)-2

    def get_phase(self, t):
        i = self.get_phase_index(t)
        return self.phases[i], t - self.times[i]

    def get_support_positions(self, t):
        # returns (stance, swing_target) positions
        (dur, sup, target), tau = self.get_phase(t)
        # what are current foot positions? We can track last settled positions
        # For simplicity assume left0/right0 for initial, and steps in order
        # Precompute foot targets for each step
        # ...
        pass

# ------------------------------------------------------
# 2) ZmpRef: piecewise-affine ZMP reference
# ------------------------------------------------------
class ZmpRef:
    def __init__(self, footsteps: FootSteps):
        self.footsteps = footsteps

    def __call__(self, t):
        # Determine phase and compute ZMP
        (dur, sup, target), tau = self.footsteps.get_phase(t)
        if sup == 'none':
            # double support: interpolate between last foot and next
            # TODO: implement interpolation
            return np.zeros(2)
        elif sup == 'left':
            # ZMP under left foot
            return np.array(self.footsteps.left0)
        else:
            return np.array(self.footsteps.right0)

# ------------------------------------------------------
# 3) LIPM + LQR CoM planner
# ------------------------------------------------------
class ComRef:
    def __init__(self, zmp_ref, h=0.3, g=9.81):
        self.zmp_ref = zmp_ref
        self.h = h
        self.omega = np.sqrt(g/h)
        # build LQR gain
        A = np.array([[0, 1], [self.omega**2, 0]])
        B = np.array([[0], [-self.omega**2]])
        Q = np.diag([1e3, 1.])
        R = np.array([[1e-6]])
        P = solve_continuous_are(A, B, Q, R)
        self.K = np.linalg.inv(R) @ B.T @ P
        self.state = np.zeros((2,))

    def update(self, dt, t):
        # simple Euler integration of x' = A x + B r_zmp + K*(x - x_ref)
        x = self.state
        ref_z = self.zmp_ref(t)
        A = np.array([[0, 1], [self.omega**2, 0]])
        B = np.array([[0], [-self.omega**2]])
        # state reference if CoM should track ZMP exactly
        x_ref = np.array([ref_z[0], 0.])  # forward direction only
        u_fb = -self.K @ (x - x_ref)
        xdot = A @ x + B.flatten()*ref_z[0] + B.flatten()*u_fb
        self.state = x + xdot*dt
        return self.state.copy()

# ------------------------------------------------------
# 4) Swing foot trajectory (cubic Hermite spline)
# ------------------------------------------------------
class SwingFootTrajectory:
    def __init__(self, p0, p1, height=0.05):
        self.p0 = np.array(p0)
        self.p1 = np.array(p1)
        self.height = height

    def eval(self, s):
        # s in [0,1]
        # horizontal interp
        pos = (1-s)*self.p0 + s*self.p1
        # vertical cubic: 0 at s=0, 1 at s=1, peak at s=0.5
        z = 4*self.height*s*(1-s)
        return np.array([pos[0], pos[1], z])

# ------------------------------------------------------
# 5) Inverse Kinematics via Pinocchio + BFGS
# ------------------------------------------------------
def inverse_kinematics(model, data, targets, q_init, maxiter=5):
    # targets: list of (frame_id, desired_placement)
    def cost(q):
        pin.forwardKinematics(model, data, q)
        pin.updateFramePlacements(model, data)
        e = 0
        for fid, Mdes in targets:
            M = data.oMf[fid]
            diff = pin.log6(Minv(M) @ Mdes)
            e += np.dot(diff, diff)
        return e

    def jac(q):
        pin.forwardKinematics(model, data, q)
        pin.computeJointJacobians(model, data, q)
        pin.updateFramePlacements(model, data)
        J_total = np.zeros((len(q),))
        # approximate gradient via finite-diff or use analytical jac
        eps = 1e-6
        grad = np.zeros_like(q)
        for i in range(len(q)):
            dq = np.zeros_like(q); dq[i] = eps
            grad[i] = (cost(q+dq) - cost(q-dq))/(2*eps)
        return grad

    res = minimize(cost, q_init, jac=jac, method='BFGS', options={'maxiter': maxiter, 'disp': False})
    return res.x

# ------------------------------------------------------
# 6) Main simulation loop
# ------------------------------------------------------
if __name__ == '__main__':
    # Load MuJoCo model
    xml = r"C:\Users\ralla\mujoco\mujoco_menagerie-main\robotis_op3\scene.xml"
    model = mj.MjModel.from_xml_path(xml)
    data  = mj.MjData(model)

    # Initialize Pinocchio model
    pin_model = pin.buildModelFromUrdf(os.path.expandvars(xml.replace('scene.xml','op3.xml')))
    pin_data  = pin_model.createData()

    # Define footstep plan
    footsteps = FootSteps([0.0,  0.1], [0.0, -0.1])
    footsteps.add_phase(.3, 'none')
    footsteps.add_phase(.7, 'left',  [0.1,  0.1])
    footsteps.add_phase(.1, 'none')
    footsteps.add_phase(.7, 'right', [0.2, -0.1])
    footsteps.add_phase(.1, 'none')
    footsteps.add_phase(.7, 'left',  [0.3,  0.1])
    footsteps.add_phase(.1, 'none')
    footsteps.add_phase(.7, 'right', [0.4, -0.1])
    footsteps.add_phase(.1, 'none')
    footsteps.add_phase(.7, 'left',  [0.5,  0.1])
    footsteps.add_phase(.1, 'none')
    footsteps.add_phase(.7, 'right', [0.5, -0.1])
    footsteps.add_phase(.5, 'none')

    # Instantiate references
    zmp_ref = ZmpRef(footsteps)
    com_ref = ComRef(zmp_ref)

    # Initial configuration
    q = pin.neutral(pin_model)

    ctrl_dt = 0.01  # 100 Hz
    sim_time = footsteps.times[-1]

    # Launch MuJoCo viewer
    with viewer.launch_passive(model, data) as v:
        t = 0.0
        while v.is_running() and t < sim_time:
            # 1) Update CoM
            com_state = com_ref.update(ctrl_dt, t)
            # 2) Determine foot trajectory and stance
            # TODO: implement foot target extraction
            stance_frame = pin_model.getFrameId('l_ankle_link')
            swing_frame  = pin_model.getFrameId('r_ankle_link')
            swing_traj = SwingFootTrajectory([0,0], [0.2,-0.1], height=0.05)
            swing_pos = swing_traj.eval((t%0.7)/0.7)
            M_des_st = pin.SE3.Identity()
            M_des_sw = pin.SE3(pin.utils.rotate('z',0), np.array([swing_pos[0],swing_pos[1],swing_pos[2]]))

            # 3) IK solve
            targets = [(stance_frame, M_des_st), (swing_frame, M_des_sw)]
            q = inverse_kinematics(pin_model, pin_data, targets, q, maxiter=5)

            # 4) Apply commands to MuJoCo
            for j, name in enumerate(model.joint_names):
                # map pinocchio joint order to MuJoCo actuators
                try:
                    aid = mj.mj_name2id(model, mj.mjtObj.mjOBJ_ACTUATOR, name + '_act')
                    data.ctrl[aid] = q[j]
                except Exception:
                    pass

            # 5) Step simulation
            mj.mj_step(model, data)

            # 6) Sync viewer and time
            v.sync()
            t += ctrl_dt

        v.close()
