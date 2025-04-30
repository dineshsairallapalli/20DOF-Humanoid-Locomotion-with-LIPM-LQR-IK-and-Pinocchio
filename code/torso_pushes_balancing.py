import mujoco as mj
import numpy as np
import mujoco.viewer as viewer
from matplotlib import pyplot as plt

# Mujoco file name (update the path if needed)
xml = r"C:\Users\ralla\mujoco\mujoco_menagerie-main\robotis_op3\scene.xml"

# Load model and initialize physics data
model = mj.MjModel.from_xml_path(xml)
data = mj.MjData(model)

# Define leg actuator names (based on op3.xml)
leg_actuators = [
    "l_hip_yaw_act", "l_hip_roll_act", "l_hip_pitch_act",
    "l_knee_act", "l_ank_pitch_act", "l_ank_roll_act",
    "r_hip_yaw_act", "r_hip_roll_act", "r_hip_pitch_act",
    "r_knee_act", "r_ank_pitch_act", "r_ank_roll_act"
]

# Create a mapping from actuator name to actuator id
actuator_ids = {name: mj.mj_name2id(model, mj.mjtObj.mjOBJ_ACTUATOR, name)
                for name in leg_actuators}
print("Leg actuator mapping:", actuator_ids)

# Simulation parameters
duration = 10.0       # total simulation time in seconds
freq = 0.5            # frequency of the sinusoidal signal (Hz)
amplitude = 0.5       # amplitude of control commands

# Lists for storing a couple of state variables (for plotting)
q0s = []  # First element of qpos (e.g. could be a base or joint state)
q1s = []  # Second element of qpos
q0_vs = []  # First element of qvel
q1_vs = []  # Second element of qvel

# Launch the viewer in passive mode using a context manager.
with viewer.launch_passive(model, data) as v:
    start_time = data.time
    # Run the simulation loop until the viewer is closed or duration is reached.
    while v.is_running() and (data.time - start_time < duration):
        
        t = data.time
        # Apply sinusoidal control signals:
        # Left leg actuators: sin(2πft)
        # Right leg actuators: sin(2πft + π) for a phase shift.
        for name, act_id in actuator_ids.items():
            if name.startswith("l_"):
                data.ctrl[act_id] = amplitude * np.sin(2 * np.pi * freq * t)
            else:
                data.ctrl[act_id] = amplitude * np.sin(2 * np.pi * freq * t + np.pi)
        
        # Record a couple of state values for plotting
        q0s.append(data.qpos[0])
        q1s.append(data.qpos[1])
        q0_vs.append(data.qvel[0])
        q1_vs.append(data.qvel[1])
        
        # Advance the simulation one time-step
        mj.mj_step(model, data)
        # Sync the viewer with the updated simulation state
        v.sync()

    # Close the viewer once simulation is complete.
    v.close()

# Plot the state variables (qpos)
plt.figure()
plt.plot(q0s, label='qpos[0]')
plt.plot(q1s, label='qpos[1]')
plt.title("Joint Positions Over Time")
plt.xlabel("Time Steps")
plt.ylabel("Position")
plt.legend()
plt.show()

# Plot the state variables (qvel)
plt.figure()
plt.plot(q0_vs, label='qvel[0]')
plt.plot(q1_vs, label='qvel[1]')
plt.title("Joint Velocities Over Time")
plt.xlabel("Time Steps")
plt.ylabel("Velocity")
plt.legend()
plt.show()
