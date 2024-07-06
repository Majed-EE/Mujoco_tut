model_path = r"wonik_allegro/left_hand.xml"

import mujoco
import numpy as np

# Load the Allegro Hand model
# model_path = 'path/to/allegro_hand.xml'  # replace with your actual path
model = mujoco.MjModel.from_xml_path(model_path)
data = mujoco.MjData(model)

# Optional: Create a viewer for visualization
def render_callback(sim):
    mujoco.mjv_updateScene(sim.model, sim.data, sim.opt, sim.pert, sim.cam, mujoco.mjvScene())
    mujoco.mjr_render(sim.viewport, sim.scn, sim.con)

viewer = mujoco.MjViewer(model, data, render_callback)

# Run the simulation for a few steps
for _ in range(100):  # number of simulation steps
    mujoco.mj_step(model, data)
    viewer.render()

    # Get joint angles (qpos) and torques (qfrc)
    joint_angles = np.copy(data.qpos)
    joint_torques = np.copy(data.qfrc_actuator)

    print("Joint Angles:", joint_angles)
    print("Joint Torques:", joint_torques)

# Close the viewer (optional)
viewer.close()

