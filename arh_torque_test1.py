model_path = r"wonik_allegro/left_hand.xml"

import mujoco
import numpy as np
import mujoco.viewer as mj_viewer
import time

model = mujoco.MjModel.from_xml_path(model_path)
data = mujoco.MjData(model)


def render_callback(sim):
    mujoco.mjv_updateScene(sim.model, sim.data, sim.opt, sim.pert, sim.cam, mujoco.mjvScene())
    mujoco.mjr_render(sim.viewport, sim.scn, sim.con)

with mj_viewer.launch_passive(model, data) as viewer:

  start = time.time()
  while viewer.is_running() and time.time() - start < 30:
    step_start = time.time()

    mujoco.mj_step(model, data)
    # joint_angles = np.copy(data.qpos)
    joint_torques = np.copy(data.qfrc_actuator)

    # print("Joint Angles:", joint_angles)
    print("Joint Torques:", joint_torques)

    with viewer.lock():
      viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = int(data.time % 2)

    viewer.sync()

    time_until_next_step = model.opt.timestep - (time.time() - step_start)
    if time_until_next_step > 0:
      time.sleep(time_until_next_step)




# viewer = mujoco.Viewer(model, data, render_callback)

# # Run the simulation for a few steps
# for _ in range(100):  # number of simulation steps
#     mujoco.mj_step(model, data)
#     viewer.render()

#     # Get joint angles (qpos) and torques (qfrc)
#     joint_angles = np.copy(data.qpos)
#     joint_torques = np.copy(data.qfrc_actuator)

#     print("Joint Angles:", joint_angles)
#     print("Joint Torques:", joint_torques)

# # Close the viewer (optional)
# viewer.close()

