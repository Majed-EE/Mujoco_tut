import mujoco as mj
from mujoco.glfw import glfw
import numpy as np
import time as T
import math

xml_path = 'arh_soft_scene1.xml' #xml file (assumes this is in the same folder as this file)
# simend = 100 #simulation time
print_camera_config = 0
model = mj.MjModel.from_xml_path(xml_path)
data = mj.MjData(model)


N = 500
simend=N

def setPositionControl(joint_id,angle):
    q_start = 0
    q_end = math.radians(angle); # ending angle # 90 degrees-------------------
    q = np.linspace(q_start,q_end,N)
    data.qpos[joint_id] = q_start;
    # print(q)
    return q


 

joint_id=6
q=setPositionControl(joint_id,80)





def init_controller(model,data):
    #initialize the controller here. This function is called once, in the beginning
    pass

def controller(model, data):
    #put the controller here. This function is called inside the simulation.
    pass


if not glfw.init():
    raise Exception("GLFW can't be initialized")

window = glfw.create_window(1200, 900, "Demo", None, None)
if not window:
    glfw.terminate()
    raise Exception("GLFW window can't be created")

glfw.make_context_current(window)
glfw.swap_interval(1)


cam = mj.MjvCamera()
cam2 = mj.MjvCamera()
opt = mj.MjvOption()
scene = mj.MjvScene(model, maxgeom=10000)
context = mj.MjrContext(model, mj.mjtFontScale.mjFONTSCALE_150)


mj.mjv_defaultFreeCamera(model, cam)
mj.mjv_defaultFreeCamera(model, cam2)
mj.mjv_defaultOption(opt)


cam.azimuth = 90
cam.elevation = -50
cam.distance = 3
cam.lookat = np.array([0.0, 0.0, 0.0])

cam2.azimuth = 270
cam2.elevation = -50
cam2.distance = 4
cam2.lookat = np.array([0.0, 0.0, 0.0])


# while not glfw.window_should_close(window):
for x in range(simend):
 
    step_start = T.time()
    mj.mj_step(model, data)
    joint_angles = np.copy(data.qpos)
    joint_torques = np.copy(data.qfrc_actuator)
    data.qpos[joint_id] = q[x];
    # data.qpos[1] = q1[i];

    print("XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX")
    print(f"angle {math.degrees(joint_angles[joint_id])}")
    
    # print(f"torque {joint_torques[joint_id]}")
    print("XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX")


    viewport1 = mj.MjrRect(0, 0, 600, 900)
    mj.mjv_updateScene(model, data, opt, None, cam, mj.mjtCatBit.mjCAT_ALL, scene)
    mj.mjr_render(viewport1, scene, context)


    viewport2 = mj.MjrRect(600, 0, 600, 900)
    mj.mjv_updateScene(model, data, opt, None, cam2, mj.mjtCatBit.mjCAT_ALL, scene)
    mj.mjr_render(viewport2, scene, context)


    glfw.swap_buffers(window)
    glfw.poll_events()

glfw.terminate()
