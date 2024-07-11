# 
import mujoco as mj
from mujoco.glfw import glfw
import numpy as np
import os
import time as T
####
xml_path = 'manipulator.xml' #xml file (assumes this is in the same folder as this file)
simend = 20 #simulation time
print_camera_config = 0 #set to 1 to print camera config
                        #this is useful for initializing view of the model)


# For callback functions
button_left = False
button_middle = False
button_right = False
lastx = 0
lasty = 0



def init_controller(model,data):
    #initialize the controller here. This function is called once, in the beginning
    pass

def controller(model, data):
    #put the controller here. This function is called inside the simulation.
    pass

def keyboard(window, key, scancode, act, mods):
    if act == glfw.PRESS and key == glfw.KEY_BACKSPACE:
        mj.mj_resetData(model, data)
        mj.mj_forward(model, data)

def mouse_button(window, button, act, mods):
    # update button state
    global button_left
    global button_middle
    global button_right

    button_left = (glfw.get_mouse_button(
        window, glfw.MOUSE_BUTTON_LEFT) == glfw.PRESS)
    button_middle = (glfw.get_mouse_button(
        window, glfw.MOUSE_BUTTON_MIDDLE) == glfw.PRESS)
    button_right = (glfw.get_mouse_button(
        window, glfw.MOUSE_BUTTON_RIGHT) == glfw.PRESS)

    # update mouse position
    glfw.get_cursor_pos(window)

def mouse_move(window, xpos, ypos):
    # compute mouse displacement, save
    global lastx
    global lasty
    global button_left
    global button_middle
    global button_right

    dx = xpos - lastx
    dy = ypos - lasty
    lastx = xpos
    lasty = ypos

    # no buttons down: nothing to do
    if (not button_left) and (not button_middle) and (not button_right):
        return

    # get current window size
    width, height = glfw.get_window_size(window)

    # get shift key state
    PRESS_LEFT_SHIFT = glfw.get_key(
        window, glfw.KEY_LEFT_SHIFT) == glfw.PRESS
    PRESS_RIGHT_SHIFT = glfw.get_key(
        window, glfw.KEY_RIGHT_SHIFT) == glfw.PRESS
    mod_shift = (PRESS_LEFT_SHIFT or PRESS_RIGHT_SHIFT)

    # determine action based on mouse button
    if button_right:
        if mod_shift:
            action = mj.mjtMouse.mjMOUSE_MOVE_H
        else:
            action = mj.mjtMouse.mjMOUSE_MOVE_V
    elif button_left:
        if mod_shift:
            action = mj.mjtMouse.mjMOUSE_ROTATE_H
        else:
            action = mj.mjtMouse.mjMOUSE_ROTATE_V
    else:
        action = mj.mjtMouse.mjMOUSE_ZOOM

    mj.mjv_moveCamera(model, action, dx/height,
                      dy/height, scene, cam)

def scroll(window, xoffset, yoffset):
    action = mj.mjtMouse.mjMOUSE_ZOOM
    mj.mjv_moveCamera(model, action, 0.0, -0.05 *
                      yoffset, scene, cam)


#get the full path
# dirname = os.path.dirname(os.path.abspath(__file__))
dirname = os.path.dirname(os.path.abspath(os.getcwd()))

abspath = os.path.join(dirname + "/" + xml_path)
# xml_path = abspath

# MuJoCo data structures
model = mj.MjModel.from_xml_path(xml_path)  # MuJoCo model
data = mj.MjData(model)                # MuJoCo data
cam = mj.MjvCamera()                        # Abstract camera
opt = mj.MjvOption()   

k=[model.geom(i).name for i in range(model.ngeom)]
print(k)

print(data.time)
print(data.qpos) # gives initial and final state of what
print(data.qvel)
print(data.geom_xpos)
# qvel, qpos, qtime, data.geom_xpos (wrold frame position)
# print(data.qvel)

# # Init GLFW, create window, make OpenGL context current, request v-sync
# glfw.init()
# window = glfw.create_window(1200, 900, "Demo", None, None)
# glfw.make_context_current(window)
# glfw.swap_interval(1)

# # initialize visualization data structures

# path_arh=r"wonik_allegro/scene_left.xml"
# arh_model=mj.MjModel.from_xml_path(path_arh)
# # model = mujoco.MjModel.from_xml_string(XML, ASSETS)
# data_arh = mj.MjData(arh_model)

# mj.mjv_defaultCamera(cam)
# mj.mjv_defaultOption(opt)
# scene = mj.MjvScene(arh_model, maxgeom=10000)
# # T.sleep(5)
# i=0
# context = mj.MjrContext(arh_model, mj.mjtFontScale.mjFONTSCALE_150.value) # some sort of releasing contextg
# # while data.time < 1:
# #   mj.mj_step(arh_model, data_arh)
# #   i=i+1
# #   print(data_arh.geom_xpos)


# print(i)


#initialize the controller
init_controller(model,data)

#set the controller
mj.set_mjcb_control(controller)
#initial manipulator config
N = 500
q0_start = 0; # starting angle
import math

q0_end = 1.57; # edding angle #90 degrees
q1_start = 0;
q1_end = -2*3.14;
q0 = np.linspace(q0_start,q0_end,N)
q1 = np.linspace(q1_start,q1_end,N)

#initialize
data.qpos[0] = q0_start;
data.qpos[1] = q1_start
i = 0;
time = 0
dt = 0.001;
