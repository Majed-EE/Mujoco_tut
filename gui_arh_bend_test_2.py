import mujoco as mj
from mujoco.glfw import glfw
import numpy as np
import time as T
import math
import xml.etree.ElementTree as ET
xml_path = 'arh_soft_scene1.xml' #xml file (assumes this is in the same folder as this file)
# simend = 100 #simulation time
print_camera_config = 0
model = mj.MjModel.from_xml_path(xml_path)
data = mj.MjData(model)


N = 500
simend=N


tree = ET.parse(xml_path)
root = tree.getroot()
body_dict = {}
joint_dict={}
body_id=0
joint_flag=False
for body in root.findall('.//body'):
        body_name = body.get('name')
        if body_name is None:
            continue
        # body.findall('joint')
        for joint in body.findall('joint'):
            joint_flag=True
            joint_name = joint.get('name')
            # print(joint_name)
            body_dict[body_id]={"body_name":body_name,
                               "body_id":body_id,
                                "joint_name": joint.get('name')
            # "joint_type" : joint.get('type'),
            # # joint_pos = joint.get('pos'),
            # # joint_axis = joint.get('axis')
            # "joint_range" : joint.get('range')
            }
        if not joint_flag: # if link is present
            body_dict[body_id]={"body_name":body_name,
                               "body_id":body_id}
        joint_flag=False
            
        body_id+=1

for key, value in enumerate(body_dict):
    if "joint_name" in body_dict[key]:
        # print(body_dict[key]["joint_name"])
        actuator_name=body_dict[key]["joint_name"].replace('j', 'a')
        body_dict[key]["actuator_name"]=actuator_name



actuator_list=[["ffa0",0,8],["ffa1",1,9],["ffa2",2,10],["ffa3",3,11],
           ["mfa0",4,4],["mfa1",5,5],["mfa2",6,6],["mfa3",7,7],
        ["rfa0",8,0],["rfa1",9,1],["rfa2",10,2],["rfa3",11,3],
           ["tha0",12,12],["tha1",13,13],["tha2",14,14],["tha3",15,15]]
def actuator_key(act_name):

    # index_found=False
    for index in range(len(actuator_list)):
        if actuator_list[index][0]==act_name:
            # print(index)
            return index
        
for key,value in enumerate(body_dict):
    if 'actuator_name' in body_dict[key].keys():
        # print(body_dict[key]['actuator_name'])        
        ind=actuator_key(body_dict[key]['actuator_name'])
        print(actuator_list[ind])
        body_dict[key]['actuator_id']=actuator_list[ind][1]
        body_dict[key]['joint_id']=actuator_list[ind][2]




















def setPositionControl(joint_id,angle):
    q_start = 0
    q_end = math.radians(angle); # ending angle # 90 degrees-------------------
    q = np.linspace(q_start,q_end,N)
    data.qpos[joint_id] = q_start;
    # print(q)
    return q


 

joint_id=6
q=setPositionControl(joint_id,80)



def positionControlArray(list_joint_id,list_target_angle):
    pass


def set_position_servo(actuator_no,bend_angle,kp=10):
    
    model.actuator_gainprm[actuator_no,0]=kp
    model.actuator_biasprm[actuator_no,1]=-kp
    data.ctrl=math.radians(bend_angle)





def actuator_name2id(actuator_name):
    actuator_id=None
    for key, value in enumerate(body_dict):
         if "actuator_name" in body_dict[key]:
            if body_dict[key]["actuator_name"]==actuator_name:
            # actuator_name=body_dict[key]["joint_name"].replace('j', 'a')
                actuator_id=body_dict[key]["actuator_id"]
                return actuator_id





def controller(model,data):
    # spring like behaviour
    actuator_name="ffa1"
    actuator_id=actuator_name2id(actuator_name)
    set_position_servo(actuator_id,90)



#set the controller
mj.set_mjcb_control(controller)



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



joint_id=13

# while not glfw.window_should_close(window):
for x in range(simend):
 
    step_start = T.time()
    mj.mj_step(model, data)
    joint_angles = np.copy(data.qpos)
    joint_torques = np.copy(data.qfrc_actuator)
    # data.qpos[joint_id] = q[x];
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
