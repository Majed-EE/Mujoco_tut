import mujoco
import mujoco.viewer
import glfw

# Load the MuJoCo model from the XML file
model = mujoco.MjModel.from_xml_path('left_hand.xml')
data = mujoco.MjData(model)

# Initialize GLFW to manage multiple windows
if not glfw.init():
    raise Exception("GLFW can't be initialized")

# Function to render from a specific camera in a new window
def render_from_camera(model, data, camera_name, window_title):
    window = glfw.create_window(800, 600, window_title, None, None)
    if not window:
        glfw.terminate()
        raise Exception("GLFW window can't be created")

    glfw.make_context_current(window)
    viewer = mujoco.viewer.MjViewerBasic(model, data)
    cam_id = model.camera_name2id(camera_name)
    viewer.cam.fixedcamid = cam_id
    viewer.cam.type = mujoco.mjtCamera.mjCAMERA_FIXED

    while not glfw.window_should_close(window):
        mujoco.mj_step(model, data)
        viewer.render()
        glfw.swap_buffers(window)
        glfw.poll_events()

    glfw.destroy_window(window)

# Simulate and render from the first camera in a new thread
import threading
thread_cam1 = threading.Thread(target=render_from_camera, args=(model, data, 'cam1', 'Camera 1'))
thread_cam1.start()

# Reset simulation data if needed
data = mujoco.MjData(model)

# Simulate and render from the second camera in a new thread
thread_cam2 = threading.Thread(target=render_from_camera, args=(model, data, 'cam2', 'Camera 2'))
thread_cam2.start()

# Wait for both threads to finish
thread_cam1.join()
thread_cam2.join()

# Terminate GLFW when done
glfw.terminate()
