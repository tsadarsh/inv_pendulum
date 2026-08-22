import mujoco, mujoco.viewer, time
import glfw
import numpy as np

model = mujoco.MjModel.from_xml_path("scene.xml")
data = mujoco.MjData(model)

joint_adr = model.jnt_qposadr[model.joint('pend_pin').id]
data.qpos[joint_adr] = 0.05  # tiny nudge off vertical, no actuator present

target = 1.0
kv = 100
kd = 10
ki = 1.0

def key_callback(keycode):
    global target
    if keycode == glfw.KEY_UP: target = 1.3
    if keycode == glfw.KEY_DOWN: target = -1.3
    if keycode == glfw.KEY_LEFT: target = 0.0

with mujoco.viewer.launch_passive(model, data, key_callback=key_callback) as viewer:
    dt = model.opt.timestep
    while viewer.is_running():
        mujoco.mj_step(model, data)
        error = target - data.qpos[joint_adr]
        error_dot = -data.qvel[joint_adr]
        torque = kv * error + kd * error_dot + ki
        data.ctrl[model.actuator('pend_pin').id] = np.clip(torque, -5, 5)
        # data.qvel[joint_adr] = kv * (target - data.qpos[joint_adr])
        print(data.qpos[joint_adr])
        viewer.sync()
        time.sleep(dt)