import mujoco, mujoco.viewer, time

model = mujoco.MjModel.from_xml_path("robot.xml")
data = mujoco.MjData(model)

joint_adr = model.jnt_qposadr[model.joint('pend_pin').id]
data.qpos[joint_adr] = 0.05  # tiny nudge off vertical, no actuator present

with mujoco.viewer.launch_passive(model, data) as viewer:
    dt = model.opt.timestep
    while viewer.is_running():
        mujoco.mj_step(model, data)
        viewer.sync()
        time.sleep(dt)