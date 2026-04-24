import mujoco
import mujoco.viewer
import numpy as np
import time

XML_PATH = "/home/ros/eb65_ws/src/eb65_description/model/mujoco/scene.xml"

def dh_matrix(theta, d, a, alpha):
    return np.array([
        [np.cos(theta), -np.sin(theta)*np.cos(alpha),  np.sin(theta)*np.sin(alpha), a*np.cos(theta)],
        [np.sin(theta),  np.cos(theta)*np.cos(alpha), -np.cos(theta)*np.sin(alpha), a*np.sin(theta)],
        [0,              np.sin(alpha),               np.cos(alpha),              d],
        [0,              0,                           0,                          1]
    ])

def solve_fk(q):
    T1 = dh_matrix(q[0], 0.23, 0.0, np.pi/2)
    T2 = dh_matrix(q[1], 0.0, 0.315, 0.0)
    T3 = dh_matrix(q[2], 0.047, 0.13, np.pi/2)
    T4 = dh_matrix(q[3], 0.0, 0.0, -np.pi/2)
    T5 = dh_matrix(q[4], 0.0, 0.204, np.pi/2)
    T6 = dh_matrix(q[5], 0.0, 0.04, 0.0)
    T_final = T1 @ T2 @ T3 @ T4 @ T5 @ T6
    return T_final

def main():
    model = mujoco.MjModel.from_xml_path(XML_PATH)
    data = mujoco.MjData(model)
    print("="*60)
    print("EB65 FORWARD KINEMATICS (MANUAL MATH VS MUJOCO)")
    print("="*60)
    with mujoco.viewer.launch_passive(model, data) as viewer:
        while viewer.is_running():
            step_start = time.time()
            q_current = data.qpos[:6]
            T_ee = solve_fk(q_current)
            math_pos = T_ee[:3, 3]
            body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "gear_link")
            mj_pos = data.xpos[body_id]
            
            print(f"MATH: {math_pos[0]:.3f}, {math_pos[1]:.3f}, {math_pos[2]:.3f} | "
                  f"MUJOCO: {mj_pos[0]:.3f}, {mj_pos[1]:.3f}, {mj_pos[2]:.3f}", end="\r")
            mujoco.mj_step(model, data)
            viewer.sync()
            elapsed = time.time() - step_start
            if elapsed < model.opt.timestep:
                time.sleep(model.opt.timestep - elapsed)

if __name__ == "__main__":
    main()