import mujoco
import mujoco.viewer
import numpy as np
import time

XML_PATH = "/home/ros/eb65_ws/src/eb65_description/model/mujoco/scene.xml"

def main():
    model = mujoco.MjModel.from_xml_path(XML_PATH)
    data = mujoco.MjData(model)
    ee_name = "gear_link"
    ee_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, ee_name)
    steps_per_point = 100
    tol = 0.001
    gripper_step = 0.05

    print("\n" + "="*60)
    print("EB65 SMOOTH IK + GRIPPER CONTROL")
    print("="*60)
    print("Commands: Enter X, Y, Z or 'o' to Open, 'c' to Close")
    print("="*60)

    with mujoco.viewer.launch_passive(model, data) as viewer:
        while viewer.is_running():
            user_input = input("\nTarget X (or o/c): ").lower().strip()
            
            if user_input == 'o':
                data.qpos[6] = np.clip(data.qpos[6] + gripper_step, model.jnt_range[6][0], model.jnt_range[6][1])
                mujoco.mj_step(model, data)
                viewer.sync()
                print("Gripper Opening...")
                continue
            elif user_input == 'c':
                data.qpos[6] = np.clip(data.qpos[6] - gripper_step, model.jnt_range[6][0], model.jnt_range[6][1])
                mujoco.mj_step(model, data)
                viewer.sync()
                print("Gripper Closing...")
                continue
            
            try:
                tx = float(user_input)
                ty = float(input("Target Y: "))
                tz = float(input("Target Z: "))
                final_target = np.array([tx, ty, tz])
            except ValueError:
                print("Invalid Input")
                continue

            start_pos = np.copy(data.xpos[ee_id])
            for s in range(1, steps_per_point + 1):
                if not viewer.is_running(): break
                alpha = s / steps_per_point
                waypoint = start_pos + alpha * (final_target - start_pos)

                for _ in range(5):
                    current_pos = data.xpos[ee_id]
                    error = waypoint - current_pos
                    jacp = np.zeros((3, model.nv))
                    jacr = np.zeros((3, model.nv))
                    mujoco.mj_jacBody(model, data, jacp, jacr, ee_id)
                    J = jacp[:, :6]
                    dq = np.linalg.pinv(J) @ error
                    data.qpos[:6] += dq * 0.5
                    for i in range(6):
                        data.qpos[i] = np.clip(data.qpos[i], model.jnt_range[i][0], model.jnt_range[i][1])
                    mujoco.mj_step(model, data)
                viewer.sync()
                time.sleep(0.02) 
            print("Position Reached.")

if __name__ == "__main__":
    main()