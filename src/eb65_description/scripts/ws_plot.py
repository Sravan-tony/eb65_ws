import mujoco
import mujoco.viewer
import numpy as np
import time
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

XML_PATH = "/home/ros/eb65_ws/src/eb65_description/model/mujoco/scene.xml"

def main():
    model = mujoco.MjModel.from_xml_path(XML_PATH)
    data = mujoco.MjData(model)
    ee_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "gear_link")
    body_names = ["base_rotation_link", "shoulder_right_link", "elbow_link", 
                  "elbow_rotation_link", "wrist_link", "gear_link"]
    body_ids = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name) for name in body_names]

    steps_per_point = 100
    gripper_step = 0.05
    target_pos = np.array([0.0, 0.0, 0.0])
    plt.ion()
    fig = plt.figure(figsize=(10, 7))
    ax = fig.add_subplot(111, projection='3d')

    with mujoco.viewer.launch_passive(model, data) as viewer:
        while viewer.is_running():
            ax.clear()
            x_coords = [data.xpos[bid][0] for bid in body_ids]
            y_coords = [data.xpos[bid][1] for bid in body_ids]
            z_coords = [data.xpos[bid][2] for bid in body_ids]
            ax.plot(x_coords, y_coords, z_coords, '-o', linewidth=3, markersize=8, label="Robot Arm")
            ax.scatter(target_pos[0], target_pos[1], target_pos[2], c='r', s=100, label="Target")
            ax.scatter(x_coords[-1], y_coords[-1], z_coords[-1], c='g', s=100, label="End Effector")
            ax.set_xlim([-1, 1]); ax.set_ylim([-1, 1]); ax.set_zlim([0, 1])
            ax.set_xlabel('X'); ax.set_ylabel('Y'); ax.set_zlabel('Z')
            ax.set_title("EB65 Workspace Visualization")
            ax.legend()
            plt.pause(0.001)

            user_input = input("\nTarget X (o/c/exit or float): ").lower().strip()
            if user_input == 'exit': break
            if user_input == 'o':
                data.qpos[6] = np.clip(data.qpos[6] + gripper_step, model.jnt_range[6][0], model.jnt_range[6][1])
            elif user_input == 'c':
                data.qpos[6] = np.clip(data.qpos[6] - gripper_step, model.jnt_range[6][0], model.jnt_range[6][1])
            else:
                try:
                    tx = float(user_input)
                    ty = float(input("Target Y: "))
                    tz = float(input("Target Z: "))
                    target_pos = np.array([tx, ty, tz])
                    start_pos = np.copy(data.xpos[ee_id])
                    for s in range(1, steps_per_point + 1):
                        alpha = s / steps_per_point
                        waypoint = start_pos + alpha * (target_pos - start_pos)

                        for _ in range(5):
                            error = waypoint - data.xpos[ee_id]
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
                        if s % 10 == 0:
                            ax.clear()
                            x_c = [data.xpos[bid][0] for bid in body_ids]
                            y_c = [data.xpos[bid][1] for bid in body_ids]
                            z_c = [data.xpos[bid][2] for bid in body_ids]
                            ax.plot(x_c, y_c, z_c, '-o')
                            ax.scatter(target_pos[0], target_pos[1], target_pos[2], c='r', s=100)
                            ax.set_xlim([-1, 1]); ax.set_ylim([-1, 1]); ax.set_zlim([0, 1])
                            plt.pause(0.001)
                        
                        time.sleep(0.01)
                except ValueError:
                    print("Invalid input.")

            mujoco.mj_step(model, data)
            viewer.sync()

if __name__ == "__main__":
    main()