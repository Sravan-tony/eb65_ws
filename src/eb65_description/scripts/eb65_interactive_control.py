import mujoco
import mujoco.viewer
import numpy as np
import time

XML_PATH = "/home/ros/eb65_ws/src/eb65_description/model/mujoco/scene.xml"

key_state = {
    "base_fwd": False, "base_bwd": False,
    "shld_fwd": False, "shld_bwd": False,
    "elbw_fwd": False, "elbw_bwd": False,
    "erot_fwd": False, "erot_bwd": False,
    "wrst_fwd": False, "wrst_bwd": False,
    "gear_fwd": False, "gear_bwd": False,
    "grip_opn": False, "grip_cls": False
}

def key_callback(keycode):
    global key_state
    if keycode in [54, 326]: key_state["base_fwd"] = True       # Base: 6 and 4
    elif keycode in [52, 324]: key_state["base_bwd"] = True
    
    elif keycode in [56, 328]: key_state["shld_fwd"] = True     # Shoulder: 8 and 2
    elif keycode in [50, 322]: key_state["shld_bwd"] = True

    elif keycode in [55, 327]: key_state["elbw_fwd"] = True     # Elbow: 7 and 1
    elif keycode in [49, 321]: key_state["elbw_bwd"] = True

    elif keycode in [48, 320]: key_state["erot_fwd"] = True     # Elbow Rotation: 0 and .
    elif keycode in [46, 330]: key_state["erot_bwd"] = True

    elif keycode in [57, 329]: key_state["wrst_fwd"] = True     # Wrist: 9 and 3
    elif keycode in [51, 323]: key_state["wrst_bwd"] = True

    elif keycode in [47, 331]: key_state["gear_fwd"] = True     # Gear: / and *
    elif keycode in [42, 332]: key_state["gear_bwd"] = True

    elif keycode == 79: key_state["grip_opn"] = True            # Gripper: O (79) and C (67)
    elif keycode == 67: key_state["grip_cls"] = True

def interactive_control():
    try:
        model = mujoco.MjModel.from_xml_path(XML_PATH)
        data = mujoco.MjData(model)
    except Exception as e:
        print(f"Error: {e}")
        return

    STEP_SIZE = 0.1
    BASE_IDX = 0
    SHLD_IDX = 1
    ELBW_IDX = 2
    EROT_IDX = 3
    WRST_IDX = 4
    GEAR_IDX = 5
    GRIP_IDX = 6

    print("\n" + "="*60)
    print("ROBOT CONTROL: 7 JOINTS ACTIVE")
    print("="*60)
    print(f"Base [4/6]   : {model.actuator(BASE_IDX).name}")
    print(f"Shld [2/8]   : {model.actuator(SHLD_IDX).name}")
    print(f"Elbow [1/7]  : {model.actuator(ELBW_IDX).name}")
    print(f"E-Rot [./0]  : {model.actuator(EROT_IDX).name}")
    print(f"Wrist [3/9]  : {model.actuator(WRST_IDX).name}")
    print(f"Gear  [*/ /] : {model.actuator(GEAR_IDX).name}")
    print(f"Grip  [C / O] : {model.actuator(GRIP_IDX).name}")
    print("-" * 60)
    print("Click the MuJoCo window to focus it!")
    print("="*60 + "\n")

    with mujoco.viewer.launch_passive(model, data, key_callback=key_callback) as viewer:
        while viewer.is_running():
            step_start = time.time()
            if key_state["base_fwd"]: data.ctrl[BASE_IDX] += STEP_SIZE; key_state["base_fwd"] = False 
            if key_state["base_bwd"]: data.ctrl[BASE_IDX] -= STEP_SIZE; key_state["base_bwd"] = False
            if key_state["shld_fwd"]: data.ctrl[SHLD_IDX] += STEP_SIZE; key_state["shld_fwd"] = False 
            if key_state["shld_bwd"]: data.ctrl[SHLD_IDX] -= STEP_SIZE; key_state["shld_bwd"] = False
            if key_state["elbw_fwd"]: data.ctrl[ELBW_IDX] += STEP_SIZE; key_state["elbw_fwd"] = False 
            if key_state["elbw_bwd"]: data.ctrl[ELBW_IDX] -= STEP_SIZE; key_state["elbw_bwd"] = False
            if key_state["erot_fwd"]: data.ctrl[EROT_IDX] += STEP_SIZE; key_state["erot_fwd"] = False 
            if key_state["erot_bwd"]: data.ctrl[EROT_IDX] -= STEP_SIZE; key_state["erot_bwd"] = False
            if key_state["wrst_fwd"]: data.ctrl[WRST_IDX] += STEP_SIZE; key_state["wrst_fwd"] = False 
            if key_state["wrst_bwd"]: data.ctrl[WRST_IDX] -= STEP_SIZE; key_state["wrst_bwd"] = False
            if key_state["gear_fwd"]: data.ctrl[GEAR_IDX] += STEP_SIZE; key_state["gear_fwd"] = False 
            if key_state["gear_bwd"]: data.ctrl[GEAR_IDX] -= STEP_SIZE; key_state["gear_bwd"] = False
            if key_state["grip_opn"]: data.ctrl[GRIP_IDX] += STEP_SIZE; key_state["grip_opn"] = False 
            if key_state["grip_cls"]: data.ctrl[GRIP_IDX] -= STEP_SIZE; key_state["grip_cls"] = False

            for idx in range(7):
                data.ctrl[idx] = np.clip(
                    data.ctrl[idx], 
                    model.actuator(idx).ctrlrange[0], 
                    model.actuator(idx).ctrlrange[1]
                )

            mujoco.mj_step(model, data)
            viewer.sync()
            time.sleep(model.opt.timestep)

if __name__ == "__main__":
    interactive_control()