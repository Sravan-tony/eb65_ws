import mujoco
import mujoco.viewer
import numpy as np
import time
import sys

XML_PATH = "/home/ros/eb65_ws/src/eb65_description/model/mujoco/scene.xml"

def smooth_random_motion():
    try:
        model = mujoco.MjModel.from_xml_path(XML_PATH)
        data = mujoco.MjData(model)
    except Exception as e:
        print(f"Error loading model: {e}")
        return
    actuator_names = []
    ctrl_ranges = []
    for i in range(model.nu):
        name = model.actuator(i).name
        ctrl_min = model.actuator(i).ctrlrange[0]
        ctrl_max = model.actuator(i).ctrlrange[1]
        actuator_names.append(name)
        ctrl_ranges.append((ctrl_min, ctrl_max))
        print(f"Actuator {i}: {name} range: [{ctrl_min:.2f}, {ctrl_max:.2f}]")
    
    print("\n" + "="*60)
    print("SMOOTH SLOW RANDOM MOTION")
    print("="*60)
    
    SMOOTHING_FACTOR = 0.02
    TARGET_CHANGE_INTERVAL = 5.0
    
    print(f"Smoothing factor: {SMOOTHING_FACTOR}")
    print(f"Target change interval: {TARGET_CHANGE_INTERVAL} seconds")
    print("Press Ctrl+C to stop\n")
    viewer = mujoco.viewer.launch_passive(model, data)
    
    try:
        current_targets = np.array([np.random.uniform(*r) for r in ctrl_ranges])
        current_actions = data.ctrl.copy()
        start_time = time.time()
        last_target_change = start_time
        
        while True:
            current_time = time.time()
            if current_time - last_target_change >= TARGET_CHANGE_INTERVAL:
                new_targets = np.array([np.random.uniform(*r) for r in ctrl_ranges])
                current_targets = new_targets
                last_target_change = current_time
                print(f"\nTime: {current_time - start_time:.1f}s - New targets:")
                for name, target in zip(actuator_names, current_targets):
                    print(f"  {name}: {target:.3f}")
            current_actions = current_actions * (1 - SMOOTHING_FACTOR) + current_targets * SMOOTHING_FACTOR
            data.ctrl[:] = current_actions
            mujoco.mj_step(model, data)
            viewer.sync()
            time.sleep(model.opt.timestep * 1)
            
    except KeyboardInterrupt:
        print("\nSimulation stopped")
    finally:
        viewer.close()

def slow_sine_wave_motion():
    try:
        model = mujoco.MjModel.from_xml_path(XML_PATH)
        data = mujoco.MjData(model)
    except Exception as e:
        print(f"Error loading model: {e}")
        return

    actuator_names = []
    ctrl_ranges = []
    for i in range(model.nu):
        name = model.actuator(i).name
        ctrl_min = model.actuator(i).ctrlrange[0]
        ctrl_max = model.actuator(i).ctrlrange[1]
        actuator_names.append(name)
        ctrl_ranges.append((ctrl_min, ctrl_max))
        print(f"Actuator {i}: {name} range: [{ctrl_min:.2f}, {ctrl_max:.2f}]")
    print("\n" + "="*60)
    print("SLOW SINE WAVE MOTION")
    print("="*60)

    BASE_FREQUENCY = 0.1
    print(f"Base frequency: {BASE_FREQUENCY} Hz")
    print("Press Ctrl+C to stop\n")
    viewer = mujoco.viewer.launch_passive(model, data)
    
    try:
        start_time = time.time()
        while True:
            current_time = time.time() - start_time
            actions = []
            for i, (ctrl_min, ctrl_max) in enumerate(ctrl_ranges):
                freq = BASE_FREQUENCY / (i + 1)
                normalized = np.sin(2 * np.pi * freq * current_time)
                action = ctrl_min + (normalized + 1) / 2 * (ctrl_max - ctrl_min)
                actions.append(action)
            data.ctrl[:] = actions
            if int(current_time * 10) % 50 == 0:
                print(f"\rTime: {current_time:.1f}s", end="")
            mujoco.mj_step(model, data)
            viewer.sync()
            time.sleep(model.opt.timestep)
            
    except KeyboardInterrupt:
        print("\nSimulation stopped")
    finally:
        viewer.close()

def random_motion_with_velocity_limits():
    try:
        model = mujoco.MjModel.from_xml_path(XML_PATH)
        data = mujoco.MjData(model)
    except Exception as e:
        print(f"Error loading model: {e}")
        return
    actuator_names = []
    ctrl_ranges = []
    for i in range(model.nu):
        name = model.actuator(i).name
        ctrl_min = model.actuator(i).ctrlrange[0]
        ctrl_max = model.actuator(i).ctrlrange[1]
        actuator_names.append(name)
        ctrl_ranges.append((ctrl_min, ctrl_max))
        print(f"Actuator {i}: {name} range: [{ctrl_min:.2f}, {ctrl_max:.2f}]")
    
    print("\n" + "="*60)
    print("RANDOM MOTION WITH VELOCITY LIMITING")
    print("="*60)

    MAX_VELOCITY = 0.5
    TARGET_CHANGE_INTERVAL = 4.0
    DT = model.opt.timestep
    print(f"Max velocity: {MAX_VELOCITY} rad/s")
    print(f"Target change interval: {TARGET_CHANGE_INTERVAL} seconds")
    print("Press Ctrl+C to stop\n")
    viewer = mujoco.viewer.launch_passive(model, data)
    
    try:
        current_targets = data.ctrl.copy()
        current_actions = data.ctrl.copy()
        start_time = time.time()
        last_target_change = start_time
        
        while True:
            current_time = time.time()
            if current_time - last_target_change >= TARGET_CHANGE_INTERVAL:
                current_targets = np.array([np.random.uniform(*r) for r in ctrl_ranges])
                last_target_change = current_time
                print(f"\nTime: {current_time - start_time:.1f}s - New targets:")
                for name, target in zip(actuator_names, current_targets):
                    print(f"  {name}: {target:.3f}")
            for i in range(len(current_actions)):
                diff = current_targets[i] - current_actions[i]
                max_step = MAX_VELOCITY * DT
                
                if abs(diff) > max_step:
                    current_actions[i] += np.sign(diff) * max_step
                else:
                    current_actions[i] = current_targets[i]
            data.ctrl[:] = current_actions
            mujoco.mj_step(model, data)
            viewer.sync()
            time.sleep(DT)
            
    except KeyboardInterrupt:
        print("\nSimulation stopped")
    finally:
        viewer.close()

def very_slow_exploration():
    try:
        model = mujoco.MjModel.from_xml_path(XML_PATH)
        data = mujoco.MjData(model)
    except Exception as e:
        print(f"Error loading model: {e}")
        return
    actuator_names = []
    ctrl_ranges = []
    for i in range(model.nu):
        name = model.actuator(i).name
        ctrl_min = model.actuator(i).ctrlrange[0]
        ctrl_max = model.actuator(i).ctrlrange[1]
        actuator_names.append(name)
        ctrl_ranges.append((ctrl_min, ctrl_max))
        print(f"Actuator {i}: {name} range: [{ctrl_min:.2f}, {ctrl_max:.2f}]")
    
    print("\n" + "="*60)
    print("VERY SLOW EXPLORATION")
    print("="*60)
    SMOOTHING_FACTOR = 0.005
    TARGET_CHANGE_INTERVAL = 10.0
    print(f"Smoothing factor: {SMOOTHING_FACTOR} (very slow)")
    print(f"Target change interval: {TARGET_CHANGE_INTERVAL} seconds")
    print("Press Ctrl+C to stop\n")
    viewer = mujoco.viewer.launch_passive(model, data)
    
    try:
        current_targets = np.array([np.random.uniform(*r) for r in ctrl_ranges])
        current_actions = data.ctrl.copy()
        
        start_time = time.time()
        last_target_change = start_time
        
        while True:
            current_time = time.time()
            if current_time - last_target_change >= TARGET_CHANGE_INTERVAL:
                current_targets = np.array([np.random.uniform(*r) for r in ctrl_ranges])
                last_target_change = current_time
                print(f"\nTime: {current_time - start_time:.1f}s - New targets:")
                for name, target in zip(actuator_names, current_targets):
                    print(f"  {name}: {target:.3f}")
            current_actions = current_actions * (1 - SMOOTHING_FACTOR) + current_targets * SMOOTHING_FACTOR
            data.ctrl[:] = current_actions
            mujoco.mj_step(model, data)
            viewer.sync()
            time.sleep(model.opt.timestep)
            
    except KeyboardInterrupt:
        print("\nSimulation stopped")
    finally:
        viewer.close()

if __name__ == "__main__":
    print("ROBOT ARM CONTROL - SLOW & SMOOTH MOVEMENTS")
    print("="*60)
    
    print("\nChoose motion type:")
    print("1. Smooth random motion (recommended - good balance)")
    print("2. Slow sine wave motion (predictable, very smooth)")
    print("3. Random motion with velocity limits (explicit speed control)")
    print("4. Very slow exploration (extremely slow, best for observation)")
    print("5. Exit")
    print("="*60)
    
    try:
        choice = input("Enter choice (1-5): ").strip()
        if choice == '1':
            smooth_random_motion()
        elif choice == '2':
            slow_sine_wave_motion()
        elif choice == '3':
            random_motion_with_velocity_limits()
        elif choice == '4':
            very_slow_exploration()
        else:
            print("Exiting")
    except KeyboardInterrupt:
        print("\nExiting")