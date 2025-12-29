import time
import os
import pickle
import numpy as np
import mujoco
import mujoco.viewer  # Required for the window
from dm_control import mujoco as dm_mujoco
from gello.agents.gello_agent import GelloAgent
from datetime import datetime

# --- CONFIGURATION ---
XML_PATH = os.path.expanduser("~/humjie/gello_software_for_xarm6/third_party/mujoco_menagerie/ufactory_xarm6/pick_and_place_scene.xml")
# Your specific port found earlier
GELLO_PORT = "/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTAAMN19-if00-port0" 
SAVE_DIR = os.path.expanduser("~/bc_data/mujoco_xarm")
# ---------------------

def get_obs(physics, width=84, height=84):
    """Captures images and state."""
    img_top = physics.render(height, width, camera_id="top_cam")
    img_wrist = physics.render(height, width, camera_id="wrist_cam")
    
    joint_pos = physics.data.qpos[:6].copy()
    gripper_state = physics.data.ctrl[6] / 255.0
    
    return {
        "rgb": np.stack([img_wrist, img_top]), 
        "joint_pos": joint_pos,
        "gripper": gripper_state,
        "timestamp": time.time()
    }

def main():
    if not os.path.exists(XML_PATH):
        print(f"ERROR: Cannot find {XML_PATH}")
        return

    print(f"Loading Simulation...")
    physics = dm_mujoco.Physics.from_xml_path(XML_PATH)
    
    print("Connecting to Gello Arm...")
    agent = GelloAgent(port=GELLO_PORT) 

    timestamp = datetime.now().strftime("%m%d_%H%M%S")
    session_dir = os.path.join(SAVE_DIR, timestamp)
    os.makedirs(session_dir, exist_ok=True)
    print(f"Saving data to: {session_dir}")

    episode_count = 0

    # --- LAUNCH THE VIEWER ---
    # We pass the underlying MuJoCo pointers (.ptr) from dm_control to the viewer
    with mujoco.viewer.launch_passive(physics.model.ptr, physics.data.ptr) as viewer:
        
        # Initial View Setup (Optional: Zoom out slightly)
        viewer.cam.distance = 2.0
        viewer.cam.lookat[:] = [0.4, 0, 0.4] 
        
        while viewer.is_running():
            print(f"\n--- Episode {episode_count} ---")
            print("1. Move Gello to START position.")
            cmd = input("Press [Enter] to start recording, or 'q' to quit: ")
            if cmd.lower() == 'q':
                break

            # --- RESET ---
            physics.reset()
            
            # 1. MEASURE REAL HAND
            start_pos = agent.act(None)
            
            # 2. TELEPORT SIM TO REAL HAND (Prevent Jump)
            physics.data.qpos[:6] = start_pos[:6] # Sync Arm
            
            # 3. SYNC GRIPPER (Fixes the gripper snap)
            # We calculate the safe gripper value immediately
            g_raw = start_pos[6]
            # Use the same logic you use in the loop (Invert if needed!)
            g_val = np.clip(g_raw, 0.0, 1.0) 
            
            physics.data.ctrl[:6] = start_pos[:6]
            physics.data.ctrl[6] = g_val * 255.0
            
            # 4. RESET CUBE
            physics.data.qpos[6+2] = 0.46
            physics.data.qpos[6+0] = 0.4 + np.random.uniform(-0.05, 0.05)
            physics.data.qpos[6+1] = 0.0 + np.random.uniform(-0.1, 0.1) 
            
            physics.forward()
            viewer.sync()
            
            # Now wait for user input or just continue

            episode_data = []
            print(">>> RECORDING... (Ctrl+C to stop episode)")
            
            # Randomize Cube
            physics.data.qpos[6+2] = 0.46 # Z (Safe height)
            physics.data.qpos[6+0] = 0.4 + np.random.uniform(-0.05, 0.05) # X
            physics.data.qpos[6+1] = 0.0 + np.random.uniform(-0.1, 0.1)   # Y
            
            # Sync viewer once after reset so you see the new cube pos
            viewer.sync()

            episode_data = []
            print(">>> RECORDING... (Ctrl+C to stop episode)")
            
            try:
                for _ in range(1000): # Max 1000 steps (approx 50s at 20Hz)
                    loop_start = time.time()
                    
                    # 1. READ HARDWARE
                    gello_act = agent.act(None) 
                    
                    # 2. APPLY TO SIM
                    physics.data.ctrl[:6] = gello_act[:6]
                    raw_gripper = gello_act[6] # Value from hardware (usually -1 to 1 or 0 to 1)
                    safe_gripper = np.clip(raw_gripper, 0.0, 1.0) 

                    # 2. Map to MuJoCo range (0 to 255)
                    physics.data.ctrl[6] = safe_gripper * 255.0

                    # 3. STEP PHYSICS
                    physics.step()
                    
                    # 4. UPDATE VIEWER (The missing piece!)
                    viewer.sync()
                    
                    # 5. RECORD
                    obs = get_obs(physics)
                    step_data = {
                        "obs": obs,
                        "control": gello_act, 
                        "reward": 0,
                        "done": False
                    }
                    episode_data.append(step_data)
                    
                    # 6. TIMING (Change 20Hz to 60Hz)
                    elapsed = time.time() - loop_start
                    target_freq = 60.0
                    sleep_time = (1.0 / target_freq) - elapsed
                    
                    if sleep_time > 0:
                        time.sleep(sleep_time)

            except KeyboardInterrupt:
                print(" Stopped by user.")
            
            # --- SAVE ---
            if len(episode_data) > 20:
                save_path = os.path.join(session_dir, f"{episode_count}.pkl")
                with open(save_path, "wb") as f:
                    pickle.dump(episode_data, f)
                print(f"Saved Ep {episode_count}: {len(episode_data)} frames")
                episode_count += 1
            else:
                print("Episode too short, discarded.")

if __name__ == "__main__":
    main()