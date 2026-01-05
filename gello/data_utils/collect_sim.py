import time
import os
import pickle
import sys
import select
import tty
import termios
import numpy as np
import mujoco
import mujoco.viewer
from dm_control import mujoco as dm_mujoco
from gello.agents.gello_agent import GelloAgent
from datetime import datetime

# --- CONFIGURATION ---
XML_PATH = os.path.expanduser("~/humjie/gello_software_for_xarm6/third_party/mujoco_menagerie/ufactory_xarm6/pick_and_place_scene.xml")
GELLO_PORT = "/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTAAMN19-if00-port0" 
SAVE_DIR = os.path.expanduser("~/bc_data/mujoco_xarm")
# ---------------------

def get_obs(physics, width=84, height=84):
    """Captures images and state for the dataset."""
    img_top = physics.render(height, width, camera_id="top_cam")
    img_wrist = physics.render(height, width, camera_id="wrist_cam")
    
    joint_pos = physics.data.qpos[:6].copy()
    gripper_state = physics.data.ctrl[6] / 0.04
    
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

    print(f"Loading Simulation from: {XML_PATH}")
    physics = dm_mujoco.Physics.from_xml_path(XML_PATH)
    
    print("Connecting to Gello Arm...")
    try:
        agent = GelloAgent(port=GELLO_PORT)
        print("SUCCESS: Gello connected.")
    except Exception as e:
        print(f"ERROR: Could not connect to Gello. Check USB.\n{e}")
        return

    timestamp = datetime.now().strftime("%m%d_%H%M%S")
    session_dir = os.path.join(SAVE_DIR, timestamp)
    os.makedirs(session_dir, exist_ok=True)
    print(f"Data will be saved to: {session_dir}")

    episode_count = 0
    target_freq = 60.0
    target_dt = 1.0 / target_freq
    sim_dt = physics.model.opt.timestep
    n_substeps = max(1, int(target_dt / sim_dt))

    # --- LAUNCH VIEWER ---
    with mujoco.viewer.launch_passive(physics.model.ptr, physics.data.ptr) as viewer:
        viewer.cam.distance = 1.5
        viewer.cam.lookat[:] = [0.4, 0, 0.2]
        viewer.cam.azimuth = 130 
        
        while viewer.is_running():
            print(f"\n" + "="*40)
            print(f"--- READY FOR EPISODE {episode_count} ---")
            print("1. Move Gello to START position.")
            print("2. Click this Terminal window to focus it.")
            cmd = input("3. Press [Enter] to start (or 'q' to quit): ")
            if cmd.lower() == 'q':
                break

            # --- 1. RESET ---
            physics.reset()
            
            # Sync to hardware
            start_pos = agent.act(None)
            physics.data.qpos[:6] = start_pos[:6]
            physics.data.ctrl[:6] = start_pos[:6]
            physics.data.ctrl[6] = np.clip(start_pos[6], 0.0, 1.0) * 0.04
            
            # Cube Setup (Height 0.14)
            physics.data.qpos[8+2] = 0.14 
            physics.data.qpos[8+0] = 0.65 + np.random.uniform(-0.05, 0.05)
            physics.data.qpos[8+1] = 0.0 + np.random.uniform(-0.1, 0.1)
            
            physics.forward()
            viewer.sync()

            episode_data = []
            print(">>> RECORDING... (Press 'd' in this Terminal to stop)")
            
            # --- ENTER RAW TERMINAL MODE ---
            # This allows us to detect 'd' instantly without waiting for Enter
            old_settings = termios.tcgetattr(sys.stdin)
            try:
                tty.setcbreak(sys.stdin.fileno())
                
                for _ in range(2000): # Max steps
                    loop_start = time.time()

                    # --- KEYBOARD CHECK (Non-blocking) ---
                    if select.select([sys.stdin], [], [], 0)[0]:
                        key = sys.stdin.read(1)
                        if key == 'd':
                            print("\nStopped via 'd'.")
                            break
                    # -------------------------------------

                    # A. Hardware
                    gello_act = agent.act(None) 
                    
                    # B. Physics
                    physics.data.ctrl[:6] = gello_act[:6]
                    physics.data.ctrl[6] = np.clip(gello_act[6], 0.0, 1.0) * 0.04
                    for _ in range(n_substeps):
                        physics.step()
                    
                    # C. Visuals & Data
                    viewer.sync()
                    obs = get_obs(physics)
                    episode_data.append({
                        "obs": obs,
                        "control": gello_act, 
                        "reward": 0, "done": False
                    })
                    
                    # D. Timing
                    elapsed = time.time() - loop_start
                    if target_dt > elapsed:
                        time.sleep(target_dt - elapsed)

            except KeyboardInterrupt:
                print("\nStopped via Ctrl+C")
            finally:
                # --- RESTORE NORMAL TERMINAL MODE ---
                # Crucial! Otherwise your terminal will be broken.
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

            # --- 3. VALIDATION ---
            if len(episode_data) > 30:
                print(f"Captured {len(episode_data)} frames.")
                valid = input("Was this episode SUCCESSFUL? (y/n): ").strip().lower()
                
                if valid == 'y':
                    save_path = os.path.join(session_dir, f"{episode_count}.pkl")
                    with open(save_path, "wb") as f:
                        pickle.dump(episode_data, f)
                    print(f"✅ Saved Episode {episode_count}")
                    episode_count += 1
                else:
                    print("❌ Discarded.")
            else:
                print("⚠️ Episode too short, discarded.")

if __name__ == "__main__":
    main()