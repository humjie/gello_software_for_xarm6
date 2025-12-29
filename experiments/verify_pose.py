import time
import os
import numpy as np
import mujoco
import mujoco.viewer
from dm_control import mujoco as dm_mujoco
from gello.agents.gello_agent import GelloAgent

# --- CONFIGURATION ---
# Path to your SCENE or ROBOT XML
XML_PATH = os.path.expanduser("~/humjie/gello_software_for_xarm6/third_party/mujoco_menagerie/ufactory_xarm6/pick_and_place_scene.xml")

# Your specific USB port
GELLO_PORT = "/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTAAMN19-if00-port0"
# ---------------------

def main():
    # 1. Check XML
    if not os.path.exists(XML_PATH):
        print(f"ERROR: Cannot find XML at {XML_PATH}")
        return

    # 2. Connect to Hardware
    print("Connecting to Real Gello Arm...")
    try:
        agent = GelloAgent(port=GELLO_PORT)
        print("SUCCESS: Gello connected.")
    except Exception as e:
        print(f"ERROR: Could not connect to Gello. Check USB.\n{e}")
        return

    # 3. Load Simulation
    print("Loading MuJoCo Simulation...")
    physics = dm_mujoco.Physics.from_xml_path(XML_PATH)

    print("\n" + "="*60)
    print("  POSE VERIFICATION MODE")
    print("  Move your Gello arm. The simulation should match exactly.")
    print("  Press Ctrl+C to exit.")
    print("="*60 + "\n")

    # 4. Launch Viewer
    with mujoco.viewer.launch_passive(physics.model.ptr, physics.data.ptr) as viewer:
        # Set camera to a good angle
        viewer.cam.distance = 2.0
        viewer.cam.lookat[:] = [0, 0, 0.5]
        viewer.cam.azimuth = 90

        try:
            while viewer.is_running():
                # --- READ REAL SENSORS ---
                # gello_act contains [Joint1, ..., Joint6, Gripper]
                real_joints = agent.act(None)
                
                # --- UPDATE SIMULATION ---
                # We map the real joint angles DIRECTLY to the sim joint positions (qpos)
                # This shows exactly where the sim 'thinks' 0 degrees is vs the real world.
                
                # 1. Arm Joints (Indices 0-5)
                physics.data.qpos[:6] = real_joints[:6]
                
                # 2. Gripper (Index 6)
                # Visualize gripper opening (Real 0-1 -> Sim slide distance)
                # Your XML gripper range is 0 to 0.04m (slide).
                # Adjust multiplier to make it look right visually.
                gripper_val = real_joints[6]
                
                # Note: In your XML, indices 6 and 7 are usually the finger joints
                # We update them directly to match the Gello input for visual check
                # Assuming 0=Open, 1=Closed. If reversed, use (1 - gripper_val)
                physics.data.qpos[6] = gripper_val * 0.04 # Right finger
                physics.data.qpos[7] = gripper_val * 0.04 # Left finger
                
                # --- FORWARD KINEMATICS ---
                # Calculates where the body parts should be based on qpos
                physics.forward() 
                
                # --- SYNC VIEWER ---
                viewer.sync()

                # --- PRINT DEBUG INFO (Every 10 frames to reduce spam) ---
                # This helps you check numerical values
                print(f"\rReal Joints: {np.round(real_joints[:6], 2)} | Gripper: {real_joints[6]:.2f}", end="")

                # Run at 60Hz for smooth visuals
                time.sleep(1/60)

        except KeyboardInterrupt:
            print("\nVerification stopped.")

if __name__ == "__main__":
    main()