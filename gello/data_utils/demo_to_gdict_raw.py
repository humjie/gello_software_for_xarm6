import glob
import os
import pickle
import shutil
from dataclasses import dataclass

import numpy as np
import tyro
from natsort import natsorted
from tqdm import tqdm

import mediapy as mp
from gdict.data import DictArray, GDict
from gello.data_utils.conversion_utils import preproc_obs

# --- CORRECTED SCRIPT ---

def convert_single_demo(
    source_dir,
    folder_idx,
    traj_output_dir,
    rgb_output_dir,
    depth_output_dir,
    state_output_dir,
    action_output_dir,
):
    """
    Converts pickles in a folder into individual GDict HDF5 files.
    """
    # Find all pickles in the timestamp folder
    pkls = natsorted(
        glob.glob(os.path.join(source_dir, "*.pkl"), recursive=True)
    )
    
    if len(pkls) == 0:
        print(f"Skipping {source_dir}, no pickles found.")
        return 0

    success_count = 0

    for pkl_idx, pkl in enumerate(pkls):
        try:
            with open(pkl, "rb") as f:
                demo = pickle.load(f)
        except Exception as e:
            print(f"Skipping {pkl}, corrupted: {e}")
            continue

        # Basic filtering (skip super short episodes)
        if len(demo) < 10: 
            continue

        # 1. Process Data
        # preproc_obs usually expects a list of dicts. 
        # It stacks them into (Time, Camera, C, H, W)
        obs = preproc_obs(demo) 
        action = np.array([step["control"] for step in demo])

        # 2. Structure for GDict
        # We create a dictionary for THIS specific episode
        curr_ts = {}
        curr_ts["obs"] = obs
        curr_ts["actions"] = action
        
        # 3. Save as HDF5
        # We skip the "stacking" logic and save directly
        # Format: traj_{FolderID}_{EpisodeID}.h5
        unique_name = f"traj_{folder_idx}_{pkl_idx}"
        
        # Wrap in a dict key matching the filename for consistency
        wrapper = {unique_name: curr_ts}
        
        # Convert to DictArray (Handles numpy conversion)
        demo_dict = DictArray.from_dict(wrapper)
        
        save_path = os.path.join(traj_output_dir, f"{unique_name}.h5")
        GDict.to_hdf5(demo_dict, save_path)
        
        success_count += 1
        
        # --- OPTIONAL: SAVE VIDEO FOR FIRST EPISODE ONLY (To save time) ---
        if pkl_idx == 0:
            try:
                # Wrist (Cam 0) and Agent (Cam 1)
                # Shape: (T, C, H, W) -> (T, H, W, C)
                video_data = demo_dict[unique_name]["obs"]["rgb"]
                
                # Wrist
                wrist = video_data[:, 0].transpose([0, 2, 3, 1]).astype(np.uint8)
                mp.write_video(os.path.join(rgb_output_dir, f"{unique_name}_wrist.mp4"), wrist, fps=30)
                
                # Agent
                if video_data.shape[1] > 1:
                    agent = video_data[:, 1].transpose([0, 2, 3, 1]).astype(np.uint8)
                    mp.write_video(os.path.join(rgb_output_dir, f"{unique_name}_agent.mp4"), agent, fps=30)
            except Exception as e:
                print(f"Video saving failed for {unique_name}: {e}")

    return success_count

@dataclass
class Args:
    source_dir: str # Path to your timestamp folder (e.g. ~/bc_data/mujoco_xarm/0105_120000)

def main(args):
    args.source_dir = os.path.expanduser(args.source_dir)
    
    # Setup Output Dirs
    output_dir = os.path.join(args.source_dir, "_conv")
    raw_dir = os.path.join(output_dir, "multiview", "train", "none") # Structure for merger
    vis_dir = os.path.join(output_dir, "vis")
    
    # Clear old conversion
    if os.path.exists(output_dir):
        shutil.rmtree(output_dir)
        
    for d in [raw_dir, os.path.join(vis_dir, "rgb")]:
        os.makedirs(d, exist_ok=True)

    print(f"Processing {args.source_dir}...")
    
    count = convert_single_demo(
        args.source_dir,
        0, # folder index (just 0 since processing one folder)
        raw_dir,
        os.path.join(vis_dir, "rgb"),
        None, None, None # Skip depth/action plots for speed
    )

    print(f"Converted {count} trajectories.")
    print(f"Ready for merger script.")

if __name__ == "__main__":
    main(tyro.cli(Args))