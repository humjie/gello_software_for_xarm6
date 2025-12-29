import h5py
import os
import glob
import numpy as np
import argparse
from tqdm import tqdm

def merge_datasets(input_root, output_file):
    print(f"Scanning for files in {input_root}...")
    
    f_out = h5py.File(output_file, "w")
    data_grp = f_out.create_group("data")
    mask_grp = f_out.create_group("mask")
    
    train_demos = []
    valid_demos = []
    
    global_idx = 0
    
    # Gello output structure: root -> train -> none -> *.h5
    phases = [("train", train_demos), ("val", valid_demos)]
    
    for phase_name, list_tracker in phases:
        # Search path matches the structure from demo_to_gdict_raw.py
        search_path = os.path.join(input_root, phase_name, "none", "*.h5")
        files = sorted(glob.glob(search_path))
        
        if len(files) == 0:
            print(f"Warning: No files found for {phase_name} phase at {search_path}")
            continue

        print(f"Processing {phase_name} ({len(files)} files)...")
        
        for file_path in tqdm(files):
            try:
                with h5py.File(file_path, "r") as f_in:
                    # Gello creates a group named "traj_N", we need to find it dynamically
                    traj_key = list(f_in.keys())[0]
                    src_data = f_in[traj_key]
                    
                    demo_name = f"demo_{global_idx}"
                    dest_demo_grp = data_grp.create_group(demo_name)
                    
                    # --- OBS ---
                    obs_grp = dest_demo_grp.create_group("obs")
                    
                    # IMAGES: Convert (T, C, H, W) -> (T, H, W, C) for compatibility
                    # Index 0 = Wrist, Index 1 = Agent/Base
                    wrist_rgb = src_data["obs"]["rgb"][:, 0] 
                    wrist_rgb = np.transpose(wrist_rgb, (0, 2, 3, 1)) # to HWC
                    obs_grp.create_dataset("wrist_image", data=wrist_rgb)
                    
                    agent_rgb = src_data["obs"]["rgb"][:, 1]
                    agent_rgb = np.transpose(agent_rgb, (0, 2, 3, 1)) # to HWC
                    obs_grp.create_dataset("agentview_image", data=agent_rgb)
                    
                    # STATE
                    state_data = src_data["obs"]["state"][...]
                    obs_grp.create_dataset("robot0_eef_pos", data=state_data)
                    
                    # --- ACTIONS ---
                    actions = src_data["actions"][...]
                    dest_demo_grp.create_dataset("actions", data=actions)
                    
                    # Meta
                    dest_demo_grp.attrs["num_samples"] = actions.shape[0]
                    
                    list_tracker.append(demo_name)
                    global_idx += 1
                    
            except Exception as e:
                print(f"Error processing {file_path}: {e}")

    # Write Masks (ASCII encoding required for Robomimic compatibility)
    mask_grp.create_dataset("train", data=np.array(train_demos, dtype='S20'))
    mask_grp.create_dataset("valid", data=np.array(valid_demos, dtype='S20'))
    
    f_out.close()
    print(f"\nSuccess! Saved to {output_file}")
    print(f"Total trajectories: {global_idx}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", type=str, required=True, help="Path to the _conv/multiview folder")
    parser.add_argument("--output", type=str, default="training_data.hdf5", help="Name of output file")
    args = parser.parse_args()
    
    merge_datasets(args.input, args.output)