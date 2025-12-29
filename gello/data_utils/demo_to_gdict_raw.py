import glob
import os
import pickle
import shutil
from dataclasses import dataclass
from typing import Tuple

import numpy as np
import tyro
from natsort import natsorted
from tqdm import tqdm

from gello.data_utils.plot_utils import plot_in_grid
import mediapy as mp
from gdict.data import DictArray, GDict
from simple_bc.utils.visualization_utils import make_grid_video_from_numpy
from gello.data_utils.conversion_utils import preproc_obs

np.set_printoptions(precision=3, suppress=True)

def convert_single_demo(
    source_dir,
    i,
    traj_output_dir,
    rgb_output_dir,
    depth_output_dir,
    state_output_dir,
    action_output_dir,
):
    """
    Converts a single demo directory into a GDict HDF5 file.
    """
    pkls = natsorted(
        glob.glob(os.path.join(source_dir, "*.pkl"), recursive=True), reverse=True
    )
    demo_stack = []

    # CHANGED: Lowered limit from 30 to 5 for debugging
    if len(pkls) < 5:
        print(f"Skipping {source_dir}, too few frames ({len(pkls)})")
        return 0

    # CHANGED: Only crop the last 5 frames if we have enough data
    if len(pkls) > 15:
        pkls = pkls[:-5]

    for pkl in pkls:
        curr_ts = {}
        try:
            with open(pkl, "rb") as f:
                demo = pickle.load(f)
        except Exception as e:
            print(f"Skipping {pkl} because it is corrupted: {e}")
            return 0

        obs = preproc_obs(demo)
        action = demo.pop("control")
        
        # CHANGED: NO NORMALIZATION
        # action = (action - bias_factor) / scale_factor 

        curr_ts["obs"] = obs
        curr_ts["actions"] = action
        curr_ts["dones"] = np.zeros(1) 
        curr_ts["episode_dones"] = np.zeros(1)

        curr_ts_wrapped = dict()
        curr_ts_wrapped[f"traj_{i}"] = curr_ts
        demo_stack = [curr_ts_wrapped] + demo_stack

    demo_dict = DictArray.stack(demo_stack)
    
    # Save individual trajectory
    save_path = os.path.join(traj_output_dir, f"traj_{i}.h5")
    GDict.to_hdf5(demo_dict, save_path)

    # --- VISUALIZATION (Optional) ---
    # Extract images for video saving
    # Assuming standard Gello format: (T, Cam, C, H, W) -> Transpose to (T, H, W, C)
    
    # Agent View (Index 1)
    all_rgbs = demo_dict[f"traj_{i}"]["obs"]["rgb"][:, 1].transpose([0, 2, 3, 1])
    all_rgbs = all_rgbs.astype(np.uint8)
    _, H, W, _ = all_rgbs.shape
    
    all_depths = demo_dict[f"traj_{i}"]["obs"]["depth"][:, 1].reshape([-1, H, W])
    all_depths = all_depths / 5.0  # visual scaling

    mp.write_video(os.path.join(rgb_output_dir, f"traj_{i}_rgb_base.mp4"), all_rgbs, fps=30)
    mp.write_video(os.path.join(depth_output_dir, f"traj_{i}_depth_base.mp4"), all_depths, fps=30)

    # Wrist View (Index 0)
    all_rgbs_wrist = demo_dict[f"traj_{i}"]["obs"]["rgb"][:, 0].transpose([0, 2, 3, 1])
    all_rgbs_wrist = all_rgbs_wrist.astype(np.uint8)
    
    all_depths_wrist = demo_dict[f"traj_{i}"]["obs"]["depth"][:, 0].reshape([-1, H, W])
    all_depths_wrist = all_depths_wrist / 2.0

    mp.write_video(os.path.join(rgb_output_dir, f"traj_{i}_rgb_wrist.mp4"), all_rgbs_wrist, fps=30)
    mp.write_video(os.path.join(depth_output_dir, f"traj_{i}_depth_wrist.mp4"), all_depths_wrist, fps=30)

    all_actions = demo_dict[f"traj_{i}"]["actions"]
    all_states = demo_dict[f"traj_{i}"]["obs"]["state"]

    curr_actions = all_actions.reshape([1, *all_actions.shape])
    curr_states = all_states.reshape([-1, *all_states.shape])

    plot_in_grid(curr_actions, os.path.join(action_output_dir, f"traj_{i}_actions.png"))
    plot_in_grid(curr_states, os.path.join(state_output_dir, f"traj_{i}_states.png"))

    return all_rgbs, all_depths, all_actions, all_states

@dataclass
class Args:
    source_dir: str
    vis: bool = True

def main(args):
    args.source_dir = os.path.expanduser(args.source_dir)
    # Support both single folder or parent folder of demos
    subdirs = [args.source_dir]

    output_dir = os.path.join(args.source_dir, "_conv")
    if not os.path.isdir(output_dir):
        os.makedirs(output_dir, exist_ok=True)

    output_dir = os.path.join(output_dir, "multiview")
    if os.path.isdir(output_dir):
        print(f"Deleting existing output: {output_dir}")
        shutil.rmtree(output_dir)
    os.makedirs(output_dir, exist_ok=True)

    train_dir = os.path.join(output_dir, "train")
    val_dir = os.path.join(output_dir, "val")
    os.makedirs(train_dir, exist_ok=True)
    os.makedirs(val_dir, exist_ok=True)

    # CHANGED: REMOVED STAT CALCULATION LOOP entirely
    print("Skipping normalization calculation for Diffusion Policy compatibility.")

    # Visualization directories
    vis_dir = os.path.join(output_dir, "vis")
    os.makedirs(os.path.join(vis_dir, "state"), exist_ok=True)
    os.makedirs(os.path.join(vis_dir, "action"), exist_ok=True)
    os.makedirs(os.path.join(vis_dir, "rgb"), exist_ok=True)
    os.makedirs(os.path.join(vis_dir, "depth"), exist_ok=True)

    # Split Train/Val
    val_size = int(min(0.1 * len(subdirs), 10))
    val_indices = set(np.random.choice(len(subdirs), size=val_size, replace=False))

    tot = 0
    pbar = tqdm(range(len(subdirs)))
    for i in pbar:
        out_dir = val_dir if i in val_indices else train_dir
        out_dir = os.path.join(out_dir, "none") # Create 'none' subfolder to match structure
        os.makedirs(out_dir, exist_ok=True)

        ret = convert_single_demo(
            subdirs[i],
            i,
            out_dir,
            os.path.join(vis_dir, "rgb"),
            os.path.join(vis_dir, "depth"),
            os.path.join(vis_dir, "state"),
            os.path.join(vis_dir, "action"),
        )

        if ret != 0:
            tot += 1

    print(f"Finished! Converted {tot} trajectories.")
    print(f"Output saved to: {output_dir}")

if __name__ == "__main__":
    main(tyro.cli(Args))