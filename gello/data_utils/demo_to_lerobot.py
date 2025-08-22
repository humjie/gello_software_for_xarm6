#!/usr/bin/env python3
"""
Convert demonstration data to LeRobot format

This script converts pickle files from gello demonstrations to the LeRobot dataset format,
which uses parquet files for data and MP4 videos for images.
"""

import glob
import os
import pickle
import json
import shutil
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Tuple, Any
import time

import numpy as np
import pandas as pd
import tyro
from natsort import natsorted
from tqdm import tqdm
import cv2
import imageio

from gello.data_utils.conversion_utils import preproc_obs_lerobot


@dataclass 
class Args:
    source_dir: str
    output_dir: str = None  # If None, will create output in source_dir/_lerobot
    robot_type: str = "xarm6_follower"
    fps: int = 30
    chunk_size: int = 1000
    max_episodes: int = None  # If None, process all episodes
    image_size: tuple = (480, 640)  # Height, Width
    joint_names: List[str] = None  # If None, use default xarm6 names


def get_default_joint_names(robot_type: str) -> List[str]:
    """Get default joint names based on robot type"""
    if "xarm6" in robot_type.lower():
        return [
            "shoulder_pan.pos",
            "shoulder_lift.pos", 
            "elbow_flex.pos",
            "wrist_flex.pos",
            "wrist_roll.pos",
            "gripper.pos"
        ]
    elif "xarm7" in robot_type.lower():
        return [
            "joint1.pos",
            "joint2.pos",
            "joint3.pos", 
            "joint4.pos",
            "joint5.pos",
            "joint6.pos",
            "joint7.pos"
        ]
    else:
        # Generic 6-DOF robot
        return [
            "joint1.pos",
            "joint2.pos", 
            "joint3.pos",
            "joint4.pos",
            "joint5.pos",
            "gripper.pos"
        ]


def process_episode_data(episode_dir: str, episode_index: int) -> Tuple[Dict[str, Any], bool]:
    """
    Process a single episode directory and return episode data
    
    Returns:
        (episode_data, success)
    """
    # Get all pickle files in the episode directory
    pkl_files = natsorted(glob.glob(os.path.join(episode_dir, "*.pkl")))
    
    if len(pkl_files) < 10:  # Minimum frames required
        print(f"Skipping episode {episode_index}: too few frames ({len(pkl_files)})")
        return {}, False
    
    # Remove last few frames (often incomplete)
    pkl_files = pkl_files[:-5] if len(pkl_files) > 10 else pkl_files
    
    episode_data = {
        "action": [],
        "observation.state": [],
        "observation.images.up": [],
        "observation.images.side": [],
        "timestamp": [],
        "frame_index": [],
        "episode_index": [],
        "task_index": []
    }
    
    for frame_idx, pkl_file in enumerate(pkl_files):
        try:
            with open(pkl_file, "rb") as f:
                demo = pickle.load(f)
        except Exception as e:
            print(f"Error loading {pkl_file}: {e}")
            continue
            
        # Process observations using the new LeRobot preprocessing
        obs = preproc_obs_lerobot(demo, joint_only=True, resize_images=False)
        
        # Extract action (control commands)
        action = demo.get("control", np.zeros(6))
        if len(action) > 6:
            action = action[:6]  # Limit to 6 DOF
        elif len(action) < 6:
            action = np.pad(action, (0, 6-len(action)), 'constant')
            
        # Extract state
        state = obs.get("state", np.zeros(6))
        if len(state) > 6:
            state = state[:6]
        elif len(state) < 6:
            state = np.pad(state, (0, 6-len(state)), 'constant')
            
        # Store frame data
        episode_data["action"].append(action.astype(np.float32))
        episode_data["observation.state"].append(state.astype(np.float32))
        
        # Store images (will be saved as videos later)
        if "images.up" in obs and obs["images.up"] is not None:
            episode_data["observation.images.up"].append(obs["images.up"])
        else:
            # Create dummy image if no camera data
            dummy_img = np.zeros((480, 640, 3), dtype=np.uint8)
            episode_data["observation.images.up"].append(dummy_img)
            
        if "images.side" in obs and obs["images.side"] is not None:
            episode_data["observation.images.side"].append(obs["images.side"])
        else:
            # Create dummy image if no camera data
            dummy_img = np.zeros((480, 640, 3), dtype=np.uint8)
            episode_data["observation.images.side"].append(dummy_img)
        
        # Metadata
        episode_data["timestamp"].append(np.array([frame_idx * (1.0/30.0)], dtype=np.float32))  # Assume 30fps
        episode_data["frame_index"].append(np.array([frame_idx], dtype=np.int64))
        episode_data["episode_index"].append(np.array([episode_index], dtype=np.int64))
        episode_data["task_index"].append(np.array([0], dtype=np.int64))  # Single task
    
    if len(episode_data["action"]) == 0:
        return {}, False
        
    # Convert lists to numpy arrays
    for key in episode_data:
        if key.startswith("observation.images"):
            continue  # Keep images as list for video creation
        episode_data[key] = np.stack(episode_data[key])
    
    return episode_data, True


def save_episode_videos(episode_data: Dict[str, Any], output_dir: Path, episode_index: int, 
                       episode_chunk: int, fps: int = 30):
    """Save episode images as MP4 videos"""
    
    video_dir = output_dir / "videos" / f"chunk-{episode_chunk:03d}"
    
    # Save "up" camera video
    up_video_dir = video_dir / "observation.images.up"
    up_video_dir.mkdir(parents=True, exist_ok=True)
    up_video_path = up_video_dir / f"episode_{episode_index:06d}.mp4"
    
    if len(episode_data["observation.images.up"]) > 0:
        images = episode_data["observation.images.up"]
        # Convert RGB to BGR for OpenCV/imageio
        images = [cv2.cvtColor(img, cv2.COLOR_RGB2BGR) if img.shape[2] == 3 else img for img in images]
        imageio.mimsave(str(up_video_path), images, fps=fps, codec='libx264')
    
    # Save "side" camera video  
    side_video_dir = video_dir / "observation.images.side"
    side_video_dir.mkdir(parents=True, exist_ok=True)
    side_video_path = side_video_dir / f"episode_{episode_index:06d}.mp4"
    
    if len(episode_data["observation.images.side"]) > 0:
        images = episode_data["observation.images.side"]
        # Convert RGB to BGR for OpenCV/imageio
        images = [cv2.cvtColor(img, cv2.COLOR_RGB2BGR) if img.shape[2] == 3 else img for img in images]
        imageio.mimsave(str(side_video_path), images, fps=fps, codec='libx264')


def save_episode_data(episode_data: Dict[str, Any], output_dir: Path, episode_index: int, 
                     episode_chunk: int):
    """Save episode data as parquet file"""
    
    data_dir = output_dir / "data" / f"chunk-{episode_chunk:03d}"
    data_dir.mkdir(parents=True, exist_ok=True)
    
    # Prepare data for parquet (exclude images)
    parquet_data = {}
    for key, value in episode_data.items():
        if not key.startswith("observation.images"):
            parquet_data[key] = value
    
    # Add global index
    num_frames = len(parquet_data["frame_index"])
    # This would need to be computed globally, for now use simple approach
    global_indices = np.arange(episode_index * 1000, episode_index * 1000 + num_frames, dtype=np.int64)
    parquet_data["index"] = global_indices.reshape(-1, 1)
    
    # Convert to DataFrame and save
    df_data = {}
    for key, value in parquet_data.items():
        # Flatten arrays to 1D for parquet
        if value.ndim > 1:
            # For multi-dimensional arrays, we need to store them properly
            df_data[key] = [row for row in value]
        else:
            df_data[key] = value.flatten()
    
    df = pd.DataFrame(df_data)
    parquet_path = data_dir / f"episode_{episode_index:06d}.parquet"
    df.to_parquet(parquet_path, index=False)


def create_metadata(output_dir: Path, args: Args, total_episodes: int, total_frames: int):
    """Create the dataset metadata JSON file"""
    
    joint_names = args.joint_names if args.joint_names else get_default_joint_names(args.robot_type)
    
    metadata = {
        "codebase_version": "v2.1",
        "robot_type": args.robot_type,
        "total_episodes": total_episodes,
        "total_frames": total_frames,
        "total_tasks": 1,
        "total_videos": total_episodes * 2,  # Two cameras per episode
        "total_chunks": (total_episodes + args.chunk_size - 1) // args.chunk_size,
        "chunks_size": args.chunk_size,
        "fps": args.fps,
        "splits": {
            "train": f"0:{total_episodes}"
        },
        "data_path": "data/chunk-{episode_chunk:03d}/episode_{episode_index:06d}.parquet",
        "video_path": "videos/chunk-{episode_chunk:03d}/{video_key}/episode_{episode_index:06d}.mp4",
        "features": {
            "action": {
                "dtype": "float32",
                "shape": [6],
                "names": joint_names
            },
            "observation.state": {
                "dtype": "float32", 
                "shape": [6],
                "names": joint_names
            },
            "observation.images.up": {
                "dtype": "video",
                "shape": [args.image_size[0], args.image_size[1], 3],
                "names": ["height", "width", "channels"],
                "info": {
                    "video.height": args.image_size[0],
                    "video.width": args.image_size[1],
                    "video.codec": "libx264",
                    "video.pix_fmt": "yuv420p",
                    "video.is_depth_map": False,
                    "video.fps": args.fps,
                    "video.channels": 3,
                    "has_audio": False
                }
            },
            "observation.images.side": {
                "dtype": "video",
                "shape": [args.image_size[0], args.image_size[1], 3],
                "names": ["height", "width", "channels"],
                "info": {
                    "video.height": args.image_size[0],
                    "video.width": args.image_size[1],
                    "video.codec": "libx264", 
                    "video.pix_fmt": "yuv420p",
                    "video.is_depth_map": False,
                    "video.fps": args.fps,
                    "video.channels": 3,
                    "has_audio": False
                }
            },
            "timestamp": {
                "dtype": "float32",
                "shape": [1],
                "names": None
            },
            "frame_index": {
                "dtype": "int64",
                "shape": [1],
                "names": None
            },
            "episode_index": {
                "dtype": "int64",
                "shape": [1], 
                "names": None
            },
            "index": {
                "dtype": "int64",
                "shape": [1],
                "names": None
            },
            "task_index": {
                "dtype": "int64",
                "shape": [1],
                "names": None
            }
        }
    }
    
    # Save metadata
    with open(output_dir / "meta_data.json", "w") as f:
        json.dump(metadata, f, indent=2)


def main(args: Args):
    # Setup paths
    source_dir = Path(args.source_dir).expanduser()
    if args.output_dir is None:
        output_dir = source_dir / "_lerobot"
    else:
        output_dir = Path(args.output_dir).expanduser()
    
    print(f"Converting data from {source_dir} to {output_dir}")
    
    # Create output directory
    if output_dir.exists():
        print(f"Output directory {output_dir} already exists, removing...")
        shutil.rmtree(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    
    # Find all episode directories
    episode_dirs = []
    for item in source_dir.iterdir():
        if item.is_dir() and not item.name.startswith('_'):
            episode_dirs.append(item)
    
    # If no subdirectories, treat the source_dir itself as single episode
    if not episode_dirs:
        episode_dirs = [source_dir]
    
    episode_dirs = natsorted(episode_dirs)
    
    if args.max_episodes is not None:
        episode_dirs = episode_dirs[:args.max_episodes]
    
    print(f"Found {len(episode_dirs)} episodes to process")
    
    # Process episodes
    total_frames = 0
    successful_episodes = 0
    
    for episode_idx, episode_dir in enumerate(tqdm(episode_dirs, desc="Processing episodes")):
        episode_data, success = process_episode_data(str(episode_dir), episode_idx)
        
        if not success:
            continue
            
        episode_chunk = episode_idx // args.chunk_size
        
        # Save episode data
        save_episode_data(episode_data, output_dir, episode_idx, episode_chunk)
        
        # Save videos
        save_episode_videos(episode_data, output_dir, episode_idx, episode_chunk, args.fps)
        
        # Update counters
        total_frames += len(episode_data["action"])
        successful_episodes += 1
        
        # Clean up memory
        del episode_data
    
    # Create metadata
    create_metadata(output_dir, args, successful_episodes, total_frames)
    
    print(f"\nConversion complete!")
    print(f"Episodes processed: {successful_episodes}/{len(episode_dirs)}")
    print(f"Total frames: {total_frames}")
    print(f"Output directory: {output_dir}")
    print(f"Dataset metadata saved to: {output_dir}/meta_data.json")


if __name__ == "__main__":
    main(tyro.cli(Args))
