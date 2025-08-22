#!/usr/bin/env python3
"""
Validate LeRobot dataset format

This script validates that a converted dataset matches the expected LeRobot format
and can be loaded correctly.
"""

import json
import os
from pathlib import Path
from typing import Dict, Any
import argparse

import pandas as pd
import numpy as np
import cv2


def validate_metadata(dataset_dir: Path) -> Dict[str, Any]:
    """Validate the metadata file and return metadata dict"""
    
    metadata_path = dataset_dir / "meta_data.json"
    if not metadata_path.exists():
        raise FileNotFoundError(f"Metadata file not found: {metadata_path}")
    
    with open(metadata_path, 'r') as f:
        metadata = json.load(f)
    
    # Check required fields
    required_fields = [
        "codebase_version", "robot_type", "total_episodes", "total_frames",
        "total_tasks", "total_videos", "total_chunks", "chunks_size", "fps",
        "splits", "data_path", "video_path", "features"
    ]
    
    for field in required_fields:
        if field not in metadata:
            raise ValueError(f"Missing required metadata field: {field}")
    
    print(f"✓ Metadata validation passed")
    print(f"  - Robot type: {metadata['robot_type']}")
    print(f"  - Total episodes: {metadata['total_episodes']}")
    print(f"  - Total frames: {metadata['total_frames']}")
    print(f"  - Total chunks: {metadata['total_chunks']}")
    
    return metadata


def validate_data_files(dataset_dir: Path, metadata: Dict[str, Any]) -> bool:
    """Validate that all expected data files exist and have correct format"""
    
    total_episodes = metadata["total_episodes"]
    chunk_size = metadata["chunks_size"]
    
    data_dir = dataset_dir / "data"
    if not data_dir.exists():
        raise FileNotFoundError(f"Data directory not found: {data_dir}")
    
    # Check each episode
    missing_files = []
    total_frames_found = 0
    
    for episode_idx in range(total_episodes):
        episode_chunk = episode_idx // chunk_size
        chunk_dir = data_dir / f"chunk-{episode_chunk:03d}"
        episode_file = chunk_dir / f"episode_{episode_idx:06d}.parquet"
        
        if not episode_file.exists():
            missing_files.append(str(episode_file))
        else:
            # Try to load and validate the parquet file
            try:
                df = pd.read_parquet(episode_file)
                total_frames_found += len(df)
                
                # Check required columns
                required_columns = [
                    "action", "observation.state", "timestamp", 
                    "frame_index", "episode_index", "index", "task_index"
                ]
                
                for col in required_columns:
                    if col not in df.columns:
                        raise ValueError(f"Missing column {col} in {episode_file}")
                
            except Exception as e:
                print(f"Error loading {episode_file}: {e}")
                missing_files.append(str(episode_file))
    
    if missing_files:
        print(f"✗ Missing or corrupted data files: {len(missing_files)}")
        for f in missing_files[:5]:  # Show first 5
            print(f"  - {f}")
        if len(missing_files) > 5:
            print(f"  - ... and {len(missing_files) - 5} more")
        return False
    
    print(f"✓ Data files validation passed")
    print(f"  - Total frames found: {total_frames_found}")
    print(f"  - Expected frames: {metadata['total_frames']}")
    
    if total_frames_found != metadata['total_frames']:
        print(f"⚠ Frame count mismatch!")
        
    return True


def validate_video_files(dataset_dir: Path, metadata: Dict[str, Any]) -> bool:
    """Validate that all expected video files exist"""
    
    total_episodes = metadata["total_episodes"]
    chunk_size = metadata["chunks_size"]
    
    videos_dir = dataset_dir / "videos"
    if not videos_dir.exists():
        raise FileNotFoundError(f"Videos directory not found: {videos_dir}")
    
    # Check for both camera views
    camera_views = ["observation.images.up", "observation.images.side"]
    missing_videos = []
    
    for episode_idx in range(total_episodes):
        episode_chunk = episode_idx // chunk_size
        
        for view in camera_views:
            video_dir = videos_dir / f"chunk-{episode_chunk:03d}" / view
            video_file = video_dir / f"episode_{episode_idx:06d}.mp4"
            
            if not video_file.exists():
                missing_videos.append(str(video_file))
            else:
                # Try to open video file to check if it's valid
                try:
                    cap = cv2.VideoCapture(str(video_file))
                    if not cap.isOpened():
                        missing_videos.append(f"{video_file} (corrupted)")
                    else:
                        # Check basic properties
                        fps = cap.get(cv2.CAP_PROP_FPS)
                        frame_count = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
                        width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
                        height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
                        
                        expected_fps = metadata["fps"]
                        if abs(fps - expected_fps) > 1:
                            print(f"⚠ FPS mismatch in {video_file}: {fps} vs {expected_fps}")
                    
                    cap.release()
                    
                except Exception as e:
                    missing_videos.append(f"{video_file} (error: {e})")
    
    if missing_videos:
        print(f"✗ Missing or corrupted video files: {len(missing_videos)}")
        for f in missing_videos[:5]:  # Show first 5
            print(f"  - {f}")
        if len(missing_videos) > 5:
            print(f"  - ... and {len(missing_videos) - 5} more")
        return False
    
    print(f"✓ Video files validation passed")
    return True


def validate_data_shapes(dataset_dir: Path, metadata: Dict[str, Any]) -> bool:
    """Validate data shapes match metadata specifications"""
    
    features = metadata["features"]
    
    # Load a sample episode to check shapes
    data_dir = dataset_dir / "data" / "chunk-000"
    sample_files = list(data_dir.glob("episode_*.parquet"))
    
    if not sample_files:
        print("✗ No sample data files found for shape validation")
        return False
    
    sample_file = sample_files[0]
    df = pd.read_parquet(sample_file)
    
    print(f"✓ Validating data shapes using {sample_file.name}")
    
    # Check each feature
    for feature_name, feature_spec in features.items():
        if feature_name.startswith("observation.images"):
            continue  # Skip video features for now
            
        if feature_name not in df.columns:
            print(f"✗ Feature {feature_name} not found in data")
            return False
        
        # Get sample data
        sample_data = df[feature_name].iloc[0]
        
        if isinstance(sample_data, (list, np.ndarray)):
            actual_shape = np.array(sample_data).shape
        else:
            actual_shape = (1,)  # Scalar
        
        expected_shape = tuple(feature_spec["shape"])
        
        if actual_shape != expected_shape:
            print(f"✗ Shape mismatch for {feature_name}: {actual_shape} vs {expected_shape}")
            return False
        else:
            print(f"  - {feature_name}: {actual_shape} ✓")
    
    return True


def main():
    parser = argparse.ArgumentParser(description="Validate LeRobot dataset format")
    parser.add_argument("dataset_dir", help="Path to the dataset directory")
    parser.add_argument("--sample-episode", type=int, default=0, 
                       help="Episode index to sample for detailed validation")
    
    args = parser.parse_args()
    
    dataset_dir = Path(args.dataset_dir)
    
    print(f"Validating LeRobot dataset: {dataset_dir}")
    print("=" * 60)
    
    try:
        # Validate metadata
        metadata = validate_metadata(dataset_dir)
        
        # Validate data files
        data_valid = validate_data_files(dataset_dir, metadata)
        
        # Validate video files
        videos_valid = validate_video_files(dataset_dir, metadata)
        
        # Validate data shapes
        shapes_valid = validate_data_shapes(dataset_dir, metadata)
        
        # Summary
        print("\n" + "=" * 60)
        if data_valid and videos_valid and shapes_valid:
            print("🎉 Dataset validation PASSED!")
            print("The dataset appears to be correctly formatted for LeRobot.")
        else:
            print("❌ Dataset validation FAILED!")
            print("Please check the errors above and regenerate the dataset.")
            
        # Additional info
        print(f"\nDataset Summary:")
        print(f"  - Episodes: {metadata['total_episodes']}")
        print(f"  - Frames: {metadata['total_frames']}")
        print(f"  - Robot: {metadata['robot_type']}")
        print(f"  - FPS: {metadata['fps']}")
        
        # Show sample episode info
        if args.sample_episode < metadata['total_episodes']:
            chunk_idx = args.sample_episode // metadata['chunks_size']
            data_file = dataset_dir / "data" / f"chunk-{chunk_idx:03d}" / f"episode_{args.sample_episode:06d}.parquet"
            
            if data_file.exists():
                df = pd.read_parquet(data_file)
                print(f"\nSample Episode {args.sample_episode}:")
                print(f"  - Frames: {len(df)}")
                print(f"  - Duration: {len(df) / metadata['fps']:.1f}s")
                
                # Show data ranges
                if "action" in df.columns:
                    actions = np.array(df["action"].tolist())
                    print(f"  - Action range: [{actions.min():.3f}, {actions.max():.3f}]")
                
                if "observation.state" in df.columns:
                    states = np.array(df["observation.state"].tolist())
                    print(f"  - State range: [{states.min():.3f}, {states.max():.3f}]")
        
    except Exception as e:
        print(f"❌ Validation failed with error: {e}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    main()
