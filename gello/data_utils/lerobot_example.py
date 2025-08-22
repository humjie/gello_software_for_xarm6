#!/usr/bin/env python3
"""
Example usage of LeRobot dataset conversion

This script demonstrates how to use the conversion tools to convert
gello demonstration data to LeRobot format.
"""

import os
import subprocess
import sys
from pathlib import Path


def run_conversion_example():
    """Run a complete example of converting data to LeRobot format"""
    
    print("LeRobot Dataset Conversion Example")
    print("=" * 50)
    
    # Example paths (adjust these to your actual data)
    source_dir = "~/bc_data/gello"  # Your demonstration data directory
    output_dir = "~/bc_data/gello_lerobot"  # Output directory for LeRobot format
    
    print(f"Source directory: {source_dir}")
    print(f"Output directory: {output_dir}")
    
    # Expand paths
    source_path = Path(source_dir).expanduser()
    output_path = Path(output_dir).expanduser()
    
    # Check if source directory exists
    if not source_path.exists():
        print(f"❌ Source directory does not exist: {source_path}")
        print("Please update the source_dir variable to point to your demonstration data.")
        return False
    
    print(f"✓ Found source directory: {source_path}")
    
    # Run conversion
    conversion_script = Path(__file__).parent / "demo_to_lerobot.py"
    
    cmd = [
        sys.executable, str(conversion_script),
        str(source_path),
        "--output_dir", str(output_path),
        "--robot_type", "xarm6_follower",
        "--fps", "30",
        "--max_episodes", "10"  # Limit to 10 episodes for example
    ]
    
    print(f"\nRunning conversion command:")
    print(" ".join(cmd))
    
    try:
        result = subprocess.run(cmd, check=True, capture_output=True, text=True)
        print("✓ Conversion completed successfully!")
        print(result.stdout)
        
    except subprocess.CalledProcessError as e:
        print(f"❌ Conversion failed: {e}")
        print("STDOUT:", e.stdout)
        print("STDERR:", e.stderr)
        return False
    
    # Validate the output
    validation_script = Path(__file__).parent / "validate_lerobot.py"
    
    cmd = [sys.executable, str(validation_script), str(output_path)]
    
    print(f"\nRunning validation command:")
    print(" ".join(cmd))
    
    try:
        result = subprocess.run(cmd, check=True, capture_output=True, text=True)
        print("✓ Validation completed successfully!")
        print(result.stdout)
        
    except subprocess.CalledProcessError as e:
        print(f"❌ Validation failed: {e}")
        print("STDOUT:", e.stdout)
        print("STDERR:", e.stderr)
        return False
    
    # Show final structure
    print(f"\nFinal dataset structure:")
    print(f"{output_path}/")
    
    for item in sorted(output_path.rglob("*")):
        if item.is_file():
            rel_path = item.relative_to(output_path)
            indent = "  " * (len(rel_path.parts) - 1)
            print(f"{indent}├── {item.name}")
    
    print(f"\n🎉 LeRobot dataset is ready at: {output_path}")
    return True


def show_usage_instructions():
    """Show detailed usage instructions"""
    
    print("\nUsage Instructions:")
    print("=" * 50)
    
    print("\n1. Convert demonstration data to LeRobot format:")
    print("   python gello/data_utils/demo_to_lerobot.py <source_dir> [options]")
    print()
    print("   Options:")
    print("   --output_dir DIR     Output directory (default: <source_dir>/_lerobot)")
    print("   --robot_type TYPE    Robot type (default: xarm6_follower)")
    print("   --fps FPS            Video frame rate (default: 30)")
    print("   --max_episodes N     Limit number of episodes to process")
    print("   --chunk_size N       Number of episodes per chunk (default: 1000)")
    
    print("\n2. Validate the converted dataset:")
    print("   python gello/data_utils/validate_lerobot.py <dataset_dir>")
    
    print("\n3. Example data structure:")
    print("   Your source data should be organized as:")
    print("   bc_data/gello/")
    print("   ├── episode_001/")
    print("   │   ├── frame_000.pkl")
    print("   │   ├── frame_001.pkl")
    print("   │   └── ...")
    print("   ├── episode_002/")
    print("   └── ...")
    print()
    print("   Each pickle file should contain:")
    print("   - 'control': action/command data")
    print("   - 'joint_positions': robot joint states")
    print("   - 'wrist_rgb' or 'realsense_rgb': camera images")
    print("   - 'base_rgb': second camera view (optional)")
    
    print("\n4. Output structure (LeRobot format):")
    print("   dataset/")
    print("   ├── meta_data.json              # Dataset metadata")
    print("   ├── data/")
    print("   │   └── chunk-000/")
    print("   │       ├── episode_000000.parquet")
    print("   │       └── ...")
    print("   └── videos/")
    print("       └── chunk-000/")
    print("           ├── observation.images.up/")
    print("           │   ├── episode_000000.mp4")
    print("           │   └── ...")
    print("           └── observation.images.side/")
    print("               ├── episode_000000.mp4")
    print("               └── ...")


if __name__ == "__main__":
    if len(sys.argv) > 1 and sys.argv[1] == "--example":
        # Run the example conversion
        success = run_conversion_example()
        if not success:
            sys.exit(1)
    else:
        # Show usage instructions
        show_usage_instructions()
        
        print("\nTo run the example conversion:")
        print(f"python {__file__} --example")
        print("\n(Make sure to update the source_dir in the script first)")
