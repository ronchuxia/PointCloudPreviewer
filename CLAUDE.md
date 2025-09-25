# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is a 3D visualization tool designed to debug alignment issues between SLAM camera poses and 3D Gaussian Splatting (3DGS) point clouds. The project was created to investigate misalignment between camera poses extracted from a feedforward SLAM method called PI3 and point cloud data used for 3DGS training.

## Architecture

The project consists of a single Python module (`viewer.py`) that provides:

- **Point Cloud Loading**: Supports multiple formats (.ply, .pcd, .xyz, .pts) via Open3D
- **Camera Pose Loading**: Flexible parser supporting two formats:
  - 4x4 transformation matrices (16 values per line)
  - Position + quaternion format (7 values: x,y,z,qx,qy,qz,qw)
- **3D Visualization**: Custom Open3D visualizer with camera frustums, coordinate frames, and point clouds
- **Camera Frustum Rendering**: Creates wireframe camera representations to visualize pose trajectories
- **Coordinate Frame Display**: Shows local coordinate systems at camera positions

## Key Components

### `load_point_cloud(file_path)`
Loads point clouds using Open3D, with error handling for empty files.

### `load_camera_poses(file_path)`
Flexible camera pose parser that handles both matrix and position+quaternion formats. Includes quaternion-to-rotation-matrix conversion.

### `create_camera_frustum(pose, size, color)`
Generates wireframe camera frustum geometry for pose visualization.

### `visualize_scene()`
Main visualization function that combines point clouds, camera poses, and coordinate frames in a single 3D scene.

## Common Commands

### Setup
```bash
pip install -r requirements.txt
```

### Running the Viewer
```bash
# Point cloud only
python viewer.py --pointcloud data/points3D.ply

# Camera poses only
python viewer.py --poses data/camera_poses.txt

# Both point cloud and camera poses
python viewer.py --pointcloud data/points3D.ply --poses data/camera_poses.txt

# Adjust camera frustum size
python viewer.py --pointcloud data/points3D.ply --camera-size 0.2

# Hide coordinate frames
python viewer.py --pointcloud data/points3D.ply --no-frames

# Quick run script
./run.sh
```

## Data Formats

### Point Clouds
- Supports: .ply, .pcd, .xyz, .pts
- Current data files: `data/points3D.ply`, `data/point_cloud.ply`
- Colors are preserved if present in the source file

### Camera Poses
Expected text file formats:
1. **4x4 Matrix format**: 16 space-separated values per line representing row-major transformation matrix
2. **Position + Quaternion format**: 7 space-separated values per line (x y z qx qy qz qw)

Lines starting with '#' are treated as comments and ignored.

## Visualization Controls
- Mouse: Rotate view
- Mouse wheel: Zoom
- Ctrl+Mouse: Pan
- 'h': Show additional controls
- 'q' or close window: Exit

## Technical Notes

- Uses Open3D for 3D rendering and geometry operations
- Camera frustums are rendered as red wireframe pyramids
- Coordinate frames use RGB axes (Red=X, Green=Y, Blue=Z)
- Point size is set to 2.0 pixels for better visibility
- Dark background (0.1, 0.1, 0.1) for better contrast
- World coordinate frame is displayed at origin with 0.5 unit size

## Problem Context

This tool addresses the specific challenge of debugging misaligned camera poses from PI3 SLAM output when used with 3DGS training data. The visualization allows inspection of spatial relationships between reconstructed camera trajectories and point cloud geometry to identify coordinate system mismatches or scale issues.

# Notes

Write minimal code the accomplish the task. The code should be as simple as possible. Avoid adding extra features unless necessary.