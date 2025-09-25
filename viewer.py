#!/usr/bin/env python3
"""
3D Viewer for Point Clouds and Camera Poses
Designed to debug alignment issues between SLAM camera poses and 3DGS point clouds
"""

import open3d as o3d
import numpy as np
import argparse
from pathlib import Path


def load_point_cloud(file_path, format=None):
    """Load point cloud from various formats"""
    pcd = o3d.io.read_point_cloud(str(file_path))
    return pcd

def load_camera_poses(file_path, format="colmap_txt"):
    """Load camera poses from various formats"""
    if format == "colmap_txt":
        poses = load_camera_poses_colmap_txt(file_path)
    else:
        poses = []
    return poses


def load_camera_poses_colmap_txt(file_path):
    """
    Load camera poses from COLMAP images.txt file
    Format: IMAGE_ID, QW, QX, QY, QZ, TX, TY, TZ, CAMERA_ID, NAME
    """
    poses = []
    with open(file_path, 'r') as f:
        for line_num, line in enumerate(f, 1):
            line = line.strip()
            if not line or line.startswith('#'):
                continue

            parts = line.split()
            if len(parts) < 8:
                continue

            # Extract quaternion (QW, QX, QY, QZ) and translation (TX, TY, TZ)
            qw, qx, qy, qz = float(parts[1]), float(parts[2]), float(parts[3]), float(parts[4])
            tx, ty, tz = float(parts[5]), float(parts[6]), float(parts[7])

            # Convert quaternion to rotation matrix
            rot = np.array([
                [1-2*(qy*qy+qz*qz), 2*(qx*qy-qz*qw), 2*(qx*qz+qy*qw)],
                [2*(qx*qy+qz*qw), 1-2*(qx*qx+qz*qz), 2*(qy*qz-qx*qw)],
                [2*(qx*qz-qy*qw), 2*(qy*qz+qx*qw), 1-2*(qx*qx+qy*qy)]
            ])

            # Create 4x4 transformation matrix
            pose = np.eye(4)
            pose[:3, :3] = rot
            pose[:3, 3] = [tx, ty, tz]

            poses.append(pose)

    print(f"Loaded {len(poses)} camera poses from COLMAP format")
    return poses


def create_camera_frustum(pose, size=0.1, color=[1, 0, 0]):
    """Create a camera frustum visualization"""
    # Define camera frustum vertices in camera coordinate system
    vertices = np.array([
        [0, 0, 0],           # Camera center
        [-size, -size, size], # Bottom left
        [size, -size, size],  # Bottom right
        [size, size, size],   # Top right
        [-size, size, size],  # Top left
    ])

    # Transform vertices to world coordinates
    vertices_homogeneous = np.hstack([vertices, np.ones((5, 1))])
    vertices_world = (pose @ vertices_homogeneous.T).T[:, :3]

    # Define lines connecting the vertices
    lines = [
        [0, 1], [0, 2], [0, 3], [0, 4],  # From center to corners
        [1, 2], [2, 3], [3, 4], [4, 1]   # Rectangle at image plane
    ]

    # Create line set
    line_set = o3d.geometry.LineSet()
    line_set.points = o3d.utility.Vector3dVector(vertices_world)
    line_set.lines = o3d.utility.Vector2iVector(lines)
    line_set.colors = o3d.utility.Vector3dVector([color for _ in lines])

    return line_set


def create_coordinate_frame(pose, size=0.2):
    """Create coordinate frame visualization at camera pose"""
    return o3d.geometry.TriangleMesh.create_coordinate_frame(size=size).transform(pose)


def visualize_scene(point_cloud_path=None, camera_poses_path=None):
    """Main visualization function"""
    geometries = []

    # Load and add point cloud
    pcd = load_point_cloud(point_cloud_path)
    geometries.append(pcd)

    # Load and add camera poses
    if camera_poses_path:
        poses = load_camera_poses(camera_poses_path, format="colmap_txt")

        for i, pose in enumerate(poses):
            # Create camera frustum
            frustum = create_camera_frustum(pose, size=0.1)  # Red
            geometries.append(frustum)

            # Add coordinate frame
            frame = create_coordinate_frame(pose, size=0.08)
            geometries.append(frame)

    # Add world coordinate frame
    world_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.5)
    geometries.append(world_frame)


    # Launch visualizer with custom settings
    vis = o3d.visualization.Visualizer()
    vis.create_window(
        window_name="3D Point Cloud and Camera Poses Viewer",
        width=1200,
        height=800,
        left=50,
        top=50
    )

    for geometry in geometries:
        vis.add_geometry(geometry)

    # Get render option and adjust point size
    render_option = vis.get_render_option()
    render_option.point_size = 2.0  # Make points more visible
    render_option.background_color = np.array([0.1, 0.1, 0.1])  # Dark background

    vis.run()
    vis.destroy_window()


def main():
    parser = argparse.ArgumentParser(description="3D Viewer for Point Clouds and Camera Poses")
    parser.add_argument("--pointcloud", "-p", type=str,
                       help="Path to point cloud file (.ply, .pcd, .xyz, .pts)")
    parser.add_argument("--poses", "-c", type=str,
                       help="Path to camera poses file")

    args = parser.parse_args()

    visualize_scene(
        point_cloud_path=args.pointcloud,
        camera_poses_path=args.poses
    )


if __name__ == "__main__":
    main()