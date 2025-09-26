#!/usr/bin/env python3
"""
3D Viewer for Point Clouds and Camera Poses
Designed to debug alignment issues between SLAM camera poses and 3DGS point clouds
"""

import open3d as o3d
import numpy as np
import argparse
import json
from pathlib import Path
from plyfile import PlyData


def load_point_cloud(file_path, args):
    """Load point cloud from various formats"""

    # Check if this is a 3DGS file by looking for SH coefficients
    if args.pointcloud_format == "3dgs":
        # Custom loader for 3DGS PLY files. The normals in the 3dgs files are zero, so we ignore them.
        pcd = load_point_cloud_3dgs(file_path, use_sh=args.use_sh)
    else:
        pcd = o3d.io.read_point_cloud(str(file_path))

    print(f"Loaded point cloud from {file_path} with {len(pcd.points)} points")

    return pcd


def load_point_cloud_3dgs(file_path, use_sh=False):
    """Load point cloud from 3DGS PLY file"""
    print(f"Loading 3DGS from {file_path}")
    plydata = PlyData.read(file_path)
    vertices = plydata['vertex']

    pcd = o3d.geometry.PointCloud()

    x = vertices['x']
    y = vertices['y']
    z = vertices['z']
    points = np.stack([x, y, z], axis=1)

    pcd.points = o3d.utility.Vector3dVector(points)

    # Extract SH DC coefficients (0th order) for RGB
    sh_dc_0 = vertices['f_dc_0']  # Red channel
    sh_dc_1 = vertices['f_dc_1']  # Green channel
    sh_dc_2 = vertices['f_dc_2']  # Blue channel

    colors = np.stack([sh_dc_0, sh_dc_1, sh_dc_2], axis=1)

    if use_sh:
        C0 = 0.28209479177387814
        colors = 0.5 + C0 * colors  # Transform from SH to color space
    colors = np.clip(colors, 0, 1)  # Clamp to valid range

    pcd.colors = o3d.utility.Vector3dVector(colors)

    return pcd


def load_camera_poses(file_path, format=None):
    """Load camera poses from various formats"""
    if format is None:
        # Auto-detect format based on file extension
        file_ext = Path(file_path).suffix.lower()
        if file_ext == '.json':
            format = "nerf_json"
        elif file_ext == '.txt':
            format = "colmap_txt"

    if format == "colmap_txt":
        poses = load_camera_poses_colmap_txt(file_path)
    elif format == "nerf_json":
        poses = load_camera_poses_nerf_json(file_path)
    else:
        poses = []
    return poses


def load_camera_poses_colmap_txt(file_path):
    """
    Load camera poses from COLMAP images.txt file
    Format: IMAGE_ID, QW, QX, QY, QZ, TX, TY, TZ, CAMERA_ID, NAME
    """
    poses = []

    # Coordinate system conversion matrix: COLMAP to Open3D
    # COLMAP: X-right, Y-down, Z-forward
    # Open3D: X-right, Y-up, Z-back (for proper visualization)
    coord_transform = np.array([
        [1,  0,  0, 0],
        [0, -1,  0, 0],
        [0,  0, -1, 0],
        [0,  0,  0, 1]
    ])

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

            # Apply coordinate system transformation
            pose = pose @ coord_transform

            poses.append(pose)

    print(f"Loaded {len(poses)} camera poses from COLMAP format")
    return poses


def load_camera_poses_nerf_json(file_path):
    """
    Load camera poses from NeRF-style JSON file
    Expected format: {"frames": [{"transform_matrix": [[4x4 matrix]]}, ...]}
    """
    # Coordinate system conversion matrix: NeRF to Open3D
    # NeRF typically follows COLMAP convention: X-right, Y-down, Z-forward
    # Open3D: X-right, Y-up, Z-back (for proper visualization)
    coord_transform = np.array([
        [1,  0,  0, 0],
        [0, -1,  0, 0],
        [0,  0, -1, 0],
        [0,  0,  0, 1]
    ])

    with open(file_path, 'r') as f:
        data = json.load(f)

    poses = []
    for frame in data['frames']:
        transform_matrix = np.array(frame['transform_matrix'])
        # Apply coordinate system transformation
        transform_matrix = transform_matrix @ coord_transform
        poses.append(transform_matrix)

    print(f"Loaded {len(poses)} camera poses from NeRF JSON format")
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


def visualize_scene(point_cloud_path, camera_poses_path, args):
    """Main visualization function"""
    geometries = []

    # Load and add point cloud
    if point_cloud_path:
        pcd = load_point_cloud(point_cloud_path, args)
        geometries.append(pcd)

    # Load and add camera poses
    if camera_poses_path:
        poses = load_camera_poses(camera_poses_path, args.pose_format)

        for i, pose in enumerate(poses):
            if i > 5:  # Limit number of cameras for clarity
                break

            # Create camera frustum
            frustum = create_camera_frustum(pose, size=0.1)  # Red
            geometries.append(frustum)

    # Launch visualizer with custom settings
    vis = o3d.visualization.VisualizerWithKeyCallback()
    vis.create_window(
        window_name="3D Point Cloud and Camera Poses Viewer - Press N/P for next/prev camera",
        width=1200,
        height=800,
        left=50,
        top=50
    )

    for geometry in geometries:
        vis.add_geometry(geometry)

    # Get render option and adjust point size
    render_option = vis.get_render_option()
    render_option.point_size = args.point_size  # Make points more visible
    render_option.background_color = np.array([0.1, 0.1, 0.1])  # Dark background

    # Coordinate frames toggling
    coord_frames = []

    world_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.5)
    coord_frames.append(world_frame)

    if camera_poses_path:
        for i, pose in enumerate(poses):
            if i > 5:
                break
            frame = create_coordinate_frame(pose, size=0.08)
            coord_frames.append(frame)

    frames_visible = False

    def toggle_frames():
        nonlocal frames_visible
        frames_visible = not frames_visible
        for frame in coord_frames:
            if frames_visible:
                vis.add_geometry(frame)
            else:
                vis.remove_geometry(frame)
        vis.update_renderer()
        print(f"Coordinate frames: {'ON' if frames_visible else 'OFF'}")

    # Camera switching
    if camera_poses_path:
        current_camera_index = -1

        def switch_to_camera(index):
            nonlocal current_camera_index
            current_camera_index = index
            pose = poses[index]
            view_control = vis.get_view_control()

            # Set camera position using extrinsic matrix
            cam_params = view_control.convert_to_pinhole_camera_parameters()
            cam_params.extrinsic = np.linalg.inv(pose)
            view_control.convert_from_pinhole_camera_parameters(cam_params)
            print(f"Camera {index + 1}/{len(poses)}")

        vis.register_key_callback(ord("N"), lambda _: switch_to_camera((current_camera_index + 1) % len(poses)) or False)
        vis.register_key_callback(ord("P"), lambda _: switch_to_camera((current_camera_index - 1) % len(poses)) or False)

    vis.register_key_callback(ord("F"), lambda _: toggle_frames() or False)

    vis.run()
    vis.destroy_window()


def main():
    parser = argparse.ArgumentParser(description="3D Viewer for Point Clouds and Camera Poses")
    parser.add_argument("--pointcloud", "-p", type=str,
                       help="Path to point cloud file (.ply, .pcd, .xyz, .pts)")
    parser.add_argument("--pose", "-c", type=str,
                       help="Path to camera poses file")
    parser.add_argument("--pointcloud_format", type=str, default=None,
                       help="Format of point cloud file (e.g., '3dgs')")
    parser.add_argument("--pose_format", type=str, default=None,
                       help="Format of camera poses file (e.g., 'colmap_txt', 'nerf_json'). If not provided, auto-detected based on file extension.")
    parser.add_argument("--use_sh", action='store_true',
                       help="Use SH coefficients for coloring 3DGS point clouds")
    parser.add_argument("--point_size", type=float, default=2.0,
                       help="Point size for visualization")

    args = parser.parse_args()

    visualize_scene(
        point_cloud_path=args.pointcloud,
        camera_poses_path=args.pose,
        args=args
    )


if __name__ == "__main__":
    main()