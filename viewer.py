#!/usr/bin/env python3
"""
3D Viewer for Point Clouds and Camera Poses
Designed to debug alignment issues between SLAM camera poses and 3DGS point clouds
"""

import open3d as o3d
import numpy as np
import argparse
import json
import struct
from pathlib import Path
from plyfile import PlyData


def load_point_cloud(file_path, args):
    """Load point cloud from various formats"""

    if args.pointcloud_format == "3dgs":
        # Custom loader for 3DGS PLY files. The normals in the 3dgs files are zero, so we ignore them.
        pcd = load_point_cloud_3dgs(file_path, use_sh=args.use_sh)
    else:
        pcd = o3d.io.read_point_cloud(str(file_path))

        # Clear normals to avoid rendering issues with zero normals
        # This is especially important for 3DGS PLY files
        if pcd.has_normals():
            pcd.normals = o3d.utility.Vector3dVector()

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
        elif file_ext == '.bin':
            format = "colmap_bin"

    if format == "colmap_txt":
        poses = load_camera_poses_colmap_txt(file_path)
    elif format == "colmap_bin":
        poses = load_camera_poses_colmap_bin(file_path)
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
            extrinsics = np.eye(4)
            extrinsics[:3, :3] = rot
            extrinsics[:3, 3] = [tx, ty, tz]

            pose = np.linalg.inv(extrinsics)  # Invert to get camera-to-world
            poses.append(pose)

    print(f"Loaded {len(poses)} camera poses from COLMAP format")
    return poses


def load_camera_poses_colmap_bin(file_path):
    """
    Load camera poses from COLMAP images.bin file
    Binary format: num_images (uint64), then for each image:
    - IMAGE_ID (uint32)
    - QW, QX, QY, QZ (4 * double)
    - TX, TY, TZ (3 * double)
    - CAMERA_ID (uint32)
    - NAME (null-terminated string)
    - num_points2D (uint64)
    - POINT2D data (skipped for pose extraction)
    """
    poses = []

    with open(file_path, 'rb') as f:
        # Read number of images
        num_images = struct.unpack('<Q', f.read(8))[0]

        for _ in range(num_images):
            # Read IMAGE_ID
            image_id = struct.unpack('<I', f.read(4))[0]

            # Read quaternion (QW, QX, QY, QZ)
            qw, qx, qy, qz = struct.unpack('<4d', f.read(32))

            # Read translation (TX, TY, TZ)
            tx, ty, tz = struct.unpack('<3d', f.read(24))

            # Read CAMERA_ID
            camera_id = struct.unpack('<I', f.read(4))[0]

            # Read image name (null-terminated string)
            name = b''
            while True:
                char = f.read(1)
                if char == b'\0' or not char:
                    break
                name += char

            # Read number of 2D points
            num_points2d = struct.unpack('<Q', f.read(8))[0]

            # Skip 2D point data (each point is 3 * double = 24 bytes)
            f.seek(num_points2d * 24, 1)

            # Convert quaternion to rotation matrix
            rot = np.array([
                [1-2*(qy*qy+qz*qz), 2*(qx*qy-qz*qw), 2*(qx*qz+qy*qw)],
                [2*(qx*qy+qz*qw), 1-2*(qx*qx+qz*qz), 2*(qy*qz-qx*qw)],
                [2*(qx*qz-qy*qw), 2*(qy*qz+qx*qw), 1-2*(qx*qx+qy*qy)]
            ])

            # Create 4x4 transformation matrix
            extrinsics = np.eye(4)
            extrinsics[:3, :3] = rot
            extrinsics[:3, 3] = [tx, ty, tz]
            
            pose = np.linalg.inv(extrinsics)  # Invert to get camera-to-world
            poses.append(pose)

    print(f"Loaded {len(poses)} camera poses from COLMAP binary format")
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
    num_cameras_to_draw = 50  # Limit number of cameras drawn for clarity
    geometries = []

    # Load and add point cloud
    if point_cloud_path:
        pcd = load_point_cloud(point_cloud_path, args)
        geometries.append(pcd)

    # Load and add camera poses
    if camera_poses_path:
        poses = load_camera_poses(camera_poses_path, args.pose_format)

        for i, pose in enumerate(poses):
            if i > num_cameras_to_draw:  # Limit number of cameras for clarity
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

    # Adjust clipping planes for large scenes to prevent clipping
    if point_cloud_path:
        view_control = vis.get_view_control()

        # Get scene bounds to calculate appropriate clipping planes
        bbox = pcd.get_axis_aligned_bounding_box()
        scene_size = np.linalg.norm(bbox.get_extent())

        # Set near/far clipping planes based on scene size
        near_plane = scene_size * 0.001  # 0.1% of scene size
        far_plane = scene_size * 5.0    # 5x scene size

        # Set the clipping planes
        view_control.set_constant_z_near(near_plane)
        view_control.set_constant_z_far(far_plane)

        print(f"Scene size: {scene_size:.2f}, Near: {near_plane:.3f}, Far: {far_plane:.1f}")


    # Coordinate frames toggling
    coord_frames = []

    world_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.5)
    coord_frames.append(world_frame)

    if camera_poses_path:
        for i, pose in enumerate(poses):
            if i > num_cameras_to_draw:
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


    # Blender-style navigation
    move_speed = 0.05
    rotate_speed = 0.05  # Rotation angle in radians

    def adjust_speed(faster):
        nonlocal move_speed
        if faster:
            move_speed *= 1.5
        else:
            move_speed /= 1.5
        print(f"Move speed: {move_speed:.3f}")

    def adjust_rotate_speed(faster):
        nonlocal rotate_speed
        if faster:
            rotate_speed *= 1.5
        else:
            rotate_speed /= 1.5
        print(f"Rotate speed: {rotate_speed:.3f} rad ({rotate_speed * 180 / np.pi:.1f}°)")

    def move_camera(direction):
        view_control = vis.get_view_control()
        cam_params = view_control.convert_to_pinhole_camera_parameters()

        # Get current camera orientation
        W2C = cam_params.extrinsic
        C2W = np.linalg.inv(W2C)
        R = C2W[:3, :3]
        t = C2W[:3, 3]

        # Camera coordinate system: right, up, forward vectors
        # Note: This is for OpenCV/COLMAP coord system
        right = R[:, 0]  # X axis
        up = -R[:, 1]    # -Y axis
        forward = R[:, 2]  # Z axis

        # Apply movement
        if direction == "forward":
            new_t = t + forward * move_speed
        elif direction == "backward":
            new_t = t - forward * move_speed
        elif direction == "left":
            new_t = t - right * move_speed
        elif direction == "right":
            new_t = t + right * move_speed
        elif direction == "up":
            new_t = t + up * move_speed
        elif direction == "down":
            new_t = t - up * move_speed

        # Update camera position
        new_C2W = np.eye(4)
        new_C2W[:3, :3] = R
        new_C2W[:3, 3] = new_t
        new_W2C = np.linalg.inv(new_C2W)
        cam_params.extrinsic = new_W2C
        view_control.convert_from_pinhole_camera_parameters(cam_params)

    def rotate_camera(axis, angle):
        view_control = vis.get_view_control()
        cam_params = view_control.convert_to_pinhole_camera_parameters()

        # Get current camera pose
        W2C = cam_params.extrinsic
        C2W = np.linalg.inv(W2C)
        R = C2W[:3, :3]
        t = C2W[:3, 3]

        # Camera coordinate system axes
        right = R[:, 0]   # X axis
        up = -R[:, 1]     # -Y axis
        forward = R[:, 2] # Z axis

        # Create rotation matrix around the specified axis
        if axis == "pitch":  # Rotate around right (X) axis
            rot_axis = right
        elif axis == "yaw":  # Rotate around up (Y) axis
            rot_axis = up
        elif axis == "roll": # Rotate around forward (Z) axis
            rot_axis = forward
        else:
            return

        # Rodrigues' rotation formula
        cos_angle = np.cos(angle)
        sin_angle = np.sin(angle)
        cross_matrix = np.array([
            [0, -rot_axis[2], rot_axis[1]],
            [rot_axis[2], 0, -rot_axis[0]],
            [-rot_axis[1], rot_axis[0], 0]
        ])

        rotation_matrix = (cos_angle * np.eye(3) +
                          sin_angle * cross_matrix +
                          (1 - cos_angle) * np.outer(rot_axis, rot_axis))

        # Apply rotation to camera orientation
        new_R = rotation_matrix @ R

        # Update camera pose
        new_C2W = np.eye(4)
        new_C2W[:3, :3] = new_R
        new_C2W[:3, 3] = t
        new_W2C = np.linalg.inv(new_C2W)
        cam_params.extrinsic = new_W2C
        view_control.convert_from_pinhole_camera_parameters(cam_params)


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


    # Navigation controls
    vis.register_key_callback(ord("W"), lambda _: move_camera("forward") or False)
    vis.register_key_callback(ord("S"), lambda _: move_camera("backward") or False)
    vis.register_key_callback(ord("A"), lambda _: move_camera("left") or False)
    vis.register_key_callback(ord("D"), lambda _: move_camera("right") or False)
    vis.register_key_callback(ord("Q"), lambda _: move_camera("up") or False)
    vis.register_key_callback(ord("E"), lambda _: move_camera("down") or False)

    vis.register_key_callback(ord("F"), lambda _: toggle_frames() or False)
    vis.register_key_callback(ord("T"), lambda _: adjust_speed(True) or False)   # Faster
    vis.register_key_callback(ord("G"), lambda _: adjust_speed(False) or False)  # Slower
    vis.register_key_callback(ord("Y"), lambda _: adjust_rotate_speed(True) or False)   # Rotate faster
    vis.register_key_callback(ord("H"), lambda _: adjust_rotate_speed(False) or False)  # Rotate slower

    # Rotation controls
    vis.register_key_callback(ord("I"), lambda _: rotate_camera("pitch", -rotate_speed) or False)  # Pitch up
    vis.register_key_callback(ord("K"), lambda _: rotate_camera("pitch", rotate_speed) or False)   # Pitch down
    vis.register_key_callback(ord("J"), lambda _: rotate_camera("yaw", rotate_speed) or False)     # Yaw left
    vis.register_key_callback(ord("L"), lambda _: rotate_camera("yaw", -rotate_speed) or False)    # Yaw right
    vis.register_key_callback(ord("U"), lambda _: rotate_camera("roll", -rotate_speed) or False)    # Roll left
    vis.register_key_callback(ord("O"), lambda _: rotate_camera("roll", rotate_speed) or False)     # Roll right

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