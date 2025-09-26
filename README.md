# PointCloudPreviewer

A simple 3D point cloud viewer with camera frustum visualization using Open3D.

Key Features:
- Visualize 3D point cloud and camera poses.
- Support PLY and 3DGS PLY format for point clouds.
- Support COLMAP TXT and NERF JSON format for camera poses.
- Interactive fly mode camera navigation with keyboard controls. Easy to navigate indoor scenes.
- Support camera switching for quick preview of camera views.

## Installation

```bash
pip install numpy open3d plyfile
```

## Usage

There are two implementations. One is based on Open3D's built-in GUI fly mode (`viewer_gui.py`), the other is a custom implementation using simple coordinate transformations (`viewer.py`).

### GUI Version

```bash
python viewer_gui.py --pointcloud <path_to_pointcloud> --pose <path_to_camera_file>
```

### Non-GUI Version

```bash
python viewer.py --pointcloud <path_to_pointcloud> --pose <path_to_camera_file>
```

### Command Line Arguments

- `--pointcloud`: Path to the point cloud file (PLY or 3DGS PLY format).
- `--pose`: Path to the camera pose file (COLMAP TXT or NERF JSON format).
- `--pointcloud_format`: Format of the point cloud file (None or '3dgs'). Default is None. None for standard PLY and other formats that open3D supports, '3dgs' for 3DGS PLY format.
- `--pose_format`: Format of the camera pose file ('colmap_txt' or 'nerf_json'). If not specified, the format will be inferred from the file extension.
- `--use_sh`: Convert spherical harmonics to RGB. Only works for 3DGS PLY format. If not specified, the program will directly interpret the DC (0th order) SH coefficients as RGB values.
- `--point_size`: Size of the points in the point cloud. Default is 2.0.

### Controls (Open3D GUI Fly Mode)
- `W`, `A`, `S`, `D`, `Q`, `Z`: Move forward, left, backward, right, up, down.
- Mouse Drag: Rotate view.
- `F`: Toggle coordinate frame.
- `N`, `P`: Next/Previous camera.
- `R`: Reset camera to initial position.

### Controls (Custom Implementation)
- `W`, `A`, `S`, `D`, `Q`, `E`: Move forward, left, backward, right, up, down.
- `I`, `J`, `K`, `L`, `U`, `O`: Rotate view.
- `F`: Toggle coordinate frame.
- `N`, `P`: Next/Previous camera.
- `R`: Reset camera to initial position.
- `T`, `G`: Increase/Decrease movement speed.
- `Y`, `H`: Increase/Decrease rotation speed.
- Mouse Drag: Rotate view in ARCBALL mode.