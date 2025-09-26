import open3d as o3d
import open3d.visualization.gui as gui
import open3d.visualization.rendering as rendering

def main():
    gui.Application.instance.initialize()
    window = gui.Application.instance.create_window("Open3D Fly Mode", 1024, 768)

    widget = gui.SceneWidget()
    window.add_child(widget)

    widget.scene = rendering.Open3DScene(window.renderer)
    widget.scene.set_background([0.1, 0.1, 0.1, 1.0])

    geom = o3d.geometry.TriangleMesh.create_coordinate_frame(size=1.0)
    mat = rendering.MaterialRecord()
    mat.shader = "defaultUnlit"
    widget.scene.add_geometry("axes", geom, mat)

    bounds = geom.get_axis_aligned_bounding_box()
    widget.setup_camera(60, bounds, bounds.get_center())

    # 🔹 Now camera_control is available
    widget.camera_control.set_mode(gui.SceneWidget.CameraControl.FLY)

    gui.Application.instance.run()

if __name__ == "__main__":
    main()
