import open3d as o3d

def visualize_point_cloud(pcd, title="Point Cloud", output_image="output.png"):
    """
    Visualize a point cloud using Open3D in headless mode and save as an image.

    Args:
        pcd (open3d.geometry.PointCloud): Point cloud to visualize.
        title (str): Title of the visualization window.
        output_image (str): Path to save the rendered image.
    """
    vis = o3d.visualization.VisualizerWithOffscreen()
    vis.create_window(window_name=title, visible=False)  # Headless mode
    vis.add_geometry(pcd)
    vis.poll_events()
    vis.update_renderer()
    vis.capture_screen_image(output_image)  # Save the visualization as an image
    vis.destroy_window()
    print(f"Saved visualization to {output_image}")