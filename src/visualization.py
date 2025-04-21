import open3d as o3d

import numpy as np

def visualize_point_cloud(pcd, title="Point Cloud"):
    """
    Visualize a point cloud using Open3D.
    
    Args:
        pcd (open3d.geometry.PointCloud): Point cloud to visualize.
        title (str): Title of the visualization window.
    """
    
    vis = o3d.visualization.Visualizer()
    vis.create_window(title=title)
    vis.add_geometry(pcd)
    vis.run()
    vis.destroy_window()

