import open3d as o3d

def load_point_cloud(file_path):
    """
    Load a point cloud from a file.
    
    Args:
        file_path (str): Path to the point cloud file.
        
    Returns:
        open3d.geometry.PointCloud: Loaded point cloud.
    """
    pcd = o3d.io.read_point_cloud(file_path)
    return pcd

def save_point_cloud(pcd, file_path):

    """
    Save a point cloud to a file.
    
    Args:
        pcd (open3d.geometry.PointCloud): Point cloud to save.
        file_path (str): Path to save the point cloud.
    """
    o3d.io.write_point_cloud(file_path, pcd)

    print(f"Point cloud saved to {file_path}")
