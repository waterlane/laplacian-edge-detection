from scipy.spatial import ConvexHull
import numpy as np
import open3d as o3d

def compute_convex_hull(points):
    """
    Compute the convex hull of a 3D point cloud.

    Parameters:
        points (numpy.ndarray): A (N, 3) array of 3D points.

    Returns:
        ConvexHull: The ConvexHull object containing the hull information.
    """
    if points.shape[1] != 3:
        raise ValueError("Input points must be a (N, 3) array representing 3D points.")
    
    hull = ConvexHull(points)
    return hull

def load_pcd(file_path):
    """
    Load a point cloud from a PCD file.

    Parameters:
        file_path (str): Path to the PCD file.

    Returns:
        numpy.ndarray: A (N, 3) array of 3D points.
    """
    pcd = o3d.io.read_point_cloud(file_path)
    points = np.asarray(pcd.points)
    return points

def save_convex_hull_to_ply(hull, output_path):
    """
    Save the convex hull edge points as a .ply file.

    Parameters:
        hull (ConvexHull): The ConvexHull object.
        output_path (str): Path to save the .ply file.
    """
    # 提取凸包的顶点
    edge_points = hull.points[hull.vertices]

    # 创建 Open3D 点云对象
    edge_pcd = o3d.geometry.PointCloud()
    edge_pcd.points = o3d.utility.Vector3dVector(edge_points)

    # 保存为 .ply 文件
    o3d.io.write_point_cloud(output_path, edge_pcd)
    print(f"Convex hull edge points saved to {output_path}")

def main():
    file_path = "../data/cloth/edge_ShirtNoCoat_segment_1.ply"
    points = load_pcd(file_path)    
    # Compute the convex hull
    hull = compute_convex_hull(points)

    # Print the vertices of the convex hull
    print("Vertices of the convex hull:")
    print(hull.vertices)

    # Print the simplices (triangular faces) of the convex hull
    print("Simplices (triangular faces) of the convex hull:")
    print(hull.simplices)

    # Save the convex hull to a .ply file
    output_path = "../data/output/edge_ShirtNoCoat_segment_1_convex_hull.ply"
    save_convex_hull_to_ply(hull, output_path)
  

if __name__ == "__main__":
    main()