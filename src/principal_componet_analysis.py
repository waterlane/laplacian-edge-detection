import numpy as np
import open3d as o3d
import os

import principal_componet_analysis_args 

def load_point_cloud(file_path):
    """Load a 3D point cloud from a file."""
    point_cloud = o3d.io.read_point_cloud(file_path)
    if not point_cloud.has_points():
        raise ValueError("The point cloud is empty or invalid.")
    return point_cloud

def compute_principal_directions(point_cloud):
    """Compute the principal directions of a 3D point cloud."""
    points = np.asarray(point_cloud.points)
    mean_centered = points - np.mean(points, axis=0)
    covariance_matrix = np.cov(mean_centered, rowvar=False)
    eigenvalues, eigenvectors = np.linalg.eigh(covariance_matrix)
    # Sort eigenvectors by eigenvalues in descending order
    sorted_indices = np.argsort(eigenvalues)[::-1]
    principal_directions = eigenvectors[:, sorted_indices]
    return principal_directions

def save_principal_directions(principal_directions, file_path):
    """Save the principal directions to a file."""
    np.savetxt(file_path, principal_directions, delimiter=',', header='Principal Directions', comments='')
    print(f"Principal directions saved to {file_path}")

def save_principal_directions_as_npy(principal_directions, file_path):
    """Save the principal directions to a .npy file."""
    np.save(file_path, principal_directions)
    print(f"Principal directions saved to {file_path}")

def visualize_with_principal_directions(point_cloud, principal_directions):
    """Visualize the point cloud with principal directions."""
    origin = np.mean(np.asarray(point_cloud.points), axis=0)
    lines = [
        [0, 1], [0, 2], [0, 3]
    ]
    colors = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]  # RGB for XYZ axes
    points = [origin]
    for direction in principal_directions.T:
        points.append(origin + direction * 0.5)  # Scale for visualization
    line_set = o3d.geometry.LineSet()
    line_set.points = o3d.utility.Vector3dVector(points)
    line_set.lines = o3d.utility.Vector2iVector(lines)
    line_set.colors = o3d.utility.Vector3dVector(colors)
    o3d.visualization.draw_geometries([point_cloud, line_set])

if __name__ == "__main__":
    # Replace 'your_point_cloud_file.ply' with your actual file path
    # file_path = "/root/projects/laplacian-edge-detection/data/cloth/edge_ShirtNoCoat_segment_1.ply"\

    args = principal_componet_analysis_args.get_args()
    file_path = args.input
    

    file_folder = file_path.split("/")[:-2]
    file_folder = "/".join(file_folder)
    print(file_folder)
    file_name = file_path.split("/")[-1].split(".")[0]
    print(file_name)

    if not os.path.exists(file_folder):
        raise FileNotFoundError(f"Directory not found: {file_folder}")
    if not os.path.exists(file_path):
        raise FileNotFoundError(f"File not found: {file_path}")
    point_cloud = load_point_cloud(file_path)
    principal_directions = compute_principal_directions(point_cloud)
    print("Principal directions computed.")
    principal_directions_name = file_folder + "/output/" + file_name + "_principal_directions"
    save_principal_directions(principal_directions, principal_directions_name + ".txt")
    save_principal_directions_as_npy(principal_directions, principal_directions_name + ".npy")
    print(f"Principal directions saved as {principal_directions_name}.txt and {principal_directions_name}.npy")
    # visualize_with_principal_directions(point_cloud, principal_directions)