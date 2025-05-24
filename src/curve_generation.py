from scipy.interpolate import splprep, splev
import matplotlib.pyplot as plt
import numpy as np
import open3d as o3d
import curve_generation_args

from scipy.spatial import ConvexHull
from scipy.spatial import distance

def sort_points_along_edge(points):
    """
    Sort points along the edge to ensure proper order for curve fitting.
    Args:
        points (numpy.ndarray): Input points (Nx3).
    Returns:
        numpy.ndarray: Sorted points (Nx3).
    """
    
    hull = ConvexHull(points[:, :2])  # 仅对 x, y 进行排序
    sorted_indices = hull.vertices
    return points[sorted_indices]

def sort_points_by_nearest_neighbor(points):
    """
    Sort points by nearest neighbor to form a continuous path.
    Args:
        points (numpy.ndarray): Input points (Nx3).
    Returns:
        numpy.ndarray: Sorted points (Nx3).
    """
    sorted_points = [points[0]]  # 从第一个点开始
    remaining_points = points[1:]

    while len(remaining_points) > 0:
        last_point = sorted_points[-1]
        distances = distance.cdist([last_point], remaining_points)
        nearest_idx = distances.argmin()
        sorted_points.append(remaining_points[nearest_idx])
        remaining_points = np.delete(remaining_points, nearest_idx, axis=0)

    return np.array(sorted_points)

def fit_curve(edge_points, smoothing=0):
    """
    Fit a B-Spline curve to the edge points.

    Args:
        edge_points (numpy.ndarray): Edge points (Nx3).
        smoothing (float): Smoothing factor for the spline.

    Returns:
        numpy.ndarray: Fitted curve points (Nx3).
    """
    # Extract x, y, z coordinates
    x, y, z = edge_points[:, 0], edge_points[:, 1], edge_points[:, 2]

    # Fit a B-Spline curve with periodicity (closed curve)
    tck, u = splprep([x, y, z], s=smoothing, per=True)  # Set per=True for closed curve
    u_fine = np.linspace(0, 1, 1000)  # Generate fine-grained parameter values
    x_fine, y_fine, z_fine = splev(u_fine, tck)

    # Return the fitted curve points
    return np.vstack([x_fine, y_fine, z_fine]).T

def generate_surface_from_edges(edge_points):
    """
    Generate a surface from edge points using triangulation.

    Args:
        edge_points (numpy.ndarray): Edge points (Nx3).

    Returns:
        open3d.geometry.TriangleMesh: Generated surface mesh.
    """
    # Create a point cloud from edge points
    edge_pcd = o3d.geometry.PointCloud()
    edge_pcd.points = o3d.utility.Vector3dVector(edge_points)

    # Perform alpha shape triangulation to generate a surface
    alpha = 1.0  # Adjust alpha value for surface tightness
    mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_alpha_shape(edge_pcd, alpha)

    return mesh

def compute_plane_normal(points):
    """
    Compute the normal vector of the plane fitted to the given points.

    Args:
        points (numpy.ndarray): Input points (Nx3).

    Returns:
        numpy.ndarray: Normal vector of the fitted plane (3,).
    """
    # Compute the centroid of the points
    centroid = np.mean(points, axis=0)

    # Compute the covariance matrix
    cov_matrix = np.cov(points - centroid, rowvar=False)

    # Compute the eigenvalues and eigenvectors
    eigenvalues, eigenvectors = np.linalg.eigh(cov_matrix)

    # The eigenvector corresponding to the smallest eigenvalue is the normal
    normal = eigenvectors[:, 0]
    return normal

def generate_surface_with_bpa(edge_points, radius=0.1):
    """
    Generate a surface using Ball Pivoting Algorithm (BPA).

    Args:
        edge_points (numpy.ndarray): Edge points (Nx3).
        radius (float): Ball radius for pivoting.

    Returns:
        open3d.geometry.TriangleMesh: Generated surface mesh.
    """
    # Create a point cloud from edge points
    edge_pcd = o3d.geometry.PointCloud()
    edge_pcd.points = o3d.utility.Vector3dVector(edge_points)

    # Compute the plane normal
    plane_normal = compute_plane_normal(edge_points)

    # Assign the same normal to all points
    normals = np.tile(plane_normal, (len(edge_points), 1))
    edge_pcd.normals = o3d.utility.Vector3dVector(normals)

    # Apply BPA to generate the surface
    distances = edge_pcd.compute_nearest_neighbor_distance()
    avg_distance = np.mean(distances)
    radius = avg_distance * 1.5  # Adjust radius based on point spacing
    mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(
        edge_pcd, o3d.utility.DoubleVector([radius, radius * 2])
    )

    return mesh

def generate_surface_with_poisson(edge_points):
    """
    Generate a surface from edge points using Poisson reconstruction.

    Args:
        edge_points (numpy.ndarray): Edge points (Nx3).

    Returns:
        open3d.geometry.TriangleMesh: Generated surface mesh.
    """
    # Create a point cloud from edge points
    edge_pcd = o3d.geometry.PointCloud()
    edge_pcd.points = o3d.utility.Vector3dVector(edge_points)

    # Perform Poisson reconstruction to generate a surface
    mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(
        edge_pcd, depth=9
    )

    return mesh
def cut_mesh_with_surface(source_mesh, cutting_surface):
    """
    Cut a source mesh with a cutting surface.

    Args:
        source_mesh (open3d.geometry.TriangleMesh): The mesh to be cut.
        cutting_surface (open3d.geometry.TriangleMesh): The surface used for cutting.

    Returns:
        open3d.geometry.TriangleMesh: The cut mesh.
    """
    # Ensure both meshes are in the correct format
    if not isinstance(source_mesh, o3d.geometry.TriangleMesh):
        raise ValueError("source_mesh must be an open3d.geometry.TriangleMesh")
    if not isinstance(cutting_surface, o3d.geometry.TriangleMesh):
        raise ValueError("cutting_surface must be an open3d.geometry.TriangleMesh")

    # Compute the intersection of the source mesh and the cutting surface
    # Note: Open3D's boolean operations may require the meshes to be manifold and watertight
    try:
        cut_mesh = source_mesh.intersection(cutting_surface)
        return cut_mesh
    except Exception as e:
        print(f"An error occurred during mesh cutting: {e}")
        return None

# Example usage
if __name__ == "__main__":
    args = curve_generation_args.get_args()
    input_file = args.input
    output_file = args.output
    smoothing = args.smoothing
    visualize = args.visualize

    # Load the point cloud
    pcd = o3d.io.read_point_cloud(input_file)
    print(f"Loaded point cloud with {len(pcd.points)} points from {input_file}")

    # Convert point cloud to numpy array
    points = np.asarray(pcd.points)
    points = sort_points_by_nearest_neighbor(points)
    # points = sort_points_along_edge(points)
    # Fit a curve to the edge points
    fitted_curve = fit_curve(points, smoothing=smoothing)
    #Save the sorted edge points to a file
    edge_output_file = output_file.replace(".ply", "_edges.ply")  # 生成边缘文件名
    edge_pcd = o3d.geometry.PointCloud()
    edge_pcd.points = o3d.utility.Vector3dVector(points)
    o3d.io.write_point_cloud(edge_output_file, edge_pcd)
    print(f"Sorted edge points saved to {edge_output_file}")
    print(f"Fitted curve with {len(fitted_curve)} points")
    # Save the fitted curve to a file
    curve_output_file = output_file.replace(".ply", "_curve.ply")  # 生成曲线文件名
    curve_pcd = o3d.geometry.PointCloud()
    curve_pcd.points = o3d.utility.Vector3dVector(fitted_curve)
    o3d.io.write_point_cloud(curve_output_file, curve_pcd)
    print(f"Fitted curve saved to {curve_output_file}")

    # Generate a surface from the edge points
    surface_mesh = generate_surface_from_edges(fitted_curve)

    # Save the generated surface mesh
    o3d.io.write_triangle_mesh(output_file, surface_mesh)
    print(f"Generated surface mesh saved to {output_file}")

    #Generate a surface using BPA
    surface_mesh_bpa = generate_surface_with_bpa(fitted_curve)
    # Save the generated surface mesh using BPA
    bpa_output_file = output_file.replace(".ply", "_bpa.ply")  # 生成BPA曲面文件名
    o3d.io.write_triangle_mesh(bpa_output_file, surface_mesh_bpa)
    print(f"Generated surface mesh using BPA saved to {bpa_output_file}")

    # Visualize the results if requested
    if visualize:
        o3d.visualization.draw_geometries([pcd, curve_pcd, surface_mesh])