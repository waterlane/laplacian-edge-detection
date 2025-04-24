from scipy.interpolate import splprep, splev
import matplotlib.pyplot as plt

def fit_curve(edge_points, smoothing=0):
    """
    Fit a B-Spline curve to the edge points.

    Args:
        edge_points (numpy.ndarray): Edge points (Nx3).
        smoothing (float): Smoothing factor for the spline.

    Returns:
        tuple: Fitted curve points (numpy.ndarray).
    """
    # Extract x, y, z coordinates
    x, y, z = edge_points[:, 0], edge_points[:, 1], edge_points[:, 2]

    # Fit a B-Spline curve
    tck, u = splprep([x, y, z], s=smoothing)
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
    alpha = 0.1  # Adjust alpha value for surface tightness
    mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_alpha_shape(edge_pcd, alpha)

    return mesh

