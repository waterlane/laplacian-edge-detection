import numpy as np
from scipy.spatial import ConvexHull

def sort_points_3d(points):
    """
    Sorts 3D points around their centroid and forms a polygonal cross-section.

    Args:
        points (np.ndarray): Nx3 array of 3D points.

    Returns:
        np.ndarray: Sorted Nx3 array of 3D points forming a convex polygon.
    """
    # Compute the centroid of the points
    centroid = np.mean(points, axis=0)

    # Project points onto a 2D plane (e.g., XY plane) relative to the centroid
    points_2d = points[:, :2] - centroid[:2]

    # Compute angles of points relative to the centroid
    angles = np.arctan2(points_2d[:, 1], points_2d[:, 0])

    # Sort points by angle
    sorted_indices = np.argsort(angles)
    sorted_points = points[sorted_indices]

    # Ensure the sorted points form a convex polygon
    hull = ConvexHull(sorted_points)
    convex_points = sorted_points[hull.vertices]

    return convex_points

# Example usage
if __name__ == "__main__":
    # Example 3D points
    points = np.array([
        [1.0, 2.0, 0.0],
        [2.0, 1.0, 0.0],
        [3.0, 3.0, 0.0],
        [1.5, 1.5, 0.0],
        [2.5, 2.5, 0.0]
    ])

    sorted_points = sort_points_3d(points)
    print("Sorted Points Forming Convex Polygon:")
    print(sorted_points)