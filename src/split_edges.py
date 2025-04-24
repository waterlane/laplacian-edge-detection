import numpy as np
import open3d as o3d

def split_edges_by_angle(point_cloud, angle_threshold):
    """
    Splits edges in a point cloud based on the angle between normals.

    Args:
        point_cloud (o3d.geometry.PointCloud): Input point cloud.
        angle_threshold (float): Angle threshold in degrees to split edges.

    Returns:
        list: A list of point clouds, each representing a segment.
    """
    # Estimate normals for the point cloud
    point_cloud.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
    
    # Convert angle threshold to radians
    angle_threshold_rad = np.deg2rad(angle_threshold)
    
    # Compute adjacency list
    distances = np.asarray(point_cloud.compute_nearest_neighbor_distance())
    avg_distance = np.mean(distances)
    adjacency_list = point_cloud.compute_adjacency_list(avg_distance * 2)
    
    # Initialize visited points and segments
    visited = np.zeros(len(point_cloud.points), dtype=bool)
    segments = []

    def dfs(point_idx, segment):
        """Depth-first search to group points into a segment."""
        stack = [point_idx]
        while stack:
            idx = stack.pop()
            if visited[idx]:
                continue
            visited[idx] = True
            segment.append(idx)
            for neighbor_idx in adjacency_list[idx]:
                if not visited[neighbor_idx]:
                    normal1 = point_cloud.normals[idx]
                    normal2 = point_cloud.normals[neighbor_idx]
                    angle = np.arccos(np.clip(np.dot(normal1, normal2), -1.0, 1.0))
                    if angle < angle_threshold_rad:
                        stack.append(neighbor_idx)

    # Split edges into segments
    for i in range(len(point_cloud.points)):
        if not visited[i]:
            segment = []
            dfs(i, segment)
            segments.append(segment)

    # Create point clouds for each segment
    segmented_point_clouds = []
    for segment in segments:
        segment_cloud = point_cloud.select_by_index(segment)
        segmented_point_clouds.append(segment_cloud)

    return segmented_point_clouds

# Example usage
if __name__ == "__main__":
    # Load a point cloud
    pcd = o3d.io.read_point_cloud("example.ply")

    # Split edges based on angle threshold
    angle_threshold = 30.0  # degrees
    segments = split_edges_by_angle(pcd, angle_threshold)

    # Save each segment as a separate file
    for i, segment in enumerate(segments):
        o3d.io.write_point_cloud(f"segment_{i}.ply", segment)