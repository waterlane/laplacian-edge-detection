import open3d as o3d
import numpy as np
from sklearn.cluster import DBSCAN
from utils import load_point_cloud, save_point_cloud
import edge_detection_args

import os

from visualization import visualize_point_cloud

def compute_edges_dynamic_radius(pcd, base_radius=0.05, threshold=0.1):
    """
    Compute edges in a point cloud using the Laplacian operator with dynamic neighborhood radius.
    
    Args:
        pcd (open3d.geometry.PointCloud): Input point cloud.
        base_radius (float): Base radius for dynamic neighborhood search.
        threshold (float): Threshold for edge detection.
        
    Returns:
        open3d.geometry.PointCloud: Point cloud with edges highlighted.
    """
    # Convert point cloud to numpy array
    points = np.asarray(pcd.points)
    edges = []

    # Create KDTree for nearest neighbor search
    pcd_tree = o3d.geometry.KDTreeFlann(pcd)

    # Estimate average point spacing (optional, for dynamic radius adjustment)
    avg_spacing = np.mean(np.linalg.norm(points - np.mean(points, axis=0), axis=1))

    for i, point in enumerate(points):
        # Dynamically adjust the search radius based on the base radius and average spacing
        dynamic_radius = base_radius * (1 + avg_spacing)

        # Find neighbors within the dynamic radius
        [k, idx, _] = pcd_tree.search_radius_vector_3d(point, dynamic_radius)
        neighbors = points[idx]

        # Skip if no neighbors are found
        if len(neighbors) <= 1:
            print(f"Point {i} has no neighbors within radius {dynamic_radius}. Skipping.")
            continue

        # Compute the Laplacian
        laplacian = np.mean(neighbors, axis=0) - point
        laplacian_norm = np.linalg.norm(laplacian)
        # print (f"Point {i}: Laplacian norm = {laplacian_norm}")
        # Check if the Laplacian norm exceeds the threshold
        if laplacian_norm > threshold:
            edges.append(point)
        #     print (f"Point {i}: Edge detected with Laplacian norm = {laplacian_norm}")
        # else:
        #     print (f"Point {i}: No edge detected with Laplacian norm = {laplacian_norm} within threshold {threshold}")

    # Create a new point cloud for edges
    edge_pcd = o3d.geometry.PointCloud()
    edge_pcd.points = o3d.utility.Vector3dVector(np.array(edges))
    
    return edge_pcd

def segment_edges_by_distance(pcd, distance_threshold=0.1):
    """
    Segment edges in a point cloud based on distance.
    
    Args:
        pcd (open3d.geometry.PointCloud): Input point cloud.
        distance_threshold (float): Distance threshold for segmentation.
        
    Returns:
        list: List of segmented point clouds.
    """
    # Convert point cloud to numpy array
    points = np.asarray(pcd.points)

    # Perform DBSCAN clustering
    clustering = DBSCAN(eps=distance_threshold, min_samples=2).fit(points)
    labels = clustering.labels_

    # Create segmented point clouds
    segmented_pcds = []
    unique_labels = set(labels)

    for label in unique_labels:
        if label == -1:  # Skip noise points
            continue
        segment_indices = np.where(labels == label)[0]
        segment_points = points[segment_indices]

        segment_pcd = o3d.geometry.PointCloud()
        segment_pcd.points = o3d.utility.Vector3dVector(segment_points)
        segmented_pcds.append(segment_pcd)

    return segmented_pcds

if __name__ == "__main__":

    args = edge_detection_args.get_args()
    input_file = args.input
    output_file = args.output
    base_radius = args.base_radius
    threshold = args.threshold
    visualize = args.visualize
    # Example usage:
    # Load the point cloud
    pcd = load_point_cloud(input_file)
    print(f"Loaded point cloud with {len(pcd.points)} points from {input_file}")

    
    # Compute edges
    edge_pcd = compute_edges_dynamic_radius(pcd, base_radius=base_radius, threshold=threshold)
    print(f"Computed edges with dynamic radius: {len(edge_pcd.points)} points")
    # Save the edge-detected point cloud
    save_point_cloud(edge_pcd, output_file)

    print(f"Saved edge-detected point cloud to {output_file}")
    # Segment edges based on distance
    segmented_pcds = segment_edges_by_distance(edge_pcd)
    print(f"Segmented edges into {len(segmented_pcds)} segments")
    # Save segmented point clouds
    for i, segment_pcd in enumerate(segmented_pcds):
        # output_name = output_file.split("/")[0]+"/"+output_file.split("/")[1:]
        output_name = os.path.splitext(output_file)[0]
        # Save each segment with a unique name
        segment_output_file = f"{output_name}_segment_{i}.ply"
        save_point_cloud(segment_pcd, segment_output_file)
        print(f"Saved segmented point cloud to {segment_output_file}")
    # Visualize the point cloud with edges highlighted
    if visualize:
        visualize_point_cloud(pcd, title="Original Point Cloud")
        visualize_point_cloud(edge_pcd, title="Edge-Detected Point Cloud")
    print(f"Edge-detected point cloud saved to {output_file}")


# This code is a simple implementation of edge detection in point clouds using the Laplacian operator.