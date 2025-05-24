import open3d as o3d
import sys
import numpy as np

def read_ply_and_reconstruct_surface(ply_file_path):
    # 读取 PLY 文件
    mesh = o3d.io.read_triangle_mesh(ply_file_path)
    if not mesh.has_vertices():
        raise ValueError("The PLY file does not contain valid vertices.")
    
    # 表面重建 - 使用 Poisson 重建
    print("Performing Poisson surface reconstruction...")
    mesh.compute_vertex_normals()
    poisson_mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(
        mesh.sample_points_uniformly(number_of_points=100000), depth=9
    )
    
    # 根据密度过滤低质量的面
    print("Filtering low-density faces...")
    densities = np.asarray(densities)
    density_threshold = np.percentile(densities, 5)  # 过滤掉最低 5% 的密度
    vertices_to_remove = densities < density_threshold
    poisson_mesh.remove_vertices_by_mask(vertices_to_remove)
    
    # 面分割
    print("Performing mesh segmentation...")
    clusters, _ = poisson_mesh.cluster_connected_triangles()
    cluster_ids = np.asarray(clusters)
    max_cluster_id = cluster_ids.max()
    
    segmented_meshes = []
    for cluster_id in range(max_cluster_id + 1):
        indices = cluster_ids == cluster_id
        segmented_mesh = poisson_mesh.select_by_index(indices)
        segmented_meshes.append(segmented_mesh)
    
    return segmented_meshes

if __name__ == "__main__":

    if len(sys.argv) != 2:
        print("Usage: python surface_reconstruction.py <path_to_ply_file>")
        sys.exit(1)

    ply_file_path = sys.argv[1]
    try:
        segmented_meshes = read_ply_and_reconstruct_surface(ply_file_path)
        print(f"Surface reconstruction and segmentation completed. Segments: {len(segmented_meshes)}")
        
        # 保存分割后的网格
        for i, mesh in enumerate(segmented_meshes):
            output_path = f"segment_{i}.ply"
            o3d.io.write_triangle_mesh(output_path, mesh)
            print(f"Segment {i} saved to {output_path}")
    except Exception as e:
        print(f"Error: {e}")