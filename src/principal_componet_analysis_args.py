import argparse

def add_args(parser):
    parser.add_argument("--input", type=str, required=True, help="Path to the input point cloud file (e.g., input.ply)")
    parser.add_argument("--output", type=str,default = "./output/", help="Path to save the output PCA point cloud (e.g., pca_output.ply)")
    # parser.add_argument("--base_radius", type=float, default=0.05, help="Base radius for dynamic neighborhood search (default: 0.05)")
    # parser.add_argument("--visualize", action="store_true", help="Visualize the point cloud with PCA results highlighted")
    
    return parser

def get_args():
    parser = argparse.ArgumentParser(description="Principal Component Analysis in Point Clouds")
    parser = add_args(parser)
    args = parser.parse_args()
    
    return args
