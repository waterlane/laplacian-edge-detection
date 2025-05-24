import argparse

def add_args(parser):
    parser.add_argument("--input", type=str, required=True, help="Path to the input point cloud file (e.g., input.ply)")
    parser.add_argument("--output", type=str, required=True, help="Path to save the output edge-detected point cloud (e.g., edges.ply)")
    parser.add_argument("--smoothing", type=float, default=0.1, help="Smoothing factor for edge detection (default: 0.1)")
    parser.add_argument("--visualize", action="store_true", help="Visualize the point cloud with edges highlighted")
    
    return parser

def get_args():

    parser = argparse.ArgumentParser(description="Edge Detection in Point Clouds")
    parser = add_args(parser)
    args = parser.parse_args()
    
    return args