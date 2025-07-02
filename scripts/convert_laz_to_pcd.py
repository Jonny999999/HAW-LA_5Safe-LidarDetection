import os
import laspy
import numpy as np

# USAGE: manually adjust the paths in variables at the end of the file

def convert_laz_to_pcd(input_dir, output_dir):
    os.makedirs(output_dir, exist_ok=True)

    for filename in os.listdir(input_dir):
        if filename.lower().endswith(".laz"):
            laz_path = os.path.join(input_dir, filename)
            print(f"Converting: {laz_path}")

            # Load LAZ file
            las = laspy.read(laz_path)
            points = np.vstack((las.x, las.y, las.z)).transpose().astype(np.float32)

            # Prepare output file path
            base_name = os.path.splitext(filename)[0]
            pcd_path = os.path.join(output_dir, f"{base_name}.pcd")

            # Write to PCD (ASCII format)
            with open(pcd_path, 'w') as f:
                f.write("# .PCD v0.7 - Point Cloud Data file format\n")
                f.write("VERSION 0.7\n")
                f.write("FIELDS x y z\n")
                f.write("SIZE 4 4 4\n")
                f.write("TYPE F F F\n")
                f.write("COUNT 1 1 1\n")
                f.write(f"WIDTH {points.shape[0]}\n")
                f.write("HEIGHT 1\n")
                f.write("VIEWPOINT 0 0 0 1 0 0 0\n")
                f.write(f"POINTS {points.shape[0]}\n")
                f.write("DATA ascii\n")
                for x, y, z in points:
                    f.write(f"{x:.6f} {y:.6f} {z:.6f}\n")

            print(f"Saved: {pcd_path}")

if __name__ == "__main__":
    input_folder = "../data/2025.06.03_4ppl-walking-sitting_laz-frames"     # Folder containing .laz files
    output_folder = "../data/2025.06.03_4ppl-walking-sitting_conv_to_pcd"   # Folder to save .pcd files

    convert_laz_to_pcd(input_folder, output_folder)
