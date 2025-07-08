import os
import laspy
import numpy as np
import glob

import config

class LazFrameExporter:
    def __init__(self, output_dir="output/laz", skip_n_frames = 0, start_index=0):
        self.output_dir = output_dir
        self.frame_count = start_index
        self.skip_n_frames = skip_n_frames
        self.internal_pointcloud_counter = 0
        os.makedirs(self.output_dir, exist_ok=True)

        if config.FILE_EXPORT_ENABLE:
            self._clear_output_directory()


    def save_frame(self, pointcloud: np.ndarray):
        """
        Saves Pointcloud as .laz File

        Parameters:
            pointcloud: NumPy-Array with shape (N, 3), dtype float32/float64
        """
        if config.FILE_EXPORT_ENABLE:
            if self.internal_pointcloud_counter >= self.skip_n_frames:
                self.internal_pointcloud_counter = 0
                if pointcloud.shape[1] != 3:
                    raise ValueError("Pointcloud must be Nx3 shaped (x, y, z)")

                points = pointcloud.astype(np.float32)

                header = laspy.LasHeader(point_format=3, version="1.4")
                header.scales = [0.001, 0.001, 0.001]   # mm precision
                header.offsets = [0.0, 0.0, 0.0]

                las = laspy.LasData(header)
                las.x = points[:, 0]
                las.y = points[:, 1]
                las.z = points[:, 2]

                filename = os.path.join(self.output_dir, f"frame_{self.frame_count:04d}.laz")
                las.write(filename)
                
                self.frame_count += 1
            else:
                self.internal_pointcloud_counter += 1

    def _clear_output_directory(self):
        """
        clears output directory. is automatically calles in constructor when File Export is enabled in config
        """
        files = glob.glob(os.path.join(self.output_dir, '*'))
        for f in files:
            try:
                os.remove(f)
            except Exception as e:
                print(f"[WARN] Could not delete file {f}: {e}")