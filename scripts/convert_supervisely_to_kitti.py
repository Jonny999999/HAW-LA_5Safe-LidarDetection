import os
import json
import numpy as np
from pathlib import Path
from pyntcloud import PyntCloud
from PIL import Image
import pandas as pd

DEFAULT_OBJECT_CLASS = "Pedestrian"


def convert_supervisely_to_kitti_aabb(json_dir, pcd_dir, output_dir, class_name_map=None, default_class="Pedestrian"):
    json_dir = Path(json_dir)
    pcd_dir = Path(pcd_dir)
    output_dir = Path(output_dir)

    label_dir = output_dir / "label_2"
    velodyne_dir = output_dir / "velodyne"
    image_dir = output_dir / "image_2"
    calib_dir = output_dir / "calib"

    for d in [label_dir, velodyne_dir, image_dir, calib_dir]:
        d.mkdir(parents=True, exist_ok=True)

    frames = sorted(json_dir.glob("*.json"))
    stats = []

    print(f"🟡 Found {len(frames)} JSON frames in {json_dir}")

    for frame in frames:
        frame_id = frame.stem
        json_path = json_dir / f"{frame_id}.json"
        pcd_path = pcd_dir / f"{frame_id}.pcd"
        label_path = label_dir / f"{frame_id}.txt"
        bin_path = velodyne_dir / f"{frame_id}.bin"
        img_path = image_dir / f"{frame_id}.png"
        calib_path = calib_dir / f"{frame_id}.txt"

        # === Step 1: Load PCD
        try:
            cloud = PyntCloud.from_file(str(pcd_path))
            print(f"→ [{frame_id}] Loaded PCD: {cloud.points.shape}")
        except Exception as e:
            print(f"❌ [{frame_id}] Failed to load PCD: {e}")
            continue

        if not all(k in cloud.points for k in ['x', 'y', 'z']):
            print(f"❌ [{frame_id}] PCD missing required fields (x,y,z)")
            continue

        xyz = cloud.points[['x', 'y', 'z']].values.astype(np.float32)
        num_points = xyz.shape[0]
        xyz_pad = np.pad(xyz, ((0, 0), (0, 1)), constant_values=0)  # Add intensity=0
        xyz_pad.tofile(bin_path)

        # === Step 2: Load JSON
        try:
            with open(json_path, "r") as f:
                data = json.load(f)
        except Exception as e:
            print(f"❌ [{frame_id}] Failed to load JSON: {e}")
            continue

        objects = {obj["key"]: obj for obj in data.get("objects", [])}
        figures = data.get("figures", [])

        print(f"   → {len(figures)} figures, {len(objects)} objects")

        lines = []
        valid_count = 0
        skipped_count = 0

        for fig in figures:
            obj_key = fig.get("objectKey")
            indices = fig.get("geometry", {}).get("indices", [])
            if not indices:
                skipped_count += 1
                continue

            indices = np.array(indices)
            if np.any(indices >= num_points):
                print(f"⚠️  [{frame_id}] Skipping object: indices exceed point cloud size")
                skipped_count += 1
                continue

            #label = objects.get(obj_key, {}).get("classTitle", default_class)
            # we only detect one type (person) ignore supervisely tags
            global DEFAULT_OBJECT_CLASS
            label = DEFAULT_OBJECT_CLASS
            if class_name_map:
                label = class_name_map.get(label, label)

            points = xyz[indices]
            min_xyz = points.min(axis=0)
            max_xyz = points.max(axis=0)
            center = (min_xyz + max_xyz) / 2
            dims = max_xyz - min_xyz

            x, y, z = center
            dx, dy, dz = dims  # l, h, w
            ry = 0.0  # assume no rotation

            line = f"{label} 0 0 -1 0 0 0 0 {dy:.2f} {dz:.2f} {dx:.2f} {x:.2f} {y:.2f} {z:.2f} {ry:.2f}"
            lines.append(line)
            valid_count += 1

        with open(label_path, "w") as f:
            f.write("\n".join(lines))

        # Dummy image and calib
        Image.new("RGB", (1, 1)).save(img_path)
        with open(calib_path, "w") as f:
            f.write("P0: 1 0 0 0\n" * 4)
            f.write("Tr_velo_to_cam: 1 0 0 0\n")
            f.write("R0_rect: 1 0 0\n0 1 0\n0 0 1\n")

        stats.append((frame_id, len(figures), valid_count, skipped_count))

    df = pd.DataFrame(stats, columns=["frame", "total_figures", "valid", "skipped"])
    print("\n=== Summary ===")
    print(df.to_string(index=False))
    print(f"\n✅ Converted {len(frames)} frames to {output_dir}")

# Run the function:
convert_supervisely_to_kitti_aabb(
    json_dir="/home/mablee/ai-training/model-training_mmdet3d/supervisely_raw/json",
    pcd_dir="/home/mablee/ai-training/model-training_mmdet3d/supervisely_raw/pcd",
    output_dir="/home/mablee/ai-training/model-training_mmdet3d/data/kitti/training"
)
