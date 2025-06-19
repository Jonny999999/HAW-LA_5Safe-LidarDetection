import numpy as np
import open3d as o3d
from config import COUNT_PEOPLE_DRAW_BOXES
from utils import * # custom logging helpers
from collections import deque
from shapely.geometry import Polygon, Point
import sys
from visualizer import draw_2d_polygon



# ===============================
# === PEOPLE DETECTION (CLUSTERS)
# ===============================

# TODO: Drop this, function is legacy and no long used
# Globals (optionally refactor to avoid)
drawn_bounding_boxes = []
def estimate_moving_people(pointcloud_np, distance_threshold=0.5, min_points=30, visualizer=None):
    """
    Detects and optionally visualizes clusters of points likely to be moving people.

    Args:
        pointcloud_np (np.ndarray): N x 3 array of points
        distance_threshold (float): DBSCAN radius (meters)
        min_points (int): Minimum points to consider a cluster
        visualizer (Visualizer): Optional Open3D visualizer for bounding boxes

    Returns:
        int: Estimated number of people
    """
    global drawn_bounding_boxes


    MAX_INPUT_POINTS = 5000  # Prevent performance issues when input is too large
    if pointcloud_np.shape[0] > MAX_INPUT_POINTS:
        log_warn(f"Too many points ({pointcloud_np.shape[0]}), skipping people detection.")
        return 0

    if pointcloud_np.shape[0] == 0:
        log_warn("No dynamic points to analyze.")
        return 0

    # Convert to Open3D point cloud
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(pointcloud_np)

    # Run DBSCAN clustering to identify dense regions
    labels = np.array(pcd.cluster_dbscan(eps=distance_threshold, min_points=min_points, print_progress=False))

    # Handle empty/noise result
    if labels.size == 0 or np.max(labels) < 0:
        log_warn("No clusters found.")
        return 0

    unique_labels, counts = np.unique(labels, return_counts=True)
    valid_clusters = 0

    if COUNT_PEOPLE_DRAW_BOXES and visualizer is not None:
        # Remove previous bounding boxes from scene
        for box in drawn_bounding_boxes:
            visualizer.remove_geometry(box, reset_bounding_box=False)
        drawn_bounding_boxes.clear()

        # Iterate through valid clusters and draw bounding boxes
        for cluster_id, count in zip(unique_labels, counts):
            if cluster_id == -1 or count < min_points:
                continue  # Skip noise or too-small clusters

            valid_clusters += 1

            # Extract cluster points and compute its bounding box
            indices = np.where(labels == cluster_id)[0]
            cluster_pcd = pcd.select_by_index(indices)
            bbox = cluster_pcd.get_axis_aligned_bounding_box()
            bbox.color = (0.0, 0.0, 0.5)  # Dark blue color for bounding boxes

            # Add to visualizer and store for later removal
            visualizer.add_geometry(bbox, reset_bounding_box=False)
            drawn_bounding_boxes.append(bbox)

    log_info(f"people_detection: Estimated number of moving people: {valid_clusters}")
    return valid_clusters















def track_moving_clusters(
    pointcloud_np,
    distance_threshold=0.4, # points within that radius are merged as one cluster
    min_points=50, # min points in a cluster to detect as person
    max_moving_points_ignore_frame=4000, # ignore entire frame if e.g. sensor moved
    min_z_height=0.6, # initial detection threshold
    min_volume_m3=0.5, # initial detection threshold
    min_frames_to_confirm=10, # frame count the initial thresholds have to be fulfilled to be added as cluster
    status_cache=None,
    retain_frames=60, # at 5fps
    match_threshold=1.0
):
    """
    Detects and persistently tracks moving clusters using DBSCAN and centroid matching.

    Unconfirmed clusters must pass filters (volume, height) and appear for at least N frames
    before becoming "confirmed". Confirmed clusters are easier to track and retained longer
    even if movement is minimal or their shape shrinks.

    Returns:
        List of cluster dicts with id, centroid, point cloud, and status.
    """
    import numpy as np
    import open3d as o3d

    # === Init persistent state across frames ===
    if not hasattr(track_moving_clusters, "last_clusters"):
        track_moving_clusters.last_clusters = {}
        track_moving_clusters.cluster_id_counter = 0

    new_clusters = []

    # === Run DBSCAN if points exist and under max limit ===
    if pointcloud_np.shape[0] == 0:
        log_info("[detect clusters] No points received.")
    elif pointcloud_np.shape[0] > max_moving_points_ignore_frame:
        log_warn(f"[detect clusters] Too many points ({pointcloud_np.shape[0]}), skipping frame.")
    else:
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(pointcloud_np)
        labels = np.array(pcd.cluster_dbscan(eps=distance_threshold, min_points=min_points, print_progress=False))

        if labels.size > 0 and np.max(labels) >= 0:
            unique_labels = np.unique(labels)
            for label in unique_labels:
                if label == -1:
                    continue
                indices = np.where(labels == label)[0]
                cluster = pcd.select_by_index(indices)
                points = np.asarray(cluster.points)
                centroid = np.mean(points, axis=0)
                bbox = cluster.get_axis_aligned_bounding_box()
                volume = bbox.volume()
                z_range = np.ptp(points[:, 2])

                new_clusters.append({
                    "centroid": centroid,
                    "pcd": cluster,
                    "bbox": bbox,
                    "volume": volume,
                    "z_range": z_range
                })
        else:
            log_info("[detect clusters] DBSCAN found no clusters.")

    # === Matching and cluster tracking ===
    results = []
    updated_cluster_state = {}
    used_prev_ids = set()
    reused_id_count = 0
    new_id_count = 0
    expired_ids = []

    for cluster in new_clusters:
        curr_centroid = np.array(cluster["centroid"])
        best_id = None
        best_dist = float("inf")

        for prev_id, prev_data in track_moving_clusters.last_clusters.items():
            if prev_id in used_prev_ids:
                continue
            dist = np.linalg.norm(curr_centroid - prev_data["centroid"])
            if dist < match_threshold and dist < best_dist:
                best_id = prev_id
                best_dist = dist

        if best_id is not None:
            reused_id_count += 1
            used_prev_ids.add(best_id)
            prev_data = track_moving_clusters.last_clusters[best_id]
            frames_observed = prev_data.get("frames_observed", 0) + 1
            was_confirmed = prev_data.get("confirmed", False)

            if not was_confirmed:
                passes_volume = cluster["volume"] >= min_volume_m3
                passes_height = cluster["z_range"] >= min_z_height
                if not passes_volume:
                    log_debug(f"[cluster {best_id}] Volume too small: {cluster['volume']:.4f} m³")
                if not passes_height:
                    log_debug(f"[cluster {best_id}] Height too small: {cluster['z_range']:.4f} m")
                confirmed = passes_volume and passes_height and frames_observed >= min_frames_to_confirm
            else:
                confirmed = True
                if cluster["bbox"].volume() < prev_data["volume"] * 0.5:
                    cluster["bbox"] = prev_data["bbox"]
                    cluster["pcd"] = prev_data["pcd"]

            updated_cluster_state[best_id] = {
                "centroid": curr_centroid,
                "pcd": cluster["pcd"],
                "bbox": cluster["bbox"],
                "frames_since_seen": 0,
                "frames_observed": frames_observed,
                "confirmed": confirmed,
                "volume": cluster["volume"],
                "z_range": cluster["z_range"]
            }

            if confirmed:
                results.append({
                    "id": best_id,
                    "centroid": curr_centroid,
                    "pcd": cluster["pcd"],
                    "status": "confirmed"
                })
                if not was_confirmed:
                    log_warn(f"[detect clusters] Cluster CONFIRMED id={best_id}")
        else:
            new_id = track_moving_clusters.cluster_id_counter
            track_moving_clusters.cluster_id_counter += 1
            new_id_count += 1

            updated_cluster_state[new_id] = {
                "centroid": cluster["centroid"],
                "pcd": cluster["pcd"],
                "bbox": cluster["bbox"],
                "frames_since_seen": 0,
                "frames_observed": 1,
                "confirmed": False,
                "volume": cluster["volume"],
                "z_range": cluster["z_range"]
            }

    # === Handle unmatched previous clusters ===
    for prev_id, prev_data in track_moving_clusters.last_clusters.items():
        if prev_id not in used_prev_ids:
            prev_data["frames_since_seen"] += 1
            if prev_data["frames_since_seen"] < retain_frames:
                updated_cluster_state[prev_id] = prev_data
                if prev_data.get("confirmed", False):
                    results.append({
                        "id": prev_id,
                        "centroid": prev_data["centroid"],
                        "pcd": prev_data["pcd"],
                        "status": "retained"
                    })
            else:
                expired_ids.append(prev_id)
                log_warn(f"[detect clusters] Cluster DROPPED id={prev_id}")

    # === Finalize and update ===
    track_moving_clusters.last_clusters = updated_cluster_state
    retained_count = len([r for r in results if r["status"] == "retained"])
    log_info(f"[detect clusters] Matched: {reused_id_count}, New: {new_id_count}, Retained: {retained_count}, Expired: {len(expired_ids)}, Total: {len(results)}")

    if status_cache:
        status_cache.update_status_key("DETECTION_TRACKED_PEOPLE_INSIDE", str(len(results)))
        status_cache.update_status_key("DETECTION_MOVING_PEOPLE", str(reused_id_count + new_id_count))

    return results






# ========== ROOM ENTRY/EXIT TRACKING ==========
people_inside_incremented = 0

def track_room_occupancy(clusters, polygon_xy_inside_area, history_buffer=None, status_cache=None, visualizer=None):
    """
    Tracks people entering or leaving a room via a virtual door polygon.
    Includes hysteresis using an inner polygon to avoid flickering from noisy movement or jittery cluster centroids.

    Entry condition: moved from outside outer polygon → inside inner polygon.
    Exit condition: moved from inside inner polygon → outside outer polygon.

    Args:
        clusters (List of (id, centroid, cluster_pcd)): Detected moving clusters
        polygon_xy_inside_area (list of (x,y)): Outer polygon describing the entry zone (room boundary)
        history_buffer (deque): Optional persistent buffer for tracking movement history
        status_cache: Optional status interface (for logging or dashboard)
        visualizer: Optional Open3D visualizer for drawing the entry polygon and debug info

    Returns:
        int: Estimated current number of people inside
    """

    ENTRY_EXIT_HYSTERESIS_METERS = 0.3
    global people_inside_incremented

    # Init persistent buffer using function attribute
    if not hasattr(track_room_occupancy, "history_buffer"):
        from collections import deque
        track_room_occupancy.history_buffer = deque(maxlen=5)
    history_buffer = track_room_occupancy.history_buffer

    # Convert outer polygon to shapely format
    outer_poly = Polygon(polygon_xy_inside_area)

    # Automatically create smaller "inner" polygon for hysteresis
    try:
        inner_poly = outer_poly.buffer(-ENTRY_EXIT_HYSTERESIS_METERS)
        if inner_poly.is_empty or not inner_poly.is_valid:
            log_warn("[track_room_occupancy] Hysteresis inner polygon is invalid or empty. Skipping hysteresis.")
            inner_poly = outer_poly
    except Exception as e:
        log_error(f"[track_room_occupancy] Failed to create inner hysteresis polygon: {e}")
        inner_poly = outer_poly

    # Optional: draw both polygons for visualization
    if visualizer is not None:
        draw_2d_polygon(polygon_xy_inside_area, visualizer, color=(1.0, 0.6, 0.0))  # orange
        if inner_poly != outer_poly:
            draw_2d_polygon(list(inner_poly.exterior.coords), visualizer, color=(0.2, 0.8, 0.2))  # green

    # Sort cluster tuples into dictionary by ID
    #cluster_dict = {cid: centroid for (cid, centroid, _) in clusters}
    cluster_dict = {c["id"]: c["centroid"] for c in clusters}
    log_debug(f"[track_room_occupancy] Current cluster IDs: {list(cluster_dict.keys())}")

    # Append this frame's centroids to history buffer (as dict of cid → position)
    history_buffer.append(cluster_dict)

    if len(history_buffer) < 2:
        log_warn("[track_room_occupancy] Not enough frames in history yet")
        return max(people_inside_incremented, 0)

    # Init persistent sets for tracking entered/exited cluster IDs
    if not hasattr(track_room_occupancy, "entered_ids"):
        track_room_occupancy.entered_ids = set()
        track_room_occupancy.exited_ids = set()

    # Match tracked IDs over time (only process IDs present in both first and last frame)
    ids_in_both = set(history_buffer[0].keys()).intersection(history_buffer[-1].keys())

    log_debug(f"[track_room_occupancy] IDs present in both first and last frame: {list(ids_in_both)}")

    for cid in ids_in_both:
        start = history_buffer[0][cid][:2]  # (x,y) in first frame
        end = history_buffer[-1][cid][:2]   # (x,y) in current frame

        was_inside_outer = outer_poly.contains(Point(start))
        was_inside_inner = inner_poly.contains(Point(start))
        is_inside_outer = outer_poly.contains(Point(end))
        is_inside_inner = inner_poly.contains(Point(end))

        movement_vector = np.array(end) - np.array(start)
        moved_distance = np.linalg.norm(movement_vector)

        log_debug(f"[track_room_occupancy] Cluster {cid} moved {moved_distance:.2f} m: {start} → {end}")
        log_debug(f"[track_room_occupancy] was_inside_outer={was_inside_outer}, was_inside_inner={was_inside_inner}, is_inside_outer={is_inside_outer}, is_inside_inner={is_inside_inner}")

        if moved_distance < 0.05:
            log_info(f"[track_room_occupancy] Cluster {cid} skipped due to low movement")
            continue  # noise or static

        # Entry: outside → inside
        if not was_inside_outer and is_inside_inner and cid not in track_room_occupancy.entered_ids:
            log_warn(f"[track_room_occupancy] Cluster {cid} ENTERED")
            sys.stdout.write('\a')
            sys.stdout.flush()
            if status_cache:
                status_cache.add_log_entry("DETECTION_LAST_EVENTS", f"Cluster {cid} ENTERED", trigger_file_update=False)
            track_room_occupancy.entered_ids.add(cid)
            track_room_occupancy.exited_ids.discard(cid)
            people_inside_incremented += 1

        # Exit: inside → outside
        elif was_inside_inner and not is_inside_outer and cid not in track_room_occupancy.exited_ids:
            log_warn(f"[track_room_occupancy] Cluster {cid} LEFT")
            if status_cache:
                status_cache.add_log_entry("DETECTION_LAST_EVENTS", f"Cluster {cid} LEFT", trigger_file_update=False)
            track_room_occupancy.exited_ids.add(cid)
            track_room_occupancy.entered_ids.discard(cid)
            people_inside_incremented -= 1

        # Clamp to non-negative
        if people_inside_incremented < 0:
            log_error("[track_room_occupancy] people_inside went negative — clamping to 0")
            people_inside_incremented = 0

    log_debug(f"[track_room_occupancy] People inside (tracked): {people_inside_incremented}")

    if status_cache:
        status_cache.update_status_key("DETECTION_COUNTED_PEOPLE_INSIDE", str(people_inside_incremented))

    return people_inside_incremented
