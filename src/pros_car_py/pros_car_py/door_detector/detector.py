# >> Basic package
import cv2
import numpy as np
import random
import math
from sklearn.cluster import DBSCAN
from concurrent.futures import ThreadPoolExecutor
import time

def hsv_to_circular_cartesian(hsv):
    h, s, v = hsv[:, 0], hsv[:, 1] / 255.0, hsv[:, 2] / 255.0
    h_rad = h / 180.0 * np.pi
    x = np.cos(h_rad) * s
    y = np.sin(h_rad) * s
    z = v
    return np.stack([x, y, z], axis=1)

def cal_door_len(door_groups):
    total_len = 0
    for group in door_groups:
        for edge in group["edges"]:
            x0, y0 = edge["start"]
            x1, y1 = edge["end"]
            length = np.hypot(x1 - x0, y1 - y0)
            total_len += length
    return total_len

def is_between_groups(door_center, pillar1_center, pillar2_center, margin=50):
    """
    檢查一條門 edge 是否在兩個柱子 edge group 中間。
    """
    x1 = min(pillar1_center[0], pillar2_center[0])
    x2 = max(pillar1_center[0], pillar2_center[0])
    return x1 + margin < door_center[0] < x2 - margin

def angle_between(p1, p2):
    dx, dy = p2[0] - p1[0], p2[1] - p1[1]
    return np.arctan2(dy, dx) 

def point_to_line_distance(p, a, b):
    a, b, p = np.array(a), np.array(b), np.array(p)
    if np.allclose(a, b):
        return np.linalg.norm(p - a)
    return np.abs(np.cross(b - a, p - a)) / np.linalg.norm(b - a)

def group_parallel_lines(edges, spatial_eps=20, angle_eps=np.radians(10), min_samples=3):
    if not edges:
        return []
    
    lines = list(edges.items())
    features = []

    for edge_id, line in lines:
        x0, y0 = line["start"]
        x1, y1 = line["end"]
        midx = (x0 + x1) / 2
        midy = (y0 + y1) / 2
        angle = angle_between(line["start"], line["end"])
        features.append([midx, midy, math.cos(angle), math.sin(angle)])
        
    X = np.array(features)
    spatial_scale = 1.0 / spatial_eps
    angle_scale = 1.0 / angle_eps

    X[:, 0:2] *= spatial_scale
    X[:, 2:4] *= angle_scale

    clustering = DBSCAN(eps=3.0, min_samples=min_samples).fit(X)
    labels = clustering.labels_
    
    groups = []
    for label in set(labels):
        if label == -1:
            continue
        group = {
            "edges": [],
            "edge_ids": []
        }
        for idx, line_label in enumerate(labels):
            if line_label == label:
                edge_id, edge = lines[idx]
                group["edges"].append(edge)
                group["edge_ids"].append(edge_id)
        groups.append(group)

    return groups
def lines_can_connect(groups, max_gap=100):
    unconnectable_groups = []
    for i in range(len(groups)):
        connectable = False
        for j in range(len(groups)):
            if i == j:
                continue
            if lines_connect_test(groups[i]["edges"], groups[j]["edges"], max_gap):
                connectable = True
                break
        if not connectable:
            unconnectable_groups.append(groups[i])
    return unconnectable_groups, len(unconnectable_groups) == 0

def lines_connect_test(g1, g2, max_gap=80):
    g1_points = [g["start"] for g in g1] + [g["end"] for g in g1]
    g2_points = [g["start"] for g in g2] + [g["end"] for g in g2]

    min_dist = min(
        np.linalg.norm(np.array(p1) - np.array(p2))
        for p1 in g1_points for p2 in g2_points
    )
    print(f"{min_dist} < {max_gap}:{min_dist < max_gap}")
    return min_dist < max_gap

def get_further_edge_group(groups):
    def avg_y(group):
        return np.mean([pt[1] for g in group["edges"] for pt in [g["start"], g["end"]]])
    if len(groups) > 0:
        return min(groups, key=avg_y)
    else:
        return None

def get_avg_color(img, x, y, dx, dy, side=1, size=3):
    h, w, _ = img.shape
    half = size // 2
    values = []

    for i in range(-half, half+1):
        sx = x + dx * side + dy * i
        sy = y + dy * side + dx * i
        if 0 <= sx < w and 0 <= sy < h:
            values.append(img[sy, sx].astype(int))

    if values:
        return np.mean(values, axis=0)
    return None

def get_avg_color_fast(image, x, y, nx, ny, side=30, patch_size=5):
    px = int(round(x + nx * side))
    py = int(round(y + ny * side))
    h, w = image.shape[:2]
    half = patch_size // 2

    if px - half < 0 or px + half >= w or py - half < 0 or py + half >= h:
        return None

    patch = image[py - half:py + half + 1, px - half:px + half + 1]
    return patch.mean(axis=(0, 1))

def draw_clusters(image, all_grouped_edges_dict):
    vis = image.copy()
    base_colors = {
        "door_edge": (255, 0, 0),
        "pillar_edge": (0, 255, 0),
        "wall_edge": (0, 0, 255),
    }

    for key, groups in all_grouped_edges_dict.items():
        base_color = base_colors.get(key, (200, 200, 200))
        for cluster_idx, group in enumerate(groups):
            color = tuple(min(255, c + random.randint(0, 80)) for c in base_color)
            for edge in group["edges"]:
                x0, y0 = map(int, edge["start"])
                x1, y1 = map(int, edge["end"])
                cv2.line(vis, (x0, y0), (x1, y1), color, 2)
                cx = int((x0 + x1) / 2)
                cy = int((y0 + y1) / 2)
                cv2.putText(vis, f"{key[0]}{cluster_idx}", (cx, cy), cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)

    return vis

def process_patch_with_coords(image, x0, y0, x1, y1):
    patch = image[y0:y1, x0:x1]
    gray = cv2.cvtColor(patch, cv2.COLOR_BGR2GRAY)
    edges = cv2.Canny(gray, 50, 150)
    dx = cv2.Sobel(gray, cv2.CV_64F, 1, 0)
    dy = cv2.Sobel(gray, cv2.CV_64F, 0, 1)

    return {
        "region": (x0, y0, x1, y1),
        "edges": edges,
        "dx": dx,
        "dy": dy
    }

def get_weighted_avg_hsv_patch(hsv_image, x, y, nx, ny, side=30, patch_size=5):
    px = int(round(x + nx * side))
    py = int(round(y + ny * side))
    h, w = hsv_image.shape[:2]
    half = patch_size // 2

    if px - half < 0 or px + half >= w or py - half < 0 or py + half >= h:
        return None

    patch = hsv_image[py - half:py + half + 1, px - half:px + half + 1]

    ys, xs = np.mgrid[-half:half+1, -half:half+1]
    dist_squared = xs**2 + ys**2
    sigma = patch_size / 2.0
    weights = np.exp(-dist_squared / (2 * sigma**2))
    weights = weights / np.sum(weights) 

    weighted_h = np.sum(patch[:, :, 0] * weights)
    weighted_s = np.sum(patch[:, :, 1] * weights)
    weighted_v = np.sum(patch[:, :, 2] * weights)

    return np.array([weighted_h, weighted_s, weighted_v])

class DoorDetector:
    def __init__(self):
        self.cluster_centers = np.load("/workspaces/src/pros_car_py/kmeans_models/kmeans_centers.npy")
        # print(self.cluster_centers)
        self.label_map = {
            0: "pillar",
            1: "wall",
            2: "floor",
            3: "door",
            4: "other",
        }
        self.executor = ThreadPoolExecutor(max_workers=8) 
    def __del__(self):
        self.executor.shutdown()
        
    def scan(self, image, detect_target=None, fast_check=False):
        if fast_check:  
            return self.fast_object_check(image, detect_target)
        
        edges, self.edge_color_image = self.process_image(image)
        
        results = {}

        for key in ["door_edge", "pillar_edge", "wall_edge"]:
            if detect_target is None or key[:-5] in detect_target:
                results[key] = edges[key]

        results["have_pillar_edge"] = len(edges["pillar_edge"]) > 0
        results["have_wall_edge"] = len(edges["wall_edge"]) > 0
        results["have_door_edge"] = len(edges["door_edge"]) > 0
        results["have_floor_edge"] = len(edges["pillar_edge"]) > 0 or len(edges["wall_edge"]) > 0 or len(edges["door_edge"]) > 0

        return results
    
    def classify_color(self, color):
        diffs = self.cluster_centers - color  # shape: (K, 3)
        dists = np.sum(diffs ** 2, axis=1)    # faster than np.linalg.norm
        return int(np.argmin(dists))
    
    def classify_avg_color(self, image, x, y, nx, ny, side):
        bgr = get_avg_color_fast(image, x, y, nx, ny, side=side)
        if bgr is None:
            return None
        hsv = cv2.cvtColor(np.uint8([[bgr]]), cv2.COLOR_BGR2HSV)[0][0]
        label = self.label_map[self.classify_color(hsv)]
        return label
    
    def classify_avg_hsv(self, hsv, x, y, nx, ny, side):
        hsv = get_avg_color_fast(hsv, x, y, nx, ny, side=side)
        if hsv is None:
            return None
        
        #hsv = cv2.cvtColor(np.uint8([[bgr]]), cv2.COLOR_BGR2HSV)[0][0]
        label = self.label_map[self.classify_color(hsv)]
        return label

    def classify_whole_image_by_ratio(self, image, threshold=0.8):

        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        h, w, _ = hsv.shape
        reshaped = hsv.reshape(-1, 3)

        distances = np.linalg.norm(self.cluster_centers - reshaped[:, None], axis=2)
        pred_indices = np.argmin(distances, axis=1)

        unique, counts = np.unique(pred_indices, return_counts=True)
        total = h * w

        label_ratios = {self.label_map[int(k)]: v / total for k, v in zip(unique, counts)}

        for label, ratio in label_ratios.items():
            # print(f"{label}:{ratio}")
            if ratio >= threshold:
                return True, label, ratio 
        return False, None, label_ratios
    
    def slice_and_process_parallel(self, image, patch_size=200, y_start=100, max_workers=4):
        h, w = image.shape[:2]
        tasks = []
        for y0 in range(y_start, h, patch_size//2):
            y1 = min(y0 + patch_size, h)
            for x0 in range(0, w, patch_size//2):
                x1 = min(x0 + patch_size, w)
                tasks.append((x0, y0, x1, y1))

        return list(self.executor.map(lambda args: process_patch_with_coords(image, *args), tasks))
    
    def fast_object_check(self, image, detect_target, threshold=0.01):
        if detect_target is None:
            detect_target = ["floor", "door", "pillar", "wall"]

        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        h, w, _ = hsv.shape

        masked_hsv = hsv[100:, :, :] 
        reshaped = masked_hsv.reshape(-1, 3)

        dists = np.linalg.norm(self.cluster_centers - reshaped[:, None], axis=2)
        pred_indices = np.argmin(dists, axis=1)

        unique, counts = np.unique(pred_indices, return_counts=True)
        total_pixels = reshaped.shape[0]
        label_ratios = {self.label_map[int(k)]: v / total_pixels for k, v in zip(unique, counts)}

        result = {}
        for label in detect_target:
            ratio = label_ratios.get(label, 0.0)
            result[f"have_{label}_edge"] = ratio >= threshold

        return result

    def process_image(self, image):
        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        edge_color_image = image.copy()
        patches = self.slice_and_process_parallel(image, y_start=60)

        results = {
            "door_edge": {},
            "pillar_edge": {},
            "wall_edge": {},
        }
        all_points = []
        for patch in patches:
            x0, y0, _, _ = patch["region"]
            edges = patch["edges"]
            dx = patch["dx"]
            dy = patch["dy"]

            ys, xs = np.where(edges == 255)
            xs = xs[::4]
            ys = ys[::4]
            for x, y in zip(xs, ys):
                gx = dx[y, x]
                gy = dy[y, x]

                angle = np.abs(np.arctan2(gy, gx) * 180.0 / np.pi)
                if angle < 20 or angle > 100:
                    continue

                norm = np.sqrt(gx ** 2 + gy ** 2)
                if norm == 0:
                    continue

                nx = int(round(gx / norm))
                ny = int(round(gy / norm))

                global_x = x0 + x
                global_y = y0 + y
                all_points.append((global_x, global_y, nx, ny))
        def process_point(idx, x, y, nx, ny):
            label1 = self.classify_avg_hsv(hsv_image, x, y, nx, ny, side=30)
            label2 = self.classify_avg_hsv(hsv_image, x, y, nx, ny, side=-30)
            labels = [label1, label2]

            if "floor" in labels:
                edge_id = f"edge_{idx}"
                edge_color_image[y, x] = (255, 0, 255)
                if "door" in labels:
                    edge_color_image[y, x] = (255, 0, 0)
                    return ("door_edge", edge_id, x, y, nx, ny)
                elif "wall" in labels:
                    edge_color_image[y, x] = (0, 0, 255)
                    return ("wall_edge", edge_id, x, y, nx, ny)
                elif "pillar" in labels:
                    edge_color_image[y, x] = (0, 255, 0)
                    return ("pillar_edge", edge_id, x, y, nx, ny)
            return None
        futures = [
            self.executor.submit(process_point, idx, x, y, nx, ny)
            for idx, (x, y, nx, ny) in enumerate(all_points)
        ]
        for future in futures:
            result = future.result()
            if result:
                key, edge_id, x, y, nx, ny = result
                results[key][edge_id] = {"start": (x - nx, y - ny), "end": (x + nx, y + ny)}
        return results, edge_color_image
    
    def detect_pillar_wall_pair(self, image):
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        edges = cv2.Canny(gray, 50, 150)
        dx = cv2.Sobel(gray, cv2.CV_64F, 1, 0)
        dy = cv2.Sobel(gray, cv2.CV_64F, 0, 1)

        edge_color_image = image.copy()
        ys, xs = np.where(edges == 255)

        pillar_lower = np.array([0, 0, 200])
        pillar_upper = np.array([180, 15, 255])

        all_points = [(x, y) for x, y in zip(xs, ys) if y > 100]
        def process_point(x, y):
            gx = dx[y, x]
            gy = dy[y, x]
            norm = np.hypot(gx, gy)
            if norm == 0:
                return None

            angle = np.degrees(np.abs(np.arctan2(gy, gx)))
            if not(angle < 30 or angle > 150):
                return None

            nx = int(round(gx / norm))
            ny = int(round(gy / norm))


            hsv1 = get_weighted_avg_hsv_patch(hsv, x, y, nx, ny, side=30)
            hsv2 = get_weighted_avg_hsv_patch(hsv, x, y, nx, ny, side=-30)

            if hsv1 is None or hsv2 is None:
                return None
            
            is_pillar1 = np.all(pillar_lower <= hsv1) and np.all(hsv1 <= pillar_upper)
            is_pillar2 = np.all(pillar_lower <= hsv2) and np.all(hsv2 <= pillar_upper)

            label1 = self.label_map[self.classify_color(hsv1)] if not is_pillar1 else "pillar"
            label2 = self.label_map[self.classify_color(hsv2)] if not is_pillar2 else "pillar"
            cv2.circle(edge_color_image, (x,y), 3, (255,0,0), 1)
            if {label1, label2} == {"pillar", "wall"}:
                return (True, (x, y))
            return None

        futures = [
            self.executor.submit(process_point, x, y)
            for x, y in all_points
        ]
        count = 0
        for future in futures:
            res = future.result()
            if res:
                self.edge_color_image = edge_color_image
                return True, res[1]

        return False, None
    
    def get_group_center(self, edges):
        xs = [e["start"][0] + e["end"][0] for e in edges]
        ys = [e["start"][1] + e["end"][1] for e in edges]
        if len(xs) == 0:
            return None
        return int(sum(xs) / (2 * len(xs))), int(sum(ys) / (2 * len(ys)))
    
    def segment_image(self, image, visual=True):
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        h_img, w_img, _ = hsv.shape
        reshaped_hsv = hsv.reshape(-1, 3)

        # 1. Convert HSV to circular Cartesian coordinates
        cartesian_pixels = hsv_to_circular_cartesian(reshaped_hsv)

        # 2. Compute distances to cluster centers (already in cartesian space)
        dists = np.linalg.norm(cartesian_pixels[:, None] - self.cluster_centers, axis=2)
        min_dists = np.min(dists, axis=1)
        pred_indices = np.argmin(dists, axis=1)

        # # 3. Optional: mask out bad matches (too far from any cluster center)
        threshold = 1.0  
        pred_indices[min_dists > threshold] = -1
        pred_labels = pred_indices.reshape(h_img, w_img)

        if not visual:
            return pred_labels  
        
        # 4. Create segmentation visualization
        color_map = {
            "floor": (255, 0, 255),
            "wall": (255, 0, 0),
            "pillar": (0, 255, 0),
            "door": (0, 0, 255),
            "other": (100, 100, 100),
            # "unknown": (30, 30, 30),
        }

        seg_img = np.zeros((h_img, w_img, 3), dtype=np.uint8)
        for label_idx, name in self.label_map.items():
            seg_img[pred_labels == label_idx] = color_map.get(name, (0, 0, 0))

        # # 5. Optional: override with HSV fixed threshold for pillars (boost recall)
        # pillar_lower = np.array([0, 0, 200])
        # pillar_upper = np.array([180, 15, 255])
        # pillar_mask = cv2.inRange(hsv, pillar_lower, pillar_upper)
        # seg_img[pillar_mask > 0] = color_map["pillar"]

        return seg_img
    def get_class_bounding_boxes(self, image, target_class="wall", min_area=100):
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        h, w, _ = hsv.shape
        reshaped = hsv.reshape(-1, 3)

        dists = np.linalg.norm(self.cluster_centers - reshaped[:, None], axis=2)
        pred_indices = np.argmin(dists, axis=1)
        pred_labels = pred_indices.reshape(h, w)

        target_label_idx = None
        for idx, name in self.label_map.items():
            if name == target_class:
                target_label_idx = idx
                break

        if target_label_idx is None:
            print(f"[Error] Class '{target_class}' not found.")
            return []

        # Create binary mask for the class
        mask = (pred_labels == target_label_idx).astype(np.uint8)
        mask[:100, :] = 0 
        binary_mask = mask * 255

        # Find contours (connected components)
        contours, _ = cv2.findContours(binary_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        bounding_boxes = []
        for cnt in contours:
            x, y, bw, bh = cv2.boundingRect(cnt)
            if bw * bh >= min_area:
                bounding_boxes.append((x, y, bw, bh))

        return bounding_boxes