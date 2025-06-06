# >> Basic package
import cv2
import numpy as np
import random
import math
from sklearn.cluster import DBSCAN
from scipy.spatial import KDTree
from concurrent.futures import ThreadPoolExecutor
from collections import Counter
from scipy.ndimage import convolve

KERNEL = np.array([[1,1,1],
                    [1,0,1],
                    [1,1,1]])

class UnionFind:
    def __init__(self):
        self.parent = dict()

    def find(self, x):
        if x not in self.parent:
            self.parent[x] = x
        while self.parent[x] != x:
            self.parent[x] = self.parent[self.parent[x]]  # 路徑壓縮
            x = self.parent[x]
        return x

    def union(self, x, y):
        self.parent[self.find(x)] = self.find(y)

def get_label_edges(label_map):
    neighbor_sum = convolve(label_map, KERNEL, mode='constant', cval=0)
    edge_mask = (neighbor_sum != label_map * np.sum(KERNEL))
    return edge_mask.astype(np.uint8) * 255

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

def check_connect_by_points_fast(group1, group2, max_gap=100):
    points1 = [e["start"] for e in group1["edges"]] + [e["end"] for e in group1["edges"]]
    points2 = [e["start"] for e in group2["edges"]] + [e["end"] for e in group2["edges"]]

    pts1 = np.array(points1)
    pts2 = np.array(points2)

    dists = np.linalg.norm(pts1[:, None, :] - pts2[None, :, :], axis=2)  # shape: (len(pts1), len(pts2))
    return np.any(dists < max_gap)

def check_connect_by_kdtree(group1, group2, max_gap=10): 
    points1 = [np.array(e["start"], dtype=np.float32) for e in group1["edges"]] + \
              [np.array(e["end"], dtype=np.float32) for e in group1["edges"]]
    points2 = [np.array(e["start"], dtype=np.float32) for e in group2["edges"]] + \
              [np.array(e["end"], dtype=np.float32) for e in group2["edges"]]

    if not points1 or not points2:
        return False

    tree = KDTree(points2)
    for p in points1:
        dist, _ = tree.query(p, k=1)
        if dist < max_gap:
            return True

    return False

def check_connect(group1, group2, max_gap=100, angle_thresh=np.radians(5)):
    params1, angle1 = fit_line(group1)
    params2, angle2 = fit_line(group2)
    if params1 is None or params2 is None:
        return False

    angle_diff = abs(angle1 - angle2)
    if angle_diff > np.pi / 2:
        angle_diff = np.pi - angle_diff
    if angle_diff > angle_thresh:
        return False

    x0_1, y0_1 = params1[0], params1[1]
    vx2, vy2 = params2[2], params2[3]
    x0_2, y0_2 = params2[0], params2[1]
    line_vec = np.array([vx2, vy2]).flatten()
    point_vec = np.array([x0_1 - x0_2, y0_1 - y0_2]).flatten()
    proj_len = np.abs(np.cross(line_vec, point_vec)) / np.linalg.norm(line_vec)

    return proj_len < max_gap

def fit_line(group):
    """Fit a line y = ax + b to the group edges, return (a, b) and direction angle."""
    points = []
    for edge in group["edges"]:
        points.extend([edge["start"], edge["end"]])
    
    if len(points) < 2:
        return None, None
    
    pts = np.array(points)
    [vx, vy, x0, y0] = cv2.fitLine(pts.astype(np.float32), cv2.DIST_L2, 0, 0.01, 0.01)
    direction = np.arctan2(vy, vx)  # radians

    return (x0, y0, vx, vy), direction

def group_connected_door_groups(door_groups, pillar_groups, max_gap=10):
    uf = UnionFind()

    n = len(door_groups)

    for i in range(n):
        for j in range(i + 1, n):
            g1 = door_groups[i]
            g2 = door_groups[j]

            # 如果 g1 和 g2 可以直接或透過 pillar 連通，就 union 起來
            if check_connect_by_kdtree(g1, g2, max_gap=max_gap):
                uf.union(id(g1), id(g2))
            else:
                for pillar in pillar_groups:
                    if check_connect_by_kdtree(g1, pillar, max_gap=max_gap) and \
                       check_connect_by_kdtree(pillar, g2, max_gap=max_gap):
                        uf.union(id(g1), id(g2))
                        break

    # 根據 union 結果，整理群組
    group_map = dict()  # root_id -> list of groups
    for g in door_groups:
        root = uf.find(id(g))
        if root not in group_map:
            group_map[root] = []
        group_map[root].append(g)

    # 將每個群組合併為新的 super group
    merged_groups = []
    for group_list in group_map.values():
        merged = {
            "edges": [],
            "edge_ids": []
        }
        for g in group_list:
            merged["edges"].extend(g["edges"])
            merged["edge_ids"].extend(g["edge_ids"])
        merged_groups.append(merged)

    return merged_groups

def door_groups_connect_via_pillar(door_groups, pillar_groups, max_gap=10):
    unconnectable_pairs = []
    unconnectable_groups = set()
    door_groups = group_connected_door_groups(door_groups, pillar_groups, max_gap=5)
    for i in range(len(door_groups)):
        for j in range(i + 1, len(door_groups)):
            g1 = door_groups[i]
            g2 = door_groups[j]

            if check_connect_by_kdtree(g1, g2, max_gap=max_gap):
                continue  

            bridged = False
            for pillar in pillar_groups:
                if check_connect_by_kdtree(g1, pillar, max_gap=max_gap) and \
                   check_connect_by_kdtree(pillar, g2, max_gap=max_gap):
                    bridged = True
                    break

            if not bridged:
                unconnectable_pairs.append((g1, g2))  

    for g1, g2 in unconnectable_pairs:
        unconnectable_groups.add(id(g1))
        unconnectable_groups.add(id(g2))

    result_groups = [g for g in door_groups if id(g) in unconnectable_groups]
    all_connected = len(result_groups) == 0
    return result_groups, all_connected

def lines_can_connect_fast(groups, max_gap=100, angle_thresh=np.radians(5)):
    line_params = []
    for group in groups:
        params, angle = fit_line(group)
        line_params.append((params, angle))
        
    unconnectable_groups = []
    for i in range(len(groups)):
        connectable = False
        for j in range(len(groups)):
            if i == j:
                continue
            (p1, angle1), (p2, angle2) = line_params[i], line_params[j]
            if p1 is None or p2 is None:
                continue

            angle_diff = abs(angle1 - angle2)
            if angle_diff > np.pi / 2:
                angle_diff = np.pi - angle_diff

            if angle_diff > angle_thresh:
                continue  # not parallel enough

            # Compute distance from line1 point to line2 line
            x0_1, y0_1 = p1[0], p1[1]
            vx2, vy2 = p2[2], p2[3]
            x0_2, y0_2 = p2[0], p2[1]
            line_vec = np.array([vx2, vy2]).flatten()
            point_vec = np.array([x0_1 - x0_2, y0_1 - y0_2]).flatten()

            proj_len = np.abs(np.cross(line_vec, point_vec)) / np.linalg.norm(line_vec)

            if proj_len < max_gap:
                connectable = True
                break

        if not connectable:
            unconnectable_groups.append(groups[i])
    return unconnectable_groups, len(unconnectable_groups) == 0

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
    # print(f"{min_dist} < {max_gap}:{min_dist < max_gap}")
    return min_dist < max_gap

def get_further_edge_group(groups, turn_side=0):
    if not groups:
        return None

    def avg_xy(group):
        xs = [pt[0] for e in group["edges"] for pt in [e["start"], e["end"]]]
        ys = [pt[1] for e in group["edges"] for pt in [e["start"], e["end"]]]
        return np.mean(xs), np.mean(ys)

    if turn_side == 1:
        return max(groups, key=lambda g: (avg_xy(g)[0], -avg_xy(g)[1]))
    elif turn_side == -1:
        return min(groups, key=lambda g: (avg_xy(g)[0], -avg_xy(g)[1]))
    else:
        return min(groups, key=lambda g: avg_xy(g)[1])
    
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
                # cx = int((x0 + x1) / 2)
                # cy = int((y0 + y1) / 2)
                # cv2.putText(vis, f"{key[0]}{cluster_idx}", (cx, cy), cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)

    return vis

def process_patch_with_coords(label_image, x0, y0, x1, y1):
    patch = label_image[y0:y1, x0:x1].astype(np.uint8)
    # edges = cv2.Canny(patch, threshold1=0, threshold2=255)
    edges = convolve(patch, KERNEL, mode='constant', cval=0)
    edge_mask = (edges != patch * np.sum(KERNEL))
    edge_mask = edge_mask.astype(np.uint8) * 255
    dx = cv2.Sobel(patch, cv2.CV_64F, 1, 0)
    dy = cv2.Sobel(patch, cv2.CV_64F, 0, 1)
    return {
        "region": (x0, y0, x1, y1),
        "edges": edge_mask,
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
    def __init__(self, trapezoid_pts):
        self.cluster_centers = np.load("/workspaces/src/pros_car_py/kmeans_models/kmeans_centers.npy")
        # print(self.cluster_centers)
        self.label_map = {
            0: "door",
            1: "floor",
            2: "pillar",
            3: "wall",
            4: "other",
        }
        self.label_to_id = {v: k for k, v in self.label_map.items()}
        self.trapezoid_pts = np.array(trapezoid_pts, dtype=np.int32)  
        self.executor = ThreadPoolExecutor(max_workers=8) 
    def __del__(self):
        self.executor.shutdown()
        
    def scan(self, label_image, detect_target=None, fast_check=False, vis_image=None):
        if fast_check:  
            return self.fast_object_check(label_image, detect_target)
        
        edges = self.process_image(label_image, vis_image)
        
        results = {}

        for key in ["door_edge", "pillar_edge", "wall_edge"]:
            if detect_target is None or key[:-5] in detect_target:
                results[key] = edges[key]

        results["have_pillar_edge"] = len(edges["pillar_edge"]) > 0
        results["have_wall_edge"] = len(edges["wall_edge"]) > 0
        results["have_door_edge"] = len(edges["door_edge"]) > 0
        results["have_floor_edge"] = len(edges["pillar_edge"]) > 0 or len(edges["wall_edge"]) > 0 or len(edges["door_edge"]) > 0

        return results
    
    def classify_label_from_map(self, label_image, x, y, nx, ny, side):
        px = int(round(x + nx * side))
        py = int(round(y + ny * side))
        h, w = label_image.shape
        if 0 <= px < w and 0 <= py < h:
            return self.label_map.get(label_image[py, px], "unknown")
        return "unknown"
    
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

    def classify_whole_image_by_ratio(self, label_image, threshold=0.8):
        h, w = label_image.shape
        total_pixels = h * w

        unique, counts = np.unique(label_image, return_counts=True)
        label_ratios = {}

        for label_idx, count in zip(unique, counts):
            name = self.label_map.get(label_idx, "unknown")
            label_ratios[name] = count / total_pixels

        for name, ratio in label_ratios.items():
            if ratio >= threshold:
                return True, name, ratio

        return False, None, label_ratios
        
    def slice_and_process_parallel(self, label_image, patch_size=200, y_start=100):
        h, w = label_image.shape[:2]
        tasks = []
        for y0 in range(y_start, h, patch_size//2):
            y1 = min(y0 + patch_size, h)
            for x0 in range(0, w, patch_size//2):
                x1 = min(x0 + patch_size, w)
                tasks.append((x0, y0, x1, y1))

        return list(self.executor.map(lambda args: process_patch_with_coords(label_image, *args), tasks))
    
    def fast_object_check(self, label_image, detect_target, threshold=0.01):
        if detect_target is None:
            detect_target = ["floor", "door", "pillar", "wall"]

        label_cropped = label_image[100:, :]
        h, w = label_cropped.shape

        unique, counts = np.unique(label_cropped, return_counts=True)
        total_pixels = h * w

        label_ratios = {}
        for k, v in zip(unique, counts):
            class_name = self.label_map.get(int(k), "unknown")
            label_ratios[class_name] = v / total_pixels

        result = {}
        for label in detect_target:
            ratio = label_ratios.get(label, 0.0)
            result[f"have_{label}_edge"] = ratio >= threshold

        return result

    def process_image(self, label_image, vis_image=None, ):
        # edge_color_image = np.stack([label_image]*3, axis=-1)  # Convert to color for visualization
        patches = self.slice_and_process_parallel(label_image, y_start=80)

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
            # xs = xs[::4]
            # ys = ys[::4]
            for x, y in zip(xs, ys):
                gx = dx[y, x]
                gy = dy[y, x]
                angle = np.abs(np.arctan2(gy, gx) * 180.0 / np.pi)

                if angle < 10 or angle > 170:
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
            h, w = label_image.shape
            num_samples = 30
            max_distance = 5 

            all_labels = []

            for _ in range(num_samples):
                dist = random.randint(1, max_distance)

                px1, py1 = x + nx * dist, y + ny * dist
                if 0 <= py1 < h and 0 <= px1 < w:
                    all_labels.append(label_image[py1, px1])

                px2, py2 = x - nx * dist, y - ny * dist
                if 0 <= py2 < h and 0 <= px2 < w:
                    all_labels.append(label_image[py2, px2])

            if not all_labels:
                return None

            counter = Counter(all_labels)
            most_common = counter.most_common(2)
            label_names = [self.label_map.get(label, None) for label, _ in most_common]

            label_pair = set(label_names)
            edge_id = f"edge_{idx}"
            if "floor" in label_pair:
                if vis_image is not None: vis_image[y, x] = (255, 0, 255)
                if "door" in label_pair:
                    if vis_image is not None: vis_image[y, x] = (255, 0, 0)
                    return ("door_edge", edge_id, x, y, nx, ny)
                elif "wall" in label_pair:
                    if vis_image is not None: vis_image[y, x] = (0, 0, 255)
                    return ("wall_edge", edge_id, x, y, nx, ny)
                elif "pillar" in label_pair:
                    if vis_image is not None: vis_image[y, x] = (0, 255, 0)
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

        return results

    
    def detect_pillar_wall_pair(self, label_image, vis_image=None):
        h, w= label_image.shape
        edges = get_label_edges(label_image)
        ys, xs = np.where(edges == 255)
        dx = cv2.Sobel(label_image.astype(np.uint8), cv2.CV_64F, 1, 0)
        dy = cv2.Sobel(label_image.astype(np.uint8), cv2.CV_64F, 0, 1)

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

            num_samples = 30
            max_distance = 30
            all_labels = []

            for _ in range(num_samples):
                dist = random.randint(1, max_distance)

                px1, py1 = x + nx * dist, y + ny * dist
                if 0 <= py1 < h and 0 <= px1 < w:
                    all_labels.append(label_image[py1, px1])

                px2, py2 = x - nx * dist, y - ny * dist
                if 0 <= py2 < h and 0 <= px2 < w:
                    all_labels.append(label_image[py2, px2])

            if not all_labels:
                return None
            
            counter = Counter(all_labels)
            most_common = counter.most_common(2)
            label_names = [self.label_map.get(label, None) for label, _ in most_common]

            label_pair = set(label_names)
            if vis_image is not None:vis_image[y, x] = (0, 255, 255)
            if "pillar" in label_pair and "wall" in label_pair:
                return True, (x, y)
            return None

        futures = [
            self.executor.submit(process_point, x, y)
            for x, y in all_points
        ]
        for future in futures:
            res = future.result()
            if res:
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

        # pred_indices[min_dists > 0.105] = -1
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
    def get_class_bounding_boxes(self, label_image, target_class="wall", min_area=100):
        h, w = label_image.shape

        target_label_idx = None
        for idx, name in self.label_map.items():
            if name == target_class:
                target_label_idx = idx
                break

        if target_label_idx is None:
            print(f"[Error] Class '{target_class}' not found in label_map.")
            return []

        # Create binary mask for the class
        mask = (label_image == target_label_idx).astype(np.uint8)
        mask[:100, :] = 0  # remove upper part
        binary_mask = mask * 255

        # Find contours (connected components)
        contours, _ = cv2.findContours(binary_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        bounding_boxes = []
        for cnt in contours:
            x, y, bw, bh = cv2.boundingRect(cnt)
            if bw * bh >= min_area:
                bounding_boxes.append((x, y, bw, bh))

        return bounding_boxes
    
    def safety_detect(self, label_image, vis_image=None, avoid_target="pillar"):
        avoid_id = self.label_to_id[avoid_target]

        mask = np.zeros_like(label_image, dtype=np.uint8)
        cv2.fillPoly(mask, [self.trapezoid_pts], 1)
        roi_mask = (mask == 1)

        avoid_mask = ((label_image == avoid_id) & roi_mask).astype(np.uint8)

        if vis_image is not None:
            cv2.polylines(
                vis_image,
                [self.trapezoid_pts.astype(np.int32)],
                isClosed=True,
                color=(255, 0, 255),
                thickness=2
            )
            overlay = vis_image.copy()
            overlay[roi_mask] = (
                overlay[roi_mask] * 0.5 + np.array([255, 200, 200]) * 0.5
            ).astype(np.uint8)
            vis_image[:] = overlay

        if not np.any(avoid_mask):
            return []  

        num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(avoid_mask, connectivity=8)

        obstacle_centers = []

        for i in range(1, num_labels):
            cx, cy = centroids[i]
            obstacle_centers.append((int(cx), int(cy)))

            if vis_image is not None:
                cv2.circle(vis_image, (int(cx), int(cy)), 5, (0, 0, 255), -1)
                
        return  obstacle_centers