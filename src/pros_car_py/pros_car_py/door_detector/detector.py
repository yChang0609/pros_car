# >> Basic package
import cv2
import numpy as np
import random
import math
from sklearn.cluster import DBSCAN

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

    clustering = DBSCAN(eps=1.0, min_samples=min_samples).fit(X)
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

def lines_can_connect(g1, g2, max_gap=30):
    g1_points = [g["start"] for g in g1] + [g["end"] for g in g1]
    g2_points = [g["start"] for g in g2] + [g["end"] for g in g2]

    min_dist = min(
        np.linalg.norm(np.array(p1) - np.array(p2))
        for p1 in g1_points for p2 in g2_points
    )

    return min_dist < max_gap

def get_further_edge_group(groups):
    def avg_y(group):
        return np.mean([pt[1] for g in group["edges"] for pt in [g["start"], g["end"]]])
    return min(groups, key=avg_y)

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


def draw_clusters(image, all_grouped_edges_dict):
    vis = image.copy()
    base_colors = {
        "door_edge": (255, 0, 0),
        "pillar_edge": (0, 255, 0),
        "whell_edge": (0, 0, 255),
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

class DoorDetector:
    def __init__(self):
        self.cluster_centers = np.load("/workspaces/src/pros_car_py/kmeans_models/kmeans_centers.npy")
        self.label_map = {
            0: "wall",
            1: "pillar",
            2: "door",
            3: "floor",
            4: "other",
        }

    def classify_color(self, color):
        distances = np.linalg.norm(self.cluster_centers - color, axis=1)
        return int(np.argmin(distances))
        
    def fast_object_check(self, image, detect_target):
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(gray, 50, 150)
        dx = cv2.Sobel(gray, cv2.CV_64F, 1, 0)
        dy = cv2.Sobel(gray, cv2.CV_64F, 0, 1)

        if detect_target is None:
            detect_target = ["floor", "door", "pillar", "wall"]

        label_flags = {label: False for label in detect_target}

        ys, xs = np.where(edges == 255)
        mask = ys > 250 
        ys = ys[mask]
        xs = xs[mask]
        for x, y in zip(xs, ys):
            gx = dx[y, x]
            gy = dy[y, x]
            norm = np.sqrt(gx ** 2 + gy ** 2)
            if norm == 0:
                continue

            nx = int(round(gx / norm))
            ny = int(round(gy / norm))

            c1 = get_avg_color(image, x, y, nx, ny, side=30)
            c2 = get_avg_color(image, x, y, nx, ny, side=-30)

            if c1 is None or c2 is None:
                continue

            label1 = self.label_map[self.classify_color(c1)]
            label2 = self.label_map[self.classify_color(c2)]

            for lbl in [label1, label2]:
                if lbl in label_flags:
                    print(lbl)
                    label_flags[lbl] = True


            if all(label_flags[k] for k in detect_target):
                break

        result = {}
        for k in ["door", "wall", "pillar"]:
            if detect_target is None or k in detect_target:
                result[f"have_{k}_edge"] = label_flags[k]

        return result

    def process_image(self, image):
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(gray, 50, 150)
        dx = cv2.Sobel(gray, cv2.CV_64F, 1, 0)
        dy = cv2.Sobel(gray, cv2.CV_64F, 0, 1)
        edge_color_image = image.copy()

        results = {
            "door_edge": {},
            "pillar_edge": {},
            "whell_edge": {},
        }

        ys, xs = np.where(edges == 255)
        for idx, (x, y) in enumerate(zip(xs, ys)):
            gx = dx[y, x]
            gy = dy[y, x]
            norm = np.sqrt(gx**2 + gy**2)
            if norm == 0:
                continue

            nx = int(round(gx / norm))
            ny = int(round(gy / norm))

            angle = np.abs(np.arctan2(gy, gx) * 180.0 / np.pi)
            if angle < 20 or angle > 100:
                continue

            c1 = get_avg_color(image, x, y, nx, ny, side=80)
            c2 = get_avg_color(image, x, y, nx, ny, side=-80)

            if c1 is None or c2 is None:
                continue

            label1 = self.label_map[self.classify_color(c1)]
            label2 = self.label_map[self.classify_color(c2)]
            labels = [label1, label2]
            edge_color_image[y, x] = (0, 0, 255)
            edge_id = f"edge_{idx}"
            if "floor" in labels:
                if "door" in labels:
                    edge_color_image[y, x] = (255, 0, 0)
                    results["door_edge"][edge_id] = {"start": (x - nx, y - ny), "end": (x + nx, y + ny)}
                elif "wall" in labels:
                    edge_color_image[y, x] = (0, 0, 255)
                    results["whell_edge"][edge_id] = {"start": (x - nx, y - ny), "end": (x + nx, y + ny)}
                elif "pillar" in labels:
                    edge_color_image[y, x] = (0, 255, 0)
                    results["pillar_edge"][edge_id] = {"start": (x - nx, y - ny), "end": (x + nx, y + ny)}
                # else:
                    # edge_color_image[y, x] = (255, 0, 255)
        return results, edge_color_image
    
    def scan(self, image, detect_target=None, fast_check=False):
        if fast_check:  
            return self.fast_object_check(image, detect_target)
        
        edges, self.edge_color_image = self.process_image(image)
        
        results = {}

        for key in ["door_edge", "pillar_edge", "whell_edge"]:
            if detect_target is None or key[:-5] in detect_target:
                results[key] = edges[key]

        results["have_pillar_edge"] = len(edges["pillar_edge"]) > 0
        results["have_whell"] = len(edges["whell_edge"]) > 0
        results["have_door"] = len(edges["door_edge"]) > 0

        return results
    
    def get_group_center(self, edges):
        xs = [e["start"][0] + e["end"][0] for e in edges]
        ys = [e["start"][1] + e["end"][1] for e in edges]
        if len(xs) == 0:
            return None
        return int(sum(xs) / (2 * len(xs))), int(sum(ys) / (2 * len(ys)))
