import os
import abc
import cv2
import yaml
import heapq
import numpy as np
from pros_car_py.path_modules.cubic_spline import cubic_spline_2d

def search_nearest(path, pos):
    """
    path: list of (x, y) or numpy array shape=(N, 2)
    pos: (x, y)
    return: (index, squared_distance)
    """
    path = np.array(path)  # 保證是 np.ndarray

    min_dist = float('inf')
    min_id = -1

    for i in range(path.shape[0]):
        dist = (pos[0] - path[i, 0]) ** 2 + (pos[1] - path[i, 1]) ** 2
        if dist < min_dist:
            min_dist = dist
            min_id = i

    return min_id, min_dist

def Bresenham(x0, x1, y0, y1):
    rec = []
    dx = abs(x1 - x0)
    dy = abs(y1 - y0)
    x, y = x0, y0
    sx = -1 if x0 > x1 else 1
    sy = -1 if y0 > y1 else 1
    if dx > dy:
        err = dx / 2.0
        while x != x1:
            rec.append((x, y))
            err -= dy
            if err < 0:
                y += sy
                err += dx
            x += sx
    else:
        err = dy / 2.0
        while y != y1:
            rec.append((x, y))
            err -= dx
            if err < 0:
                x += sx
                err += dy
            y += sy
    return rec

def pos_int(p):
    return (int(p[0]), int(p[1]))

def distance(n1, n2):
        d = np.array(n1) - np.array(n2)
        return np.hypot(d[0], d[1])

class MapLoader:
    def __init__(self, path):
        aruco_path = os.path.join(path, 'config', 'aruco_location.yaml')

        with open(aruco_path, 'r') as f:
            full_config = yaml.safe_load(f)

        self.unflipped_ids = full_config.get('unflipped_ids', [])
        self.aruco_config = {int(k): v for k, v in full_config.items() if k != 'unflipped_ids'}

        print("\nLoaded ArUco config:")
        print("Unflipped IDs:", self.unflipped_ids)
        print("Marker Config:")
        for marker_id, pose in self.aruco_config.items():
            print(f"  ID {marker_id}: {pose}")

        map_path = os.path.join(path, 'map')
        yaml_path = os.path.join(map_path, 'map01.yaml')
        with open(yaml_path, 'r') as f:
            map_metadata = yaml.safe_load(f)

        image_path = os.path.join(map_path, map_metadata['image'])
        self.resolution = map_metadata['resolution']
        self.origin = map_metadata['origin']
        self.occupied_thresh = map_metadata.get('occupied_thresh', 0.65)

        img = cv2.imread(image_path, cv2.IMREAD_UNCHANGED)
        if img is None:
            raise RuntimeError(f"Can't load map: {image_path}")
        if len(img.shape) == 3:
            img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # self.map = cv2.flip(img, 0)
        self.map = img
        self.map = cv2.GaussianBlur(self.map, (5, 5), 0)
        thresh_val = int(self.occupied_thresh * 255)
        _, self.map = cv2.threshold(self.map, thresh_val, 255, cv2.THRESH_BINARY)
        self.height, self.width = self.map.shape

    def position_to_pixel(self, position):
        """
        position: (x, y) in meters
        return: (u, v) in pixels
        """
        x, y = position
        u = int((x - self.origin[0]) / self.resolution)
        v = int((y - self.origin[1]) / self.resolution)

        v = self.height - v
        return (u, v)

    
    def pixel_to_position(self, pixel):
        """
        pixel: (u, v) in pixels
        return: (x, y) in meters
        """
        u, v = pixel
        v = self.height - v
        x = u * self.resolution + self.origin[0]
        y = v * self.resolution + self.origin[1]
        return (x, y)

class Planner:
    def __init__(self, maploader:MapLoader):
        self.maploader = maploader
        self.path = np.array([[0.0,0.0,0.0,0.0]])

    @abc.abstractmethod
    def planning(self, start, goal):
        return NotImplementedError
    
class PlannerAStar(Planner):
    def __init__(self, maploader:MapLoader, inter=5):
        super().__init__(maploader)
        self.inter = inter
        self.initialize()

    def initialize(self):
        self.queue = [] 
        self.entry_finder = {} 
        self.parent = {}
        self.h = {} # Distance from start to node
        self.g = {} # Distance from node to goal
        self.goal_node = None

    def _key_func(self, p):
        return self.g[p] + self.h[p]
    
    def push_to_queue(self, node):
        priority = self.g[node] + self.h[node]
        if node not in self.entry_finder:
            heapq.heappush(self.queue, (priority, node))
            self.entry_finder[node] = True

    def pop_from_queue(self):
        while self.queue:
            _, node = heapq.heappop(self.queue)
            if node in self.entry_finder:
                del self.entry_finder[node]
                return node
        return None

    def planning(self, start=(100,200), goal=(375,520), inter=None, img=None):
        # img = cv2.cvtColor(self.maploader.map, cv2.COLOR_GRAY2BGR)
        if inter is None:
            inter = self.inter
        # >> map coordinate
        start = self.maploader.position_to_pixel(start)
        goal = self.maploader.position_to_pixel(goal)

        # Initialize 
        self.initialize()
        self.g[start] = 0
        self.h[start] = distance(start, goal)
        self.parent[start] = None
        self.push_to_queue(start)

        while self.queue:
            current_node = self.pop_from_queue()
            if img is not None: img[current_node[1]][current_node[0]] = (0,0,1)
            if current_node is None:
                break

            if self.h[current_node] < inter:
                self.goal_node = current_node
                break
            
            for dx in range(-inter, inter+1, inter):
                for dy in range(-inter, inter+1, inter):
                    nx = current_node[0] + dx
                    ny = current_node[1] + dy
                    neighbor = (nx,ny)
                    
                    if  0 <= nx < self.maploader.map.shape[1] and \
                        0 <= ny <  self.maploader.map.shape[0] and \
                        self.maploader.map[ny][nx] >= 200:
                        if img is not None: img[ny][nx] = (1,0,0)
                        t_g = self.g[current_node] + inter 
                        if neighbor not in self.g or t_g < self.g[neighbor]:
                            self.g[neighbor] = t_g
                            self.h[neighbor] = distance(neighbor, goal)
                            self.parent[neighbor] = current_node
                            self.push_to_queue(neighbor)
        
        # Extract path
        path = []
        p = self.goal_node
        if p is None:
            return
        while p is not None:
            path.insert(0, p)
            p = self.parent[p]
        if path[-1] != goal:
            path.append(goal)
        # cv2.imshow("Display window", img)
        # k = cv2.waitKey(0)
        # path = cubic_spline_2d(path, interval=inter//2) # get list[(x,y,yaw,curv)]
        # spilt_p = lambda p: [*self.maploader.pixel_to_position(p[:2]), p[2], p[3]]
    
        spilt_p = lambda p: [*self.maploader.pixel_to_position(p), 0, 0]
        self.path = np.array([spilt_p(p) for p in path])
    
class PlannerRRTStar(Planner):
    def __init__(self, maploader:MapLoader, extend_len=5):
        super().__init__(maploader)
        self.extend_len = extend_len 

    def _random_node(self, goal, shape):
        r = np.random.choice(2,1,p=[0.5,0.5])
        if r==1:
            return (float(goal[0]), float(goal[1]))
        else:
            rx = float(np.random.randint(int(shape[1])))
            ry = float(np.random.randint(int(shape[0])))
            return (rx, ry)

    def _nearest_node(self, samp_node):
        min_dist = 99999
        min_node = None
        for n in self.ntree:
            dist = distance(n, samp_node)
            if dist < min_dist:
                min_dist = dist
                min_node = n
        return min_node

    def _check_collision(self, n1, n2):
        n1_ = pos_int(n1)
        n2_ = pos_int(n2)
        line = Bresenham(n1_[0], n2_[0], n1_[1], n2_[1])
        for pts in line:
            if self.maploader.map[int(pts[1]),int(pts[0])] < 125:
                return True
        return False

    def _steer(self, from_node, to_node, extend_len):
        vect = np.array(to_node) - np.array(from_node)
        v_len = np.hypot(vect[0], vect[1])
        v_theta = np.arctan2(vect[1], vect[0])
        if extend_len > v_len:
            extend_len = v_len
        new_node = (from_node[0]+extend_len*np.cos(v_theta), from_node[1]+extend_len*np.sin(v_theta))
        if new_node[1]<0 or new_node[1]>=self.maploader.map.shape[0] or new_node[0]<0 or new_node[0]>=self.maploader.map.shape[1] or self._check_collision(from_node, new_node):
            return False, None
        else:        
            return new_node, distance(new_node, from_node)
    
    def _search_near_node(self, node, extend_len):
        return [n for n in self.ntree.keys() if distance(n, node) < extend_len]

    def _key_func(self, node):
        return self.cost[node]

    def planning(self, start, goal, extend_len=None, img=None):
        if extend_len is None:
            extend_len = self.extend_len
        # >> map coordinate
        start = self.maploader.position_to_pixel(start)
        goal = self.maploader.position_to_pixel(goal)

        # >> RRT start in pixel level
        self.ntree = {}
        self.ntree[start] = None
        self.cost = {}
        self.cost[start] = 0
        goal_node = None
        for it in range(2000): # 20000
            print("\r", it, len(self.ntree), end="")
            samp_node = self._random_node(goal, self.maploader.map.shape)
            near_node = self._nearest_node(samp_node)
            new_node, cost = self._steer(near_node, samp_node, extend_len)
            if new_node is not False:
                # Re-Parent
                near_list = self._search_near_node(new_node, extend_len*3)
                best_parent = near_node
                min_cost = cost + self.cost[near_node]
                for node in near_list:
                    if self._check_collision(node, new_node):
                        continue
                    if self.cost[node] + distance(node, new_node) < min_cost:
                        best_parent = node
                        min_cost = self.cost[node] + distance(node, new_node)
                self.ntree[new_node] = best_parent
                self.cost[new_node] = min_cost

                # Re-Wire
                for node in near_list:
                    if node == self.ntree[new_node] or self._check_collision(new_node, node):
                        continue
                    new_cost = self.cost[new_node] + distance(new_node, node)
                    if new_cost < self.cost[node]:
                        self.ntree[node] = new_node
                        self.cost[node] = new_cost
            else:
                continue
            
            if distance(new_node, goal) < extend_len:
                goal_node = new_node
                break

        # >> Extract Path
        path = []
        n = goal_node
        while n is not None:
            path.insert(0, n)
            n = self.ntree[n]
        path.append(goal)
        # print(path)
        path = cubic_spline_2d(path, interval=2) # get list[(x,y,yaw,curv)]
        spilt_p = lambda p: [*self.maploader.pixel_to_position(p[:2]), p[2], p[3]]
        self.path = np.array([spilt_p(p) for p in path])
        # self.path = np.array([self.maploader.pixel_to_position(p) for p in path])

