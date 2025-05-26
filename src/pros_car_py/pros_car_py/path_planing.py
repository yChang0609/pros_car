# x=2.310, y=6.440, theta=1.571 rad

import cv2
import numpy as np
import sys
import abc

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

class Planner:
    def __init__(self, m, map_config):
        self.map = m
        self.map_config = map_config
    def position_to_pixel(self, position):
        return position
    def pixel_to_position(self, pixel):
        return pixel

    @abc.abstractmethod
    def planning(self, start, goal):
        return NotImplementedError

class PlannerRRTStar(Planner):
    def __init__(self, m, map_config, extend_len=20):
        super().__init__(m, map_config)
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
            if self.map[int(pts[1]),int(pts[0])]<0.5:
                return True
        return False

    def _steer(self, from_node, to_node, extend_len):
        vect = np.array(to_node) - np.array(from_node)
        v_len = np.hypot(vect[0], vect[1])
        v_theta = np.arctan2(vect[1], vect[0])
        if extend_len > v_len:
            extend_len = v_len
        new_node = (from_node[0]+extend_len*np.cos(v_theta), from_node[1]+extend_len*np.sin(v_theta))
        if new_node[1]<0 or new_node[1]>=self.map.shape[0] or new_node[0]<0 or new_node[0]>=self.map.shape[1] or self._check_collision(from_node, new_node):
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
        self.ntree = {}
        self.ntree[start] = None
        self.cost = {}
        self.cost[start] = 0
        goal_node = None
        for it in range(2000000): # 20000
            #print("\r", it, len(self.ntree), end="")
            samp_node = self._random_node(goal, self.map.shape)
            near_node = self._nearest_node(samp_node)
            new_node, cost = self._steer(near_node, samp_node, extend_len)
            if new_node is not False:
                # self.ntree[new_node] = near_node
                # self.cost[new_node] = cost + self.cost[near_node]

                # TODO: Re-Parent & Re-Wire
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

        # Extract Path
        path = []
        n = goal_node
        while(True):
            if n is None:
                break
            path.insert(0,n)
            node = self.ntree[n]
            n = self.ntree[n] 
        path.append(goal)
        return path
