import numpy as np
from pros_car_py.path_planing import search_nearest

class ControllerPurePursuit:
    def __init__(self, kp=1.0, Lfc=0.3):
        self.path = None
        self.kp = kp
        self.Lfc = Lfc
        # TODO only find forwad index
        self.max_idx = 0

    def feedback(self, x, y, yaw, v):
        if self.path is None or len(self.path) == 0:
            print("[Pure Pursuit] No path!!")
            return 0.0, None
        # TODO only find forwad index
        min_idx, _ = search_nearest(self.path, (x, y))
        
        Ld = self.kp * v + self.Lfc
        target_idx = min_idx
        for i in range(min_idx, len(self.path) - 1):
            dist = np.linalg.norm(self.path[i+1] - np.array([x, y]))
            if dist > Ld:
                target_idx = i
                break
        target = self.path[target_idx]

        dx = target[0] - x
        dy = target[1] - y
        alpha = np.arctan2(dy, dx) - yaw
        alpha = (alpha + np.pi) % (2 * np.pi) - np.pi

        omega = (2.0 * v * np.sin(alpha)) / Ld
        return omega, target