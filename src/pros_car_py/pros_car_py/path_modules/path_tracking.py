import numpy as np
from pros_car_py.path_modules.path_planing import search_nearest

class ControllerPurePursuit:
    def __init__(self, kp=1.0, Lfc=0.3):
        """
        Pure Pursuit controller for path tracking.

        Args:
            kp (float): Gain for dynamic lookahead distance.
            Lfc (float): Base lookahead distance (in meters).
        """
        self.path = None
        self.kp = kp
        self.Lfc = Lfc
        self.max_idx = 0

    def feedback(self, x, y, yaw, v):
        """
        Compute angular velocity using pure pursuit based on current robot state.

        Args:
            x (float): Current x position of the robot.
            y (float): Current y position of the robot.
            yaw (float): Current yaw angle of the robot (in radians).
            v (float): Current linear velocity (in m/s).

        Returns:
            omega (float): Angular velocity command.
            target (tuple): Target point on the path.
        """
        if self.path is None or len(self.path) == 0:
            print("[Pure Pursuit] No path!!")
            return 0.0, None

        min_idx, _ = search_nearest(self.path, (x, y))

        # Ensure only forward progress (no backward tracking)
        if min_idx >= self.max_idx:
            self.max_idx = min_idx
        else:
            min_idx = self.max_idx

        Ld = self.kp * v + self.Lfc
        target_idx = min_idx
        for i in range(min_idx, len(self.path) - 1):
            dist = np.linalg.norm(self.path[i+1][:2] - np.array([x, y]))
            if dist > Ld:
                target_idx = i
                break
            else:
                target_idx = -1
        target = self.path[target_idx]

        dx = target[0] - x
        dy = target[1] - y

        alpha = np.arctan2(dy, dx) - yaw
        alpha = (alpha + np.pi) % (2 * np.pi) - np.pi

        # print(f"alpha:{np.rad2deg(alpha)} , yaw:{np.rad2deg(yaw)}")
        omega = (2.0 * v * np.sin(alpha)) / Ld
        return omega, target
    