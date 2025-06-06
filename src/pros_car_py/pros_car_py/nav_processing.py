from pros_car_py.nav2_utils import (
    get_yaw_from_quaternion,
    get_direction_vector,
    get_angle_to_target,
    calculate_angle_point,
    cal_distance,
)
import numpy as np
from pros_car_py.data_processor import DataProcessor
from pros_car_py.ros_communicator import RosCommunicator
from pros_car_py.path_modules.path_planing import Planner, PlannerAStar, MapLoader
from pros_car_py.path_modules.path_tracking import ControllerPurePursuit
from pros_car_py.door_detector.detector import \
    (DoorDetector, 
     group_parallel_lines, 
     lines_can_connect, 
     get_further_edge_group,
     draw_clusters,
     angle_between,
     door_groups_connect_via_pillar)
import time
import cv2


class Nav2Processing:
    def __init__(self, ros_communicator:RosCommunicator, data_processor:DataProcessor):
        self.ros_communicator = ros_communicator
        self.data_processor = data_processor
        self.finishFlag = False
        self.global_plan_msg = None
        self.index = 0
        self.index_length = 0
        self.recordFlag = 0
        self.goal_published_flag = False

    def reset_nav_process(self):
        self.finishFlag = False
        self.recordFlag = 0
        self.goal_published_flag = False
        self.first_nav = True

    def finish_nav_process(self):
        self.finishFlag = True
        self.recordFlag = 1

    def get_finish_flag(self):
        return self.finishFlag

    def get_action_from_nav2_plan(self, goal_coordinates=None):
        if goal_coordinates is not None and not self.goal_published_flag:
            self.ros_communicator.publish_goal_pose(goal_coordinates)
            self.goal_published_flag = True
        orientation_points, coordinates = (
            self.data_processor.get_processed_received_global_plan()
        )
        action_key = "STOP"
        if not orientation_points or not coordinates:
            action_key = "STOP"
        else:
            try:
                z, w = orientation_points[0]
                plan_yaw = get_yaw_from_quaternion(z, w)
                car_position, car_orientation = (
                    self.data_processor.get_processed_amcl_pose()
                )
                car_orientation_z, car_orientation_w = (
                    car_orientation[2],
                    car_orientation[3],
                )
                goal_position = self.ros_communicator.get_latest_goal()
                target_distance = cal_distance(car_position, goal_position)
                if target_distance < 0.5:
                    action_key = "STOP"
                    self.finishFlag = True
                else:
                    car_yaw = get_yaw_from_quaternion(
                        car_orientation_z, car_orientation_w
                    )
                    diff_angle = (plan_yaw - car_yaw) % 360.0
                    if diff_angle < 30.0 or (diff_angle > 330 and diff_angle < 360):
                        action_key = "FORWARD"
                    elif diff_angle > 30.0 and diff_angle < 180.0:
                        action_key = "COUNTERCLOCKWISE_ROTATION"
                    elif diff_angle > 180.0 and diff_angle < 330.0:
                        action_key = "CLOCKWISE_ROTATION"
                    else:
                        action_key = "STOP"
            except:
                action_key = "STOP"
        return action_key

    def get_action_from_nav2_plan_no_dynamic_p_2_p(self, goal_coordinates=None):
        if goal_coordinates is not None and not self.goal_published_flag:
            self.ros_communicator.publish_goal_pose(goal_coordinates)
            self.goal_published_flag = True

        # 只抓第一次路径
        if self.recordFlag == 0:
            if not self.check_data_availability():
                return "STOP"
            else:
                print("Get first path")
                self.index = 0
                self.global_plan_msg = (
                    self.data_processor.get_processed_received_global_plan_no_dynamic()
                )
                self.recordFlag = 1
                action_key = "STOP"

        car_position, car_orientation = self.data_processor.get_processed_amcl_pose()

        goal_position = self.ros_communicator.get_latest_goal()
        target_distance = cal_distance(car_position, goal_position)

        # 抓最近的物標(可調距離)
        target_x, target_y = self.get_next_target_point(car_position)

        if target_x is None or target_distance < 0.5:
            self.ros_communicator.reset_nav2()
            self.finish_nav_process()
            return "STOP"

        # 計算角度誤差
        diff_angle = self.calculate_diff_angle(
            car_position, car_orientation, target_x, target_y
        )
        if diff_angle < 20 and diff_angle > -20:
            action_key = "FORWARD"
        elif diff_angle < -20 and diff_angle > -180:
            action_key = "CLOCKWISE_ROTATION"
        elif diff_angle > 20 and diff_angle < 180:
            action_key = "COUNTERCLOCKWISE_ROTATION"
        return action_key

    def check_data_availability(self):
        return (
            self.data_processor.get_processed_received_global_plan_no_dynamic()
            and self.data_processor.get_processed_amcl_pose()
            and self.ros_communicator.get_latest_goal()
        )

    def get_next_target_point(self, car_position, min_required_distance=0.5):
        """
        選擇距離車輛 min_required_distance 以上最短路徑然後返回 target_x, target_y
        """
        if self.global_plan_msg is None or self.global_plan_msg.poses is None:
            print("Error: global_plan_msg is None or poses is missing!")
            return None, None
        while self.index < len(self.global_plan_msg.poses) - 1:
            target_x = self.global_plan_msg.poses[self.index].pose.position.x
            target_y = self.global_plan_msg.poses[self.index].pose.position.y
            distance_to_target = cal_distance(car_position, (target_x, target_y))

            if distance_to_target < min_required_distance:
                self.index += 1
            else:
                self.ros_communicator.publish_selected_target_marker(
                    x=target_x, y=target_y
                )
                return target_x, target_y

        return None, None

    def calculate_diff_angle(self, car_position, car_orientation, target_x, target_y):
        target_pos = [target_x, target_y]
        diff_angle = calculate_angle_point(
            car_orientation[2], car_orientation[3], car_position[:2], target_pos
        )
        return diff_angle

    def filter_negative_one(self, depth_list):
        return [depth for depth in depth_list if depth != -1.0]

    def camera_nav(self):
        """
        YOLO 目標資訊 (yolo_target_info) 說明：

        - 索引 0 (index 0)：
            - 表示是否成功偵測到目標
            - 0：未偵測到目標
            - 1：成功偵測到目標

        - 索引 1 (index 1)：
            - 目標的深度距離 (與相機的距離，單位為公尺)，如果沒偵測到目標就回傳 0
            - 與目標過近時(大約 40 公分以內)會回傳 -1

        - 索引 2 (index 2)：
            - 目標相對於畫面正中心的像素偏移量
            - 若目標位於畫面中心右側，數值為正
            - 若目標位於畫面中心左側，數值為負
            - 若沒有目標則回傳 0

        畫面 n 個等分點深度 (camera_multi_depth) 說明 :

        - 儲存相機畫面中央高度上 n 個等距水平點的深度值。
        - 若距離過遠、過近（小於 40 公分）或是實體相機有時候深度會出一些問題，則該點的深度值將設定為 -1。
        """
        yolo_target_info = self.data_processor.get_yolo_target_info()
        camera_multi_depth = self.data_processor.get_camera_x_multi_depth()
        if camera_multi_depth == None or yolo_target_info == None:
            return "STOP"

        camera_forward_depth = self.filter_negative_one(camera_multi_depth[7:13])
        camera_left_depth = self.filter_negative_one(camera_multi_depth[0:7])
        camera_right_depth = self.filter_negative_one(camera_multi_depth[13:20])

        action = "STOP"
        limit_distance = 0.7

        if all(depth > limit_distance for depth in camera_forward_depth):
            if yolo_target_info[0] == 1:
                if yolo_target_info[2] > 200.0:
                    action = "CLOCKWISE_ROTATION_SLOW"
                elif yolo_target_info[2] < -200.0:
                    action = "COUNTERCLOCKWISE_ROTATION_SLOW"
                else:
                    if yolo_target_info[1] < 0.8:
                        action = "STOP"
                    else:
                        action = "FORWARD_SLOW"
            else:
                action = "FORWARD"
        elif any(depth < limit_distance for depth in camera_left_depth):
            action = "CLOCKWISE_ROTATION"
        elif any(depth < limit_distance for depth in camera_right_depth):
            action = "COUNTERCLOCKWISE_ROTATION"
        return action

    def camera_nav_unity(self):
        """
        YOLO 目標資訊 (yolo_target_info) 說明：

        - 索引 0 (index 0)：
            - 表示是否成功偵測到目標
            - 0：未偵測到目標
            - 1：成功偵測到目標

        - 索引 1 (index 1)：
            - 目標的深度距離 (與相機的距離，單位為公尺)，如果沒偵測到目標就回傳 0
            - 與目標過近時(大約 40 公分以內)會回傳 -1

        - 索引 2 (index 2)：
            - 目標相對於畫面正中心的像素偏移量
            - 若目標位於畫面中心右側，數值為正
            - 若目標位於畫面中心左側，數值為負
            - 若沒有目標則回傳 0

        畫面 n 個等分點深度 (camera_multi_depth) 說明 :

        - 儲存相機畫面中央高度上 n 個等距水平點的深度值。
        - 若距離過遠、過近（小於 40 公分）或是實體相機有時候深度會出一些問題，則該點的深度值將設定為 -1。
        """
        yolo_target_info = self.data_processor.get_yolo_target_info()
        camera_multi_depth = self.data_processor.get_camera_x_multi_depth()
        yolo_target_info[1] *= 100.0
        camera_multi_depth = list(
            map(lambda x: x * 100.0, self.data_processor.get_camera_x_multi_depth())
        )

        if camera_multi_depth == None or yolo_target_info == None:
            return "STOP"

        camera_forward_depth = self.filter_negative_one(camera_multi_depth[7:13])
        camera_left_depth = self.filter_negative_one(camera_multi_depth[0:7])
        camera_right_depth = self.filter_negative_one(camera_multi_depth[13:20])
        action = "STOP"
        limit_distance = 10.0
        print(yolo_target_info[1])
        if all(depth > limit_distance for depth in camera_forward_depth):
            if yolo_target_info[0] == 1:
                if yolo_target_info[2] > 200.0:
                    action = "CLOCKWISE_ROTATION_SLOW"
                elif yolo_target_info[2] < -200.0:
                    action = "COUNTERCLOCKWISE_ROTATION_SLOW"
                else:
                    if yolo_target_info[1] < 2.0:
                        action = "STOP"
                    else:
                        action = "FORWARD_SLOW"
            else:
                action = "FORWARD"
        elif any(depth < limit_distance for depth in camera_left_depth):
            action = "CLOCKWISE_ROTATION"
        elif any(depth < limit_distance for depth in camera_right_depth):
            action = "COUNTERCLOCKWISE_ROTATION"
        return action

    def test_loop(self):
        start = time.time() 
        while True:
            diff = time.time() - start
            if diff < 1.0:
                yield "CLOCKWISE_ROTATION"
            elif diff < 4.0:
                yield "FORWARD"
            elif diff < 7.0:
                yield "CLOCKWISE_ROTATION"
            elif diff < 15.0:
                yield "FORWARD"
            else:
                yield "STOP"
    
    def publish_map_data(self, path_planner:Planner):
        map = cv2.flip(path_planner.maploader.map, 0)
        data = []
        for row in map:
            for pixel in row:
                occ = 100 if pixel < path_planner.maploader.occupied_thresh * 255 else 0
                data.append(occ)
        self.ros_communicator.publish_map(
            data, path_planner.maploader.origin, path_planner.maploader.resolution,
            path_planner.maploader.width, path_planner.maploader.height)
        
    def rotate_to_angle(self, kp, kd, target_angle, threshold, min_speed, max_speed):
        prev_diff = None
        while True:
            pose = self.data_processor.get_aruco_estimate_pose()
            current_yaw = get_yaw_from_quaternion(pose["qz"], pose["qw"])
            diff = (target_angle - current_yaw + 180) % 360 - 180

            if abs(diff) < threshold:
                yield [0.0, 0.0, 0.0, 0.0]
                break

            diff_rate = 0.0 if prev_diff is None else diff - prev_diff
            prev_diff = diff

            turn_speed = kp * diff + kd * diff_rate
            if abs(turn_speed) < min_speed:
                turn_speed = min_speed * np.sign(turn_speed)
            turn_speed = np.clip(turn_speed, -max_speed, max_speed)

            yield [-turn_speed, turn_speed, -turn_speed, turn_speed]
            
    def plan_and_track_to_goal(self, goal, planner:Planner, controller:ControllerPurePursuit, base_speed, omega_gain, dist_th):
        pose = self.data_processor.get_aruco_estimate_pose()
        planner.planning((pose["x"], pose["y"]), goal)
        self.ros_communicator.publish_plan(planner.path)
        controller.path = np.array(planner.path)

        while True:
            pose = self.data_processor.get_aruco_estimate_pose()
            robot_pos = np.array([pose["x"], pose["y"]])
            robot_yaw = get_yaw_from_quaternion(pose["qz"], pose["qw"])
            robot_v = 1.0

            omega, target = controller.feedback(x=robot_pos[0], y=robot_pos[1], yaw=np.deg2rad(robot_yaw), v=robot_v)
            self.ros_communicator.publish_selected_target_marker(target[0], target[1])
            if target is None or np.linalg.norm(robot_pos - controller.path[-1][:2]) < dist_th:
                break

            turn_adjust = omega * omega_gain
            v_l = base_speed - turn_adjust
            v_r = base_speed + turn_adjust
            yield [v_l, v_r, v_l, v_r]

    def align_to_pikachu_center(self, rotate_speed, kp, kd, min_speed, max_speed):
        prev_error = None
        while True:
            image = self.data_processor.get_latest_image()
            is_detected, position = self.ros_communicator.detect_pikachu(image, (0, image.shape[1]))

            if not is_detected:
                yield [rotate_speed, -rotate_speed, rotate_speed, -rotate_speed]
                continue

            error = image.shape[1] // 2 - position.x
            if abs(error) < 10:
                yield [0.0, 0.0, 0.0, 0.0]
                break

            error_rate = 0.0 if prev_error is None else error - prev_error
            prev_error = error

            turn_speed = kp * error + kd * error_rate
            if abs(turn_speed) < min_speed:
                turn_speed = min_speed * np.sign(turn_speed)
            turn_speed = np.clip(turn_speed, -max_speed, max_speed)

            yield [-turn_speed, turn_speed, -turn_speed, turn_speed]

    def forward_and_align_until_pikachu_lost(self, base_speed, Kp, Kd, max_turn, center_th=10):
        prev_error = None
        while True:
            image = self.data_processor.get_latest_image()
            is_detected, position = self.ros_communicator.detect_pikachu(image, (0, image.shape[1]))

            if not is_detected:
                print("[Pokemon] Gotcha Pikachu!!!!")
                yield [10.0]*4
                continue
            
            image_center_x = image.shape[1] // 2
            error = image_center_x - position.x

            # Check if Pikachu is already centered
            if abs(error) < center_th:
                v_l = base_speed
                v_r = base_speed
            else:
                error_rate = 0.0 if prev_error is None else error - prev_error
                prev_error = error

                turn_adjust = Kp * error + Kd * error_rate
                turn_adjust = np.clip(turn_adjust, -max_turn, max_turn)

                # Reduce base speed when turning
                max_base_speed = base_speed
                min_base_speed = 2.0
                error_norm = min(abs(error) / image_center_x, 1.0)
                adjusted_base = max_base_speed * (1 - error_norm) + min_base_speed * error_norm

                v_l = adjusted_base - turn_adjust
                v_r = adjusted_base + turn_adjust

            print(f"[Track] error={error}, v_l={v_l:.1f}, v_r={v_r:.1f}")
            yield [v_l, v_r, v_l, v_r]
    def fix_living_room_nav(self):
        print(f"\n")    
        rotate_speed = 8.0
        forward_speed = 15.0
        show_state = True
        while True:
            image = self.data_processor.get_latest_image()
            is_detected, position = self.ros_communicator.detect_pikachu(image, (0, image.shape[1]))
            if is_detected:
                if show_state:print("[Wall Search] Pikachu detected, aligning temporarily...")
                break
            yield [-rotate_speed, rotate_speed, -rotate_speed, rotate_speed]

        yield from self.forward_and_align_until_pikachu_lost(forward_speed, 0.1, 0.5, 10.0)

    def random_living_room_nav(self):
        print(f"\n")    
        start = time.clock() 
        rotate_speed = 8.0
        forward_speed = 15.0
        show_state = True
        while True:
            image = self.data_processor.get_latest_image()
            is_detected, position = self.ros_communicator.detect_pikachu(image, (0, image.shape[1]))
            if is_detected or (time.clock() - start)> 5:
                if show_state:print("[Wall Search] Pikachu detected, aligning temporarily...")
        
                break
            yield [-rotate_speed, rotate_speed, -rotate_speed, rotate_speed]

        yield from self.forward_and_align_until_pikachu_lost(forward_speed, 0.1, 0.5, 10.0)

    def random_living_room_nav(self):
        # === Configurable Parameters ===
        map_path = "/workspaces/src/pros_car_py/config/living_room"
        G_A = 0.0
        G_B = 45.0
        G_C = 90.0
        angles = [G_A, G_B, G_C]  # Scan directions in degrees
        goal_map = {
            G_A: (2.0, 0.0),
            G_B: (2.0, 3.0),
            G_C: (0.0, 4.0),
        }
        goal_speed_scale = 2.0
        final_goal_dist_th = 1.0 # Threshold distance (in meters) to consider the robot has reached the final goal.
        rotation_th = 5.0 # Acceptable yaw angle error (in degrees) when rotating to a target angle during scanning
        forward_speed = 20.0 * goal_speed_scale # Base forward speed used when driving straight toward Pikachu after detection.
        rotate_speed = 10.0 * goal_speed_scale # Speed used for basic rotation (deg/s or unitless motor power) when trying to align.
        tracking_base_speed = 20.0 * goal_speed_scale # Base speed (m/s or unitless) for path tracking using pure pursuit.
        tracking_omega_gain = 30.0 * goal_speed_scale # Gain used to scale the angular velocity (omega) from pure pursuit into differential wheel speeds.
        align_kp, align_kd = 0.1, 0.08 # Proportional and Derivative gains used for PD control during visual alignment of Pikachu in the image center.
        min_turn_speed, max_turn_speed = 5.0 * goal_speed_scale, 10.0 * goal_speed_scale # Limits for turning speed during alignment: ensures minimum torque and caps max speed for stability.

        # === Initialization ===
        path_planner = PlannerAStar(MapLoader(map_path), inter=5)
        controller = ControllerPurePursuit()
        self.ros_communicator.publish_aruco_marker_config(
            path_planner.maploader.unflipped_ids,
            path_planner.maploader.aruco_config)
        self.publish_map_data(path_planner)

        # === Wait for ArUco Pose ===
        while(not self.data_processor.get_aruco_estimate_pose()): 
            yield "STOP"
        # === Step 1: Scan 3 angles and detect Pikachu ===
        detected_angle = None
        for angle in angles:
            # Rotate to target angle
            yield from self.rotate_to_angle(align_kp, align_kd, angle, rotation_th, min_turn_speed, max_turn_speed)

            # Check for Pikachu
            image = self.data_processor.get_latest_image()
            is_detected, _ = self.ros_communicator.detect_pikachu(image, (0, image.shape[1]))
            if is_detected:
                print(f"[Detect] Pikachu detected at {angle} deg")
                detected_angle = angle
                break
            else:
                print(f"[Detect] No Pikachu at {angle} deg")
        if detected_angle == None: detected_angle = G_B
        goal = goal_map[detected_angle]

        # === Step 2: If front detect, go straight until Pikachu disappears ===
        if detected_angle == G_C:
            yield from self.forward_and_align_until_pikachu_lost(forward_speed, align_kp, align_kd, max_turn_speed)
        else:
            yield from self.rotate_to_angle(align_kp, align_kd, detected_angle, rotation_th, min_turn_speed, max_turn_speed)
        # === Step 3: Path planning and tracking ===
        # input:(goal, planner, controller, base_speed, omega_gain, dist_th)
        yield from self.plan_and_track_to_goal(goal, path_planner, controller, tracking_base_speed, tracking_omega_gain, final_goal_dist_th)
        
        # === Step 4: Final face alignment using bounding box position ===
        # input:(rotate_speed, kp, kd, min_speed, max_speed)
        yield from self.align_to_pikachu_center(rotate_speed, align_kp, align_kd, min_turn_speed, max_turn_speed)
        
        # === Step 5: Final forward motion to catch Pikachu ===
        yield from self.forward_and_align_until_pikachu_lost(forward_speed, align_kp, align_kd, max_turn_speed)       
    
    def align_to(
        self, target, center_x, base_speed=4.0,
        Kp=0.01, max_turn=4.0, center_th=10,
        exten_error=None,
        ratio=0.8
    ):
        error_main = center_x - target[0]
        turn_adjust_total = Kp * error_main if abs(error_main) > center_th else 0
        print(f"[BaseErr] error={error_main}, Kp={Kp}, adjust={turn_adjust_total:.2f}")

        if exten_error:
            for name, err_info in exten_error.items():
                obstacle_centers = err_info.get("obstacle_center", [])
                Pgain = err_info.get("Pgain", 0)
                if len(obstacle_centers) == 0:
                    continue
                target_x, target_y = target
                print(obstacle_centers)
                closest = min(obstacle_centers, key=lambda pt: abs(pt[0] - center_x))

                e = closest[0] * np.sign(closest[0] - target_x)
                turn = e * Pgain
                turn_adjust_total += turn
                print(f"[ExtErr] {name}: closest={closest}, error={e}, Pgain={Pgain} → add_turn={turn:.2f}")

        turn_adjust_total = np.clip(turn_adjust_total, -max_turn, max_turn)
        error_norm = min(abs(turn_adjust_total) / max_turn, ratio)
        adjusted_base = base_speed * (1.0 - error_norm)

        v_l = adjusted_base - turn_adjust_total
        v_r = adjusted_base + turn_adjust_total

        print(f"[Align] turn_adjust={turn_adjust_total:.2f}, v_l={v_l:.2f}, v_r={v_r:.2f}")
        yield [v_l, v_r, v_l, v_r]

    def align_group_to_horizontal(self, group, Kp=0.1, max_turn=5.0, angle_th_deg=5):
        angles = []
        done = False
        
        for edge in group["edges"]:
            theta = angle_between(edge["start"], edge["end"])  # radians
            angles.append(theta)

        if len(angles) == 0:
            return [0.0]*4, done

        mean_angle = np.arctan2(np.mean(np.sin(angles)), np.mean(np.cos(angles)))
        print(np.rad2deg(mean_angle))
        angle_error = -90 - np.rad2deg(mean_angle)

        if abs(angle_error) < angle_th_deg:
            v_l = 0.0
            v_r = 0.0
            done = True
        else:
            turn_adjust = Kp * angle_error
            turn_adjust = np.clip(turn_adjust, -max_turn, max_turn)
            v_l = -turn_adjust
            v_r = turn_adjust

        # print(f"[AlignAngle] error={angle_error:.2f}°, v_l={v_l:.1f}, v_r={v_r:.1f}")
        return [v_l, v_r, v_l, v_r], done
    def align_to_with_orientation(
        self, 
        target, 
        group, 
        center_x, 
        base_speed=4.0, 
        Kp_center=0.01, 
        Kp_angle=0.5, 
        max_turn=4.0, 
        center_th=10,
        weight_center=0.2,
        weight_angle=0.8,
        extra_turn_error=0.0  # <-- add this
    ):
        error_center = center_x - target[0]

        angles = []
        for edge in group["edges"]:
            theta = angle_between(edge["start"], edge["end"])  # radians
            angles.append(theta)
        
        if not angles:
            angle_error = 0.0
        else:
            avg_angle = np.mean(angles)
            angle_error = np.sin(avg_angle)
        
  
        turn_center = Kp_center * error_center
        turn_angle = Kp_angle * angle_error * 180 / np.pi
        turn_adjust = weight_center * turn_center + weight_angle * turn_angle
        # turn_adjust = weight_center * turn_center + weight_angle * turn_angle + extra_turn_error
        turn_adjust = np.clip(turn_adjust, -max_turn, max_turn)

        error_norm = min(abs(error_center) / center_x, 1.0)
        adjusted_base = base_speed * (1.0 - error_norm)

        v_l = adjusted_base - turn_adjust
        v_r = adjusted_base + turn_adjust

        print(f"[Align2] center_err={error_center}, angle_err={angle_error:.2f}, v_l={v_l:.2f}, v_r={v_r:.2f}")
        yield [v_l, v_r, v_l, v_r]

    
    def avoid(self, error, base_speed=4.0, Kp=0.01, max_turn=4.0):
        turn_adjust = Kp * error
        turn_adjust = np.clip(turn_adjust, -max_turn, max_turn)

        error_norm = min(abs(error) / 320, 1.0)
        adjusted_base = base_speed * (1.0 - error_norm)

        v_l = adjusted_base - turn_adjust
        v_r = adjusted_base + turn_adjust

        print(f"[Avoid] error={error}, v_l={v_l:.2f}, v_r={v_r:.2f}")
        yield [v_l, v_r, v_l, v_r]

    def random_door_nav(self):
        print(f"\n")
        detector = DoorDetector([
                (129, 420),  
                (511, 420),  
                (600, 480),  
                (40, 480), 
            ])
        show_state = True
        # while True:
            # image = self.data_processor.get_latest_image()
            # # wall_bounding_boxes = detector.get_class_bounding_boxes(image, target_class="wall")
            # # x, y, w, h = wall_bounding_boxes[0] 
            # # print(f"h:{h}")
            # # print(f"w:{w}")
            # # if w*h > 500: color = (0, 255, 255)
            # # else:color = (0, 0, 255)
            # # cv2.rectangle(image, (x, y), (x + w, y + h), color, 2)
            # # scan_result = detector.scan(image)
            # self.ros_communicator.edge_image_publish(detector.segment_image(image))       
            # yield [0.0]*4

        # Step 1: Move backward until pillar edges are visible
        if show_state:print("[Step 1] Reversing until floor edge appears...")
        backward_speed = -15.0
        while True:
            ## >> FIXME: For Debug pub image, final need dropout
            image = self.data_processor.get_latest_image()
            label_image = detector.segment_image(image, visual=False)
            scan_result = detector.scan(label_image, ["pillar"], vis_image=image)
            self.ros_communicator.edge_image_publish(image)
            if scan_result["have_pillar_edge"]:
                if show_state:print("[Step 1] Floor detected, stop reversing.")
                break
            if show_state:print("[Step 1] Still no floor, continue reversing.")
            yield [backward_speed]*4
        yield [0.0]*4

        # Step 2: Look for doors and navigate accordingly
        turn_side = 1
        forward_speed = 30.0
        rotate_speed = 4.0
        other_side_door_len = None
        MIN_BOX_WIDTH = 100
        MIN_BOX_HEIGHT = 100  
        if show_state:print("[Step 2] Start scanning for walls and doors...")
        while True:
            # Rotate until wall is seen
            leave_wall = False
            if show_state:print("[Wall Search] Rotating to find wall...")
            while True:
                yield [-rotate_speed*turn_side, rotate_speed*turn_side, -rotate_speed*turn_side, rotate_speed*turn_side]
                image = self.data_processor.get_latest_image()
                label_image = detector.segment_image(image, visual=False)
                scan_result = detector.scan(label_image, ["wall"], fast_check=True)
                if scan_result["have_wall_edge"]:
                    if show_state:print("[Wall Search] Wall detected!")
                    is_detected, position = self.ros_communicator.detect_pikachu(image, (0, image.shape[1]))
                    if is_detected:
                        if show_state:print("[Wall Search] Pikachu detected, aligning temporarily...")
                        yield from self.forward_and_align_until_pikachu_lost(forward_speed, 0.1, 0.5, 10.0)
                    if not leave_wall:
                        print("[Wall Search] Still hugging wall, keep rotating.")
                        continue
                    if show_state:print("[Wall Search] Left wall and found again, stop rotating.")
                    yield [0.0] * 4
                    break
                else:
                    leave_wall = True

            # show detect iamge
            ## >> FIXME: For Debug pub image, final need dropout
            self.ros_communicator.edge_image_publish(detector.segment_image(image))

            # Scan door edges and process connection
            ## >> FIXME: For Debug pub image, final need dropout
            image = self.data_processor.get_latest_image()
            label_image = detector.segment_image(image, visual=False)
            scan_result = detector.scan(label_image, vis_image=image)
            self.ros_communicator.edge_image_publish(image)
            door_groups = group_parallel_lines(scan_result["door_edge"], spatial_eps=20, angle_eps=np.radians(10), min_samples=3)
            pillar_groups = group_parallel_lines(scan_result["pillar_edge"], spatial_eps=20, angle_eps=np.radians(10), min_samples=3)

            if show_state:print(f"[Door Check] Found {len(door_groups)} door groups.")

            all_groups = {
                "door_edge":door_groups,
                "pillar_edge":pillar_groups
            }
            
            self.ros_communicator.edge_image_publish(draw_clusters(image, all_groups))
            # Not have break line than detect pillar wall combind 
            if detector.detect_pillar_wall_pair(label_image,vis_image=image)[0]:
                self.ros_communicator.edge_image_publish(detector.segment_image(image))
                if show_state:print("[Exit Detection] Found pillar-wall pair (potential exit).")
                lost_wall_time = 0
                while True:
                    image = self.data_processor.get_latest_image()
                    label_image = detector.segment_image(image,visual=False)
                    obstacle_center = detector.safety_detect(label_image, image)
                    scan_result = detector.scan(label_image, detect_target=["floor"], vis_image=image,fast_check=True)
                    
                    # pillar_groups = group_parallel_lines(scan_result["pillar_edge"], spatial_eps=20, angle_eps=np.radians(10), min_samples=3)
                    # wall_groups = group_parallel_lines(scan_result["wall_edge"], spatial_eps=20, angle_eps=np.radians(10), min_samples=3)
                    wall_bounding_boxes = detector.get_class_bounding_boxes(label_image, target_class="wall")

                    if len(wall_bounding_boxes) > 0 :
                        if len(wall_bounding_boxes)>1: wall_bounding_boxes.sort(key=lambda b: b[0])
                        if turn_side == 1:
                            target_box = wall_bounding_boxes[-1] 
                        else:
                            target_box = wall_bounding_boxes[0] 
                        if show_state:
                            print(f"[Wall BBox] Selected box: (x={target_box[0]}, y={target_box[1]}, w={target_box[2]}, h={target_box[3]})")
                            if target_box[2]*target_box[3] > 500: color = (0, 255, 255)
                            else:color = (0, 0, 255)
                            cv2.rectangle(image, (target_box[0], target_box[1]), (target_box[0] + target_box[2], target_box[1] + target_box[3]), color, 2)
                            self.ros_communicator.edge_image_publish(image)

                    if len(wall_bounding_boxes) > 0:
                        target_center = (target_box[0] + target_box[2] // 2, target_box[1] + target_box[3] // 2)
                        cv2.circle(image,target_center,5,(0,0,255),3)   
                        self.ros_communicator.edge_image_publish(image)
                        is_detected, position = self.ros_communicator.detect_pikachu(image, (0, image.shape[1]))
                        if is_detected:
                            if show_state:print("[Wall Search] Pikachu detected, aligning temporarily...")
                            yield from self.forward_and_align_until_pikachu_lost(forward_speed, 0.1, 0.5, 10.0)
                        yield from self.align_to(target_center, image.shape[1] // 2, 25.0, 0.03, 6.0, 10,{"avoid error": {"obstacle_center":obstacle_center,"Pgain":0.1}})
                        
                    else:
                        yield [forward_speed]*4

                    if not scan_result["have_floor_edge"] and detector.classify_whole_image_by_ratio(label_image,0.8)[0]:
                        yield [forward_speed//5]*4
                        time.sleep(1.0)
                        if show_state:print("[Exit Align] Detected mostly floorq (exit). Break alignment.")
                        break

                # Turn back to find pillars again
                rotate_speed_ = rotate_speed*turn_side*-1
                leave_pillar = False
                while True:
                    yield [-rotate_speed_, rotate_speed_, -rotate_speed_, rotate_speed_]
                    image = self.data_processor.get_latest_image()
                    label_image = detector.segment_image(image, visual=False)
                    scan_result = detector.scan(label_image, ["pillar"], fast_check=True)
                    self.ros_communicator.edge_image_publish(detector.segment_image(image))
                    if scan_result["have_pillar_edge"]:
                        if not leave_pillar: 
                            if show_state:print("[Recovery] Still hugging pillar, continue turning.")
                            continue
                        if show_state:print("[Recovery] Pillar detected, stop turning.")
                        yield [0.0] * 4
                        break
                    else:
                        is_detected, position = self.ros_communicator.detect_pikachu(image, (0, image.shape[1]))
                        if is_detected:
                            if show_state:print("[Wall Search] Pikachu detected, aligning temporarily...")
                            yield from self.forward_and_align_until_pikachu_lost(forward_speed, 0.1, 0.5, 10.0)
                        leave_pillar = True

                # Move forward slightly until view is mostly clear
                if show_state:print("[Recovery] Moving forward to re-align...")
                while True:
                    yield [forward_speed]*4
                    image = self.data_processor.get_latest_image()
                    label_image = detector.segment_image(image, visual=False)
                    scan_result = detector.scan(label_image)
                    if  not scan_result["have_floor_edge"]:
                        if show_state:print("[Recovery] Floor occupies most of view. Done.")
                        break
                # Back to look 
                # while True:
                #     yield [backward_speed//2]*4 
                #     image = self.data_processor.get_latest_image()
                #     label_image = detector.segment_image(image,visual=False)
                #     scan_result = detector.scan(label_image, ["floor"], fast_check=True)
                #     if scan_result["have_floor_edge"]:
                #         break
                turn_side = turn_side * -1
                continue

            # Have two or more door line in image
            if len(door_groups) > 1 :
                unconnected_groups, is_connect= door_groups_connect_via_pillar(door_groups, pillar_groups, max_gap=10)
                all_groups = {
                    "door_edge": unconnected_groups
                }
                self.ros_communicator.edge_image_publish(draw_clusters(image, all_groups))
                if show_state:print(f"[Door Check] Door groups connected? {is_connect}")
                if not is_connect:
                    # Align to furthest group
                    if show_state:print("[Align Furthest] Moving towards furthest pillar...")
                    while True:
                        ## >> FIXME: For Debug pub image, final need dropout
                        image = self.data_processor.get_latest_image()
                        label_image = detector.segment_image(image, visual=False)
                        scan_result = detector.scan(label_image, vis_image=image)
                        obstacle_center = detector.safety_detect(label_image, vis_image=image, avoid_target="pillar")
                        pillar_groups = group_parallel_lines(scan_result["pillar_edge"], spatial_eps=20, angle_eps=np.radians(10), min_samples=3)

                        if len(pillar_groups) > 0:
                            target_pillar = get_further_edge_group(pillar_groups, turn_side=turn_side)
                            target_center = detector.get_group_center(target_pillar["edges"])
                        else:
                            break
                        cv2.circle(image, target_center, 5, (0,0,255), 3)   
                        self.ros_communicator.edge_image_publish(image)
                        yield from self.align_to(target_center, image.shape[1] // 2, 25.0, 0.03, 6.0, 20, {"avoid error": {"obstacle_center":obstacle_center,"Pgain":0.05}},ratio=0.6)


                    # Back to look 
                    while True:
                        yield [backward_speed//2]*4 
                        image = self.data_processor.get_latest_image()
                        label_image = detector.segment_image(image,visual=False)
                        scan_result = detector.scan(label_image, ["pillar"])
                        if scan_result["have_pillar_edge"]:
                            if show_state:print("[Recovery] Floor occupies most of view. Done.")
                            break
                turn_side = turn_side * -1


            

    def stop_nav(self):
        return "STOP"
