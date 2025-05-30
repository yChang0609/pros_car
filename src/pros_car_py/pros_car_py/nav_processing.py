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
from pros_car_py.path_modules.path_planing import PlannerRRTStar, PlannerAStar, MapLoader
from pros_car_py.path_modules.path_tracking import ControllerPurePursuit
import time

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
    

    def random_living_room_nav(self):
        
        path_planner = PlannerAStar(MapLoader("/workspaces/src/pros_car_py/config/living_room"))
        controller = ControllerPurePursuit()
        self.ros_communicator.publish_aruco_marker_config(
            path_planner.maploader.unflipped_ids,
            path_planner.maploader.aruco_config)
        
        # Debug
        data = []
        for row in path_planner.maploader.map:
            for pixel in row:
                occ = 100 if pixel < path_planner.maploader.occupied_thresh * 255 else 0
                data.append(occ)
        self.ros_communicator.publish_map(
            data, path_planner.maploader.origin, path_planner.maploader.resolution,
                path_planner.maploader.width, path_planner.maploader.height)
            
        while(not self.data_processor.get_aruco_estimate_pose()): yield "STOP"

        # >> TODO apporch: 
        # go 0 deg, 45 deg, 90 deg,thought detect aruco 9
        # if 0 deg detect pickachu goal == g1
        # if 45 deg detect pickachu goal == g2
        # if 90 deg detect pickachu goal == g3
        # # Plan a path to the goal
        # planned_path = path_planning(current_pose, goal)

        # # Follow the planned path
        # path_tracking(planned_path)
        
        # trun around and go place pikachu on image center 
        # select subgoal 

        # set goal
        angles = [0, 45, 90]
        goal_map = {
            0: (2.0, 0.0),
            45: (2.0, 3.0),
            90: (0.0, 4.0),
        }

        # >> detect 3 direction 
        detected_angle = None
        for angle in angles:
            # Rotate to the target angle
            while True:
                pose = self.data_processor.get_aruco_estimate_pose()
                current_yaw_deg = get_yaw_from_quaternion(pose["qz"], pose["qw"])
                diff = (angle - current_yaw_deg + 180) % 360 - 180

                if abs(diff) < 1.0:
                    yield [0.0, 0.0, 0.0, 0.0]
                    print(f"[Rotate] Reached target angle {angle}")
                    break

                base_speed = 5.0
                k_p = 0.1
                turn_speed = np.clip(base_speed + k_p * diff, -base_speed, base_speed)

                left_speed = -turn_speed
                right_speed = turn_speed
                action = [left_speed, right_speed, left_speed, right_speed]
                yield action

            # Detect Pikachu
            image = self.data_processor.get_latest_image()
            is_detected, cx = self.ros_communicator.detect_pikachu(image, (0, image.shape[1]))
            if is_detected:
                detected_angle = angle
                print(f"[Detect] Pikachu detected at {angle} deg")
                break
            else:
                print(f"[Detect] No Pikachu at {angle} deg")

        if detected_angle is not None:
            goal = goal_map[detected_angle]
        else:
            # >> to center point 
            goal = goal_map[45] 
        
        if detected_angle == 90:
            while True:
                image = self.data_processor.get_latest_image()
                is_detected, cx = self.ros_communicator.detect_pikachu(image, (0, image.shape[1]))
                if is_detected:
                    forward = 5.0
                    yield [forward, forward, forward, forward]
                else:
                    while True: 
                        print("[Pokemon] gotcha pikachu!!!!")
                        yield [0.0, 0.0, 0.0, 0.0]

        # Plan and track
        pose = self.data_processor.get_aruco_estimate_pose()
        path_planner.planning((pose["x"], pose["y"]), goal)
        self.ros_communicator.publish_plan(path_planner.path)
        controller.path = np.array(path_planner.path)

        while True:
            pose = self.data_processor.get_aruco_estimate_pose()
            robot_pos = np.array([pose["x"], pose["y"]])
            robot_yaw = get_yaw_from_quaternion(pose["qz"], pose["qw"])
            robot_v = 1.0

            omega, target = controller.feedback(
                x=robot_pos[0], y=robot_pos[1], yaw=np.deg2rad(robot_yaw), v=robot_v
            )
            self.ros_communicator.publish_selected_target_marker(target[0], target[1])
            if target is None or np.linalg.norm(robot_pos - controller.path[-1][:2]) < 0.1:
                break

            # Compute wheel command
            v_base = 10.0
            omega_gain = 20.0
            turn_adjust = omega * omega_gain
            v_l = v_base - turn_adjust
            v_r = v_base + turn_adjust
            yield [v_l, v_r, v_l, v_r]
            
        # Final adjustment (face Pikachu)
        while True:
            image = self.data_processor.get_latest_image()
            is_detected, position = self.ros_communicator.detect_pikachu(image, (0, image.shape[1]))
            print(position)
            error = image.shape[1] // 2  - position.x

            if not is_detected:
                rotate = 8.0
                yield [-rotate, rotate, -rotate, rotate]
                continue

            if abs(error) < 10:
                yield [0.0, 0.0, 0.0, 0.0]
                break

            # Simple P controller to align
            k_p = 0.01
            base_speed = 5.0
            correction = base_speed + error * k_p
            v_l = -correction
            v_r = correction
            yield [v_l, v_r, v_l, v_r]

        while True:
            image = self.data_processor.get_latest_image()
            is_detected, cx = self.ros_communicator.detect_pikachu(image, (0, image.shape[1]))
            if is_detected:
                forward = 5.0
                yield [forward, forward, forward, forward]
            else:
                while True: 
                    print("[Pokemon] gotcha pikachu!!!!")
                    yield [0.0, 0.0, 0.0, 0.0]



        # # >> TEST try path planning and tarcking 
        # target_list = [(2.57, 1.84)]
        # for target in target_list:
        #     pose = self.data_processor.get_aruco_estimate_pose()
        #     count = 0
        #     while(path_planner.path.shape[0] <= 1):
        #         print("Replanning")
        #         path_planner.planning((pose["x"], pose["y"]), target)
        #         count += 1
        #         yield [0.0, 0.0, 0.0, 0.0]
        #         if count < 10:
        #             continue
        #         else:
        #             return
        #     self.ros_communicator.publish_plan(path_planner.path)
        #     controller.path = np.array(path_planner.path)
        #     while True:
        #         pose = self.data_processor.get_aruco_estimate_pose()
        #         robot_pos = np.array([pose["x"], pose["y"]])
        #         robot_yaw = get_yaw_from_quaternion(pose["qz"], pose["qw"])  # in radians
        #         robot_v = 1.0  # normalized base speed for control
        #         # print(f"robot_yaw:{np.rad2deg(robot_yaw)}")
        #         omega, target = controller.feedback(
        #             x=robot_pos[0], y=robot_pos[1], yaw=np.deg2rad(robot_yaw), v=robot_v
        #         )
        #         self.ros_communicator.publish_selected_target_marker(target[0], target[1])
        #         if target is None:
        #             print("[Pure Pursuit] No target found")
        #             yield [0.0, 0.0, 0.0, 0.0]
        #             break

        #         end_dist = np.linalg.norm(robot_pos - controller.path[-1][:2])
        #         if end_dist < 0.05:
        #             break

        #         v_base = 10.0
        #         omega_gain = 20.0
        #         turn_adjust = omega * omega_gain

        #         v_l = v_base - turn_adjust
        #         v_r = v_base + turn_adjust
        #         # gain = robot_v
        #         out_l , out_r = v_l, v_r
        #         action = [out_l, out_r, out_l, out_r]
        #         # print(f"[Pure Pursuit] omega={omega:.2f}, out_l={out_l:.1f}, out_r={out_r:.1f}, target={target}")
        #         yield action

        #         pose = self.data_processor.get_aruco_estimate_pose()
        while True:
            print("[Pure Pursuit] Goal reached")
            yield [0.0, 0.0, 0.0, 0.0]
        # return "STOP"


    def stop_nav(self):
        return "STOP"
