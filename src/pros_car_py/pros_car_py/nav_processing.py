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
from pros_car_py.path_planing import PlannerRRTStar, MapLoader
from pros_car_py.path_tracking import ControllerPurePursuit

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
        self.path_planner = None

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

    def random_living_room_nav(self):
        
        self.path_planner = PlannerRRTStar(MapLoader("/workspaces/src/pros_car_py/config/living_room"))
        controller = ControllerPurePursuit()
        self.ros_communicator.publish_aruco_marker_config(
            self.path_planner.maploader.unflipped_ids,
            self.path_planner.maploader.aruco_config)
        # Debug
        data = []
        for row in self.path_planner.maploader.map:
            for pixel in row:
                occ = 100 if pixel < self.path_planner.maploader.occupied_thresh * 255 else 0
                data.append(occ)
        self.ros_communicator.publish_map(
            data, self.path_planner.maploader.origin, self.path_planner.maploader.resolution,
                self.path_planner.maploader.width, self.path_planner.maploader.height)
            
        while(not self.data_processor.get_aruco_estimate_pose()): yield "STOP"
        ## For Testing
        # if self.data_processor.get_aruco_estimate_pose():
        #     print("cal path")
        #     path = self.path_planner.planning(self.data_processor.get_aruco_estimate_pose(), (2.0, 3.0))
        #     print("pub path")
        #     self.ros_communicator.publish_plan(path)
        
        
        ## >> TODO follow pseudo code 
        # # Loop until:
        # # - YOLO detects Pikachu
        # # - Pikachu is centered in the camera view
        # # - ArUco marker with ID 9 (frontier) is detected
        
        # while (
        #     YOLO_result.detected == False and
        #     YOLO_result.image_position != image_center and
        #     not ArUco_has_detected(ID=9)
        # ):
        #     yield turn_right()  # Keep turning right to search for Pikachu

        # # Once the above loop breaks (i.e., a condition is met):
        # # Use robot's current pose to determine the goal from a predefined map or dictionary
        # goal = goal_dictionary.search(robot_pose)

        # # Plan a path to the goal
        # planned_path = path_planning(current_pose, goal)

        # # Follow the planned path
        # path_tracking(planned_path)

        target_list = [(2.0, 3.0)]
        for target in target_list:
            pose = self.data_processor.get_aruco_estimate_pose()
            count = 0
            while(not self.path_planner.path):
                self.path_planner.planning((pose["x"], pose["y"]), target)
                count += 1
                if count < 10:
                    continue
            self.ros_communicator.publish_plan(self.path_planner.path)
            controller.path = np.array(self.path_planner.path)
            while True:
                pose = self.data_processor.get_aruco_estimate_pose()
                robot_pos = np.array([pose["x"], pose["y"]])
                robot_yaw = get_yaw_from_quaternion(pose["qz"], pose["qw"])  # in radians
                robot_v = 5.0  # assume forward velocity constant or from odom

                omega, target = controller.feedback(
                    x=robot_pos[0], y=robot_pos[1], yaw=robot_yaw, v=robot_v
                )
                self.ros_communicator.publish_selected_target_marker(target[0],target[1])
                if target is None:
                    print("[Pure Pursuit] No target found")
                    yield [0.0, 0.0, 0.0, 0.0]
                    break

                end_dist = np.linalg.norm(robot_pos - controller.path[-1])
                if end_dist < 1.0:
                    print("[Pure Pursuit] Goal reached")
                    yield [0.0, 0.0, 0.0, 0.0]
                    break

                # Calculate wheel velocity from diff-drive
                wheel_radius = 0.04  # meters
                wheel_base = 0.23    # meters
                v = robot_v  # base linear speed [m/s]
                # omega = np.clip(omega, -3.0, 3.0)  # limit angular speed
                # omega = np.rad2deg(omega)
                v_l = v - (omega * wheel_base / 2.0)
                v_r = v + (omega * wheel_base / 2.0)

                # Convert to wheel rotation speed (m/s -> deg/s)
                to_rad = lambda vel: vel / wheel_radius
                rad_l = to_rad(v_l)
                rad_r = to_rad(v_r)

                action = [rad_l, rad_r, rad_l, rad_r]
                print(f"[Pure Pursuit] omega={omega:.2f}, deg_l={rad_l:.1f}, deg_r={rad_r:.1f}, target={target}")
                yield action

                pose = self.data_processor.get_aruco_estimate_pose()
        return "STOP"


    def stop_nav(self):
        return "STOP"
