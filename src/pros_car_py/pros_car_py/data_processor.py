# from geometry_msgs.msg impor
import math
# LiDAR global constants
LIDAR_RANGE = 90
LIDAR_PER_SECTOR = 20
FRONT_LIDAR_INDICES = list(range(0, 16)) + list(range(-15, 0))  # front lidar indices
LEFT_LIDAR_INDICES = list(range(16, 46))  # left lidar indices
RIGHT_LIDAR_INDICES = list(range(-45, -15))  # right lidar indices

class DataProcessor:
    def __init__(self, ros_communicator):
        self.ros_communicator = ros_communicator
        print(self.ros_communicator.get_latest_lidar())

    def get_processed_amcl_pose(self):
        amcl_pose_msg = self.ros_communicator.get_latest_amcl_pose()
        position = amcl_pose_msg.pose.pose.position
        orientation = amcl_pose_msg.pose.pose.orientation
        pose = [position.x, position.y, position.z]
        quaternion = [orientation.x, orientation.y, orientation.z, orientation.w]
        return pose, quaternion

    def get_processed_lidar(self):
        lidar_msg = self.ros_communicator.get_latest_lidar()
        angle_min = lidar_msg.angle_min
        angle_max = lidar_msg.angle_max
        angle_increment = lidar_msg.angle_increment
        ranges_180 = []
        direction_180 = []
        all_ranges = lidar_msg.ranges
        for i in range(len(all_ranges)):
            if i % LIDAR_PER_SECTOR == 0:  # handle the amount of lidar.
                angle_tmp = angle_min + i * angle_increment
                ranges_180.append(all_ranges[i])
                direction_180.append([math.cos(angle_tmp), math.sin(angle_tmp), 0])
        combined_lidar_data = (
            [ranges_180[i] for i in FRONT_LIDAR_INDICES]
            + [ranges_180[i] for i in LEFT_LIDAR_INDICES]
            + [ranges_180[i] for i in RIGHT_LIDAR_INDICES]
        )
        return combined_lidar_data

    def get_processed_received_global_plan(self):
        received_global_plan_msg = self.ros_communicator.get_latest_received_global_plan()
        if received_global_plan_msg is None:
            return None, None
        path_length = len(received_global_plan_msg.poses)
        orientation_points = []
        coordinates = []
        if path_length > 0:
            last_recorded_point = received_global_plan_msg.poses[0].pose.position
            orientation_points.append((
                received_global_plan_msg.poses[0].pose.orientation.z, 
                received_global_plan_msg.poses[0].pose.orientation.w))
            coordinates.append((
                received_global_plan_msg.poses[0].pose.position.x, 
                received_global_plan_msg.poses[0].pose.position.y))
            for i in range(1, path_length):
                current_point = received_global_plan_msg.poses[i].pose.position
                distance = math.sqrt(
                    (current_point.x - last_recorded_point.x) ** 2 +
                    (current_point.y - last_recorded_point.y) ** 2
                )
                if distance >= 0.1:
                    orientation_points.append((
                        received_global_plan_msg.poses[i].pose.orientation.z, 
                        received_global_plan_msg.poses[i].pose.orientation.w))
                    coordinates.append((current_point.x, current_point.y))
                    last_recorded_point = current_point
        return orientation_points, coordinates
