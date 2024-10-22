from geometry_msgs.msg import get_latest_amcl_pose
import math
from ros_communicator import RosCommunicator

# LiDAR global constants
LIDAR_RANGE = 90
LIDAR_PER_SECTOR = 20
FRONT_LIDAR_INDICES = list(range(0, 16)) + list(range(-15, 0))  # front lidar indices
LEFT_LIDAR_INDICES = list(range(16, 46))  # left lidar indices
RIGHT_LIDAR_INDICES = list(range(-45, -15))  # right lidar indices

    
def get_processed_amcl_pose(amcl_pose_msg):
    position = amcl_pose_msg.pose.pose.position
    orientation = amcl_pose_msg.pose.pose.orientation
    pose = [position.x, position.y, position.z]
    quaternion = [orientation.x, orientation.y, orientation.z, orientation.w]
    return pose, quaternion

def get_processed_lidar(lidar_msg):
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