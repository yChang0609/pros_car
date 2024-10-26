from rclpy.node import Node
from pros_car_py.car_models import DeviceDataTypeEnum, CarCControl
from geometry_msgs.msg import (
    PoseWithCovarianceStamped,
    PoseStamped,
)
from std_msgs.msg import String, Header
from nav_msgs.msg import Path
from sensor_msgs.msg import LaserScan
from trajectory_msgs.msg import JointTrajectoryPoint
import orjson
from pros_car_py.ros_communicator_config import ACTION_MAPPINGS

class RosCommunicator(Node):
    def __init__(self):
        super().__init__("RosCommunicator")

        # subscribeamcl_pose
        self.latest_amcl_pose = None
        self.subscriber_amcl = self.create_subscription(
            PoseWithCovarianceStamped, "/amcl_pose", self.subscriber_amcl_callback, 10
        )

        # subscribe goal_pose
        self.target_pose = None
        self.subscriber_goal = self.create_subscription(
            PoseStamped, "/goal_pose", self.subscriber_goal_callback, 1
        )
        
        # subscribe lidar
        self.latest_lidar = None
        self.subscriber_lidar = self.create_subscription(
            LaserScan, "/scan", self.subscriber_lidar_callback, 1
        )

        # subscribe global_plan
        self.latest_received_global_plan = None
        self.subscriber_received_global_plan = self.create_subscription(
            Path, "/received_global_plan", self.received_global_plan_callback, 1
        )

        # publish car_C_rear_wheel and car_C_front_wheel
        self.publisher_rear = self.create_publisher(String, DeviceDataTypeEnum.car_C_rear_wheel, 10)
        self.publisher_forward = self.create_publisher(String, DeviceDataTypeEnum.car_C_front_wheel, 10)

        # publish goal_pose
        self.publisher_goal_pose = self.create_publisher(PoseStamped, "/goal_pose", 10)

        # publish robot arm angle
        self.publisher_joint_trajectory = self.create_publisher(
            JointTrajectoryPoint, DeviceDataTypeEnum.robot_arm, 10
        )

    # amcl_pose callback and get_latest_amcl_pose
    def subscriber_amcl_callback(self, msg):
        self.latest_amcl_pose = msg

    def get_latest_amcl_pose(self):
        if self.latest_amcl_pose is None:
            self.get_logger().warn("No AMCL pose data received yet.")
        return self.latest_amcl_pose

    # goal callback and get_latest_goal
    def subscriber_goal_callback(self, msg):
        position = msg.pose.position
        target = [position.x, position.y, position.z]
        self.target_pose = target
    
    def get_latest_goal(self):
        if self.target_pose is None:
            self.get_logger().warn("No goal pose data received yet.")
        return self.target_pose
    
    # lidar callback and get_latest_lidar
    def subscriber_lidar_callback(self, msg):
        self.latest_lidar = msg

    def get_latest_lidar(self):
        if self.latest_lidar is None:
            self.get_logger().warn("No Lidar data received yet.")
        return self.latest_lidar
    
    # received_global_plan callback and get_latest_received_global_plan
    def received_global_plan_callback(self, msg):
        self.latest_received_global_plan = msg

    def get_latest_received_global_plan(self):
        if self.latest_received_global_plan is None:
            self.get_logger().warn("No received global plan data received yet.")
            return None
        return self.latest_received_global_plan
    
    # publish car_C_rear_wheel and car_C_front_wheel
    def publish_car_control(self, action_key,publish_rear=True, publish_front=True):
        if action_key not in ACTION_MAPPINGS:
            self.get_logger().warn(f"Unknown action key: {action_key}")
            return
        velocities = ACTION_MAPPINGS[action_key]
        self._vel1, self._vel2, self._vel3, self._vel4 = velocities
        if publish_rear:
            control_signal_rear = {
                "type": str(DeviceDataTypeEnum.car_C_rear_wheel),
                "data": dict(CarCControl(target_vel=[self._vel1, self._vel2])),
            }
            control_msg_rear = String()
            control_msg_rear.data = orjson.dumps(control_signal_rear).decode()
            self.publisher_rear.publish(control_msg_rear)
            # self.get_logger().info(f"Published to rear: {control_msg_rear}")

        if publish_front:
            control_signal_front = {
                "type": str(DeviceDataTypeEnum.car_C_front_wheel),
                "data": dict(CarCControl(target_vel=[self._vel3, self._vel4])),
            }
            control_msg_front = String()
            control_msg_front.data = orjson.dumps(control_signal_front).decode()
            self.publisher_forward.publish(control_msg_front)
            # self.get_logger().info(f"Published to front: {control_msg_front}")
    
    # publish goal_pose
    def publish_goal_pose(self, goal_pose):
        goal_pose = PoseStamped()
        goal_pose.header = Header()
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.header.frame_id = "map"
        goal_pose.pose.position.x = goal_pose[0]
        goal_pose.pose.position.y = goal_pose[1]
        goal_pose.pose.position.z = 0.0
        goal_pose.pose.orientation.w = 1.0
        self.publisher_goal_pose.publish(goal_pose)
    
    # publish robot arm angle
    def publish_robot_arm_angle(self, angle):
        joint_trajectory_point = JointTrajectoryPoint()
        joint_trajectory_point.positions = angle
        joint_trajectory_point.velocities = [0.0] * len(angle)
        self.publisher_joint_trajectory.publish(joint_trajectory_point)
