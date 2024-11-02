from pros_car_py.nav2_utils import get_yaw_from_quaternion, cal_distance

class Nav2Processing:
    def __init__(self, ros_communicator, data_processor):
        self.ros_communicator = ros_communicator
        self.data_processor = data_processor
    
    def get_action_from_nav2_plan(self, goal_coordinates = None):
        if goal_coordinates is not None:
            self.ros_communicator.publish_goal_pose(goal_coordinates)
        orientation_points, coordinates = self.data_processor.get_processed_received_global_plan()
        action_key = "STOP"
        if not orientation_points or not coordinates:
            action_key = "STOP"
        else:
            try:
                z, w = orientation_points[0]
                plan_yaw = get_yaw_from_quaternion(z, w)
                car_position, car_orientation = self.data_processor.get_processed_amcl_pose()
                car_orientation_z, car_orientation_w = car_orientation[2], car_orientation[3]
                goal_position = self.ros_communicator.get_latest_goal()
                target_distance = cal_distance(car_position, goal_position)
                if target_distance < 0.3:
                    action_key = "STOP"
                else:
                    car_yaw = get_yaw_from_quaternion(car_orientation_z, car_orientation_w)
                    diff_angle = (plan_yaw - car_yaw) % 360.0
                    if diff_angle < 10.0 or (diff_angle > 350 and diff_angle < 360):
                        action_key = "FORWARD"
                    elif diff_angle > 10.0 and diff_angle < 180.0:
                        action_key = "COUNTERCLOCKWISE_ROTATION"
                    elif diff_angle > 180.0 and diff_angle < 350.0:
                        action_key = "CLOCKWISE_ROTATION"
                    else:
                        action_key = "STOP"
            except:
                action_key = "STOP"
        return action_key   
    
    def stop_nav(self):
        return "STOP"