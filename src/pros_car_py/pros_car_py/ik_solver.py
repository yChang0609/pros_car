import pybullet as p
import pybullet_data
import numpy as np
import time
import os
from ament_index_python.packages import get_package_share_directory
class RobotIKSolver:
    def __init__(self):
        # 初始化 PyBullet
        self.physics_client = p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        
        # 設置環境
        self.setup_environment()
        # 載入機器人
        self.robot_id = self.load_robot()
        
        # 強制移動到原點
        self.move_to_origin()
        
        # 設置調試參數
        self.debug_mode = True
        
        # 設置模擬參數
        self.use_realtime = False  # 不使用實時模擬

    
    def set_all_joints_to_90_degrees(self):
        """將每個關節設置到90度"""
        target_angle_degrees = 90
        target_angle_radians = np.radians(target_angle_degrees)
        
        # 設置每個關節到目標角度
        for joint_index in range(p.getNumJoints(self.robot_id)):
            self.set_joint_angle(joint_index, target_angle_radians)

    def set_joint_angle(self, joint_index, angle):
        """直設置指定關節的角度"""
        # print(f"\n設置關節 {joint_index} 的角度為 {np.degrees(angle):.1f} 度")
        
        # 使用 resetJointState 直接設置角度
        p.resetJointState(self.robot_id, joint_index, angle)
        
        # 立即更新模擬
        p.stepSimulation()
        time.sleep(1./240.)  # 控制模擬速度


    def setup_environment(self):
        """設置模擬環境"""
        # 設置重力
        p.setGravity(0, 0, -9.81)
        
        # 載入地面
        p.loadURDF("plane.urdf")
        
        # 設置相機，讓原點更容易看到
        p.resetDebugVisualizerCamera(
            cameraDistance=1.0,  # 更近的視角
            cameraYaw=45,
            cameraPitch=-20,
            cameraTargetPosition=[0, 0, 0]  # 對準原點
        )
        
        # 添加座標軸視覺化
        p.addUserDebugLine([0, 0, 0], [0.2, 0, 0], [1, 0, 0])  # X軸 紅色
        p.addUserDebugLine([0, 0, 0], [0, 0.2, 0], [0, 1, 0])  # Y軸 綠色
        p.addUserDebugLine([0, 0, 0], [0, 0, 0.2], [0, 0, 1])  # Z軸 藍色
    
    def load_robot(self):
        """載入機器人模型"""
        # 獲取 URDF 路徑
        # urdf_path = os.path.abspath("./excurate_arm/target.urdf")
        urdf_path = os.path.join(get_package_share_directory('pros_car_py'), 'urdf', 'excurate_arm', 'target.urdf')
        print(f"載入 URDF: {urdf_path}")
        
        
        # 載入機器人並設置位置
        robot_id = p.loadURDF(
            urdf_path,
            basePosition=[0, 0, 0],  # 先設置在原點
            baseOrientation=p.getQuaternionFromEuler([0, 0, 0]),
            useFixedBase=True
        )
        
        # 打印關節信息
        # self.print_joint_info(robot_id)
        
        return robot_id
    
    def move_to_origin(self):
        """強制移動機器人到原點"""
        print("\n正在移動機器人到原點...")
        
        # 獲取基座位置
        base_pos, base_orn = p.getBasePositionAndOrientation(self.robot_id)
        print(f"當前基座位置: {base_pos}")
        
        # 設置新的基座位置
        new_pos = [0, 0, 0]  # 原點
        p.resetBasePositionAndOrientation(
            self.robot_id,
            new_pos,
            p.getQuaternionFromEuler([0, 0, 0])  # 保持原始方向
        )
        
        # 驗證新位置
        new_base_pos, _ = p.getBasePositionAndOrientation(self.robot_id)
        print(f"新基座位置: {new_base_pos}")
        
        # 重置所有關節到零位
        for i in range(p.getNumJoints(self.robot_id)):
            p.resetJointState(self.robot_id, i, 0)
    
    def print_joint_info(self, robot_id):
        """打印關節信息"""
        print("\n關節信息:")
        num_joints = p.getNumJoints(robot_id)
        for i in range(num_joints):
            joint_info = p.getJointInfo(robot_id, i)
            print(f"關節 {i}: {joint_info[1].decode('utf-8')}")
            print(f"  類型: {joint_info[2]}")
            if joint_info[8] != joint_info[9]:  # 如果有關節限制
                print(f"  限制: [{np.degrees(joint_info[8]):.1f}, {np.degrees(joint_info[9]):.1f}] 度")
            else:
                print("  無限制")
            # 打印當前位置
            state = p.getJointState(robot_id, i)
            print(f"  當前角度: {np.degrees(state[0]):.1f} 度")
    
    def get_joint_states(self):
        """獲取所有關節狀態"""
        joint_states = []
        for i in range(p.getNumJoints(self.robot_id)):
            state = p.getJointState(self.robot_id, i)
            joint_states.append(state[0])
        return joint_states
    
    def solve_ik(self, target_position, target_orientation=None):
        """求解逆運動學"""
        print(f"\n開始 IK 求解...")
        print(f"目標位置: {target_position}")
        
        # 設置末端執行器索引
        end_effector_index = 5  # 需要根據實際機器人調整
        
        # 如果沒有指定方向，使用默認方向
        if target_orientation is None:
            target_orientation = p.getQuaternionFromEuler([0, 0, 0])
        
        # 獲取當前狀態
        current_joints = self.get_joint_states()
        print(f"當前關節角度（度）: {np.degrees(current_joints)}")
        
        # 求解 IK
        joint_poses = p.calculateInverseKinematics(
            self.robot_id,
            end_effector_index,
            target_position,
            target_orientation,
            maxNumIterations=100,
            residualThreshold=1e-5
        )
        
        return joint_poses
    
    def move_to_target(self, target_position, steps=50):
        """移動到目標位置"""
        # 求解 IK
        joint_poses = self.solve_ik(target_position)
        
        if joint_poses is not None:
            # print("\nIK 求解成功!")
            # print(f"目標關節角度（度）: {np.degrees(joint_poses)}")
            
            # Convert joint_poses to a list to allow modification
            joint_poses = list(joint_poses)
            
            # 確保最後兩個關節角度相同
            if len(joint_poses) >= 2:
                joint_poses[-1] = joint_poses[-2] = joint_poses[-1]  # 設置最後兩個關節角度相同
            
            # 獲取當前關節角度
            current_joints = self.get_joint_states()
            
            # 逐步移動到目標位置
            for step in range(steps + 1):
                # 計算插值
                t = step / steps
                for i in range(len(joint_poses)):
                    angle = current_joints[i] + t * (joint_poses[i] - current_joints[i])
                    p.setJointMotorControl2(
                        self.robot_id,
                        i,
                        p.POSITION_CONTROL,
                        targetPosition=angle
                    )
                
                # 更新模擬
                p.stepSimulation()
                time.sleep(1./240.)  # 控制模擬速度
                
                # 打印進度
                # if step % 10 == 0:
                #     print(f"\n步驟 {step}/{steps}")
                #     end_state = p.getLinkState(self.robot_id, 5)
                #     print(f"當前位置: {end_state[0]}")
                #     current_angles = self.get_joint_states()
                #     print(f"當前關節角度（度）: {np.degrees(current_angles)}")
            
            # 驗證最終位置
            end_state = p.getLinkState(self.robot_id, 5)
            return True  # Return the joint angles
        return False  # Return None if IK failed
    
    def cleanup(self):
        """清理資源"""
        p.disconnect()

    def get_joint_angles_degrees(self):
        """Returns the current joint angles in degrees."""
        joint_states = self.get_joint_states()
        joint_angles_degrees = np.degrees(joint_states)
        return joint_angles_degrees

# def main():
#     try:
#         # 創建控制器
#         controller = RobotIKSolver()
#         controller.set_all_joints_to_90_degrees()
#         for i in range(50): 
#             z = 0.2 + 0.1 * i
#             target_position = [0.1, 0.0, z]
#             success = controller.move_to_target(target_position)
#             if success is not None:
#                 print("\n移動到指定位置成功!")
#             else:
#                 print("\n移動到指定位置失敗!")
#         input("\n機器人已移動到指定角度，按 Enter 繼續...")
        
#         controller.cleanup()
        
#     except Exception as e:
#         print(f"\n發生錯誤: {str(e)}")
#         import traceback
#         traceback.print_exc()
#         p.disconnect()

# if __name__ == "__main__":
#     main()
