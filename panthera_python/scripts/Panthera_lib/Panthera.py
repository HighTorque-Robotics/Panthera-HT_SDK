import time
import sys
import os
import yaml
import numpy as np
import pinocchio as pin

try:
    import hightorque_robot as htr
except ImportError as e:
    print(f"导入hightorque_robot失败: {e}")
    print("请确保已安装hightorque_robot whl包")
    print("安装方法: pip install hightorque_robot-*.whl")
    sys.exit(1)

class Panthera(htr.Robot):  # 继承自htr.Robot
    def __init__(self, config_path=None):
        # 如果未提供配置路径，使用默认的配置文件
        if config_path is None:
            script_dir = os.path.dirname(os.path.abspath(__file__))
            config_path = os.path.normpath(
                os.path.join(script_dir, "..", "..", "robot_param", "Follower.yaml")
            )

        # 加载总配置文件
        self.config = None
        self.model = None
        self.data = None
        self.joint_names = None
        self.joint_ids = []
        self.joint_limits = None
        self.gripper_limits = None
        self.velocity_limits = None

        # 读取总配置文件
        try:
            with open(config_path, 'r', encoding='utf-8') as f:
                self.config = yaml.safe_load(f)
                print(f"配置文件加载成功: {config_path}")
        except Exception as e:
            print(f"配置文件加载失败: {e}")
            sys.exit(1)

        # 读取关节限位
        try:
            if 'robot' in self.config and 'joint_limits' in self.config['robot']:
                self.joint_limits = {
                    'lower': np.array(self.config['robot']['joint_limits']['lower']),
                    'upper': np.array(self.config['robot']['joint_limits']['upper'])
                }
                print(f"关节限位加载成功: lower={self.joint_limits['lower']}, upper={self.joint_limits['upper']}")
            else:
                print("警告: 配置文件中未找到joint_limits")
        except Exception as e:
            print(f"关节限位加载失败: {e}")

        # 读取夹爪限位
        try:
            if 'robot' in self.config and 'gripper_limits' in self.config['robot']:
                self.gripper_limits = {
                    'lower': self.config['robot']['gripper_limits']['lower'],
                    'upper': self.config['robot']['gripper_limits']['upper']
                }
                print(f"夹爪限位加载成功: lower={self.gripper_limits['lower']}, upper={self.gripper_limits['upper']}")
            else:
                print("警告: 配置文件中未找到gripper_limits")
        except Exception as e:
            print(f"夹爪限位加载失败: {e}")

        # 保存配置文件目录，用于后续加载URDF等资源
        config_dir = os.path.dirname(os.path.abspath(config_path))
        self.config_dir = config_dir

        # 调用父类的初始化方法，传入配置文件路径
        # SDK会从配置文件中读取robot.param_file来加载电机参数
        super().__init__(config_path)

        # 获取电机列表（不要创建新的Robot实例）
        self.Motors = self.get_motors()

        #夹爪电机id
        self.gripper_id = len(self.Motors)
        print("初始化机械臂...")
        # 获取电机数量(不包含夹爪电机)
        self.motor_count = len(self.Motors) - 1
        print(f"发现 {self.motor_count} 个电机")
        if self.motor_count == 0:
            print("未发现电机。请检查您的配置和连接。")
            return
        # 打印电机信息
        for i, motor in enumerate(self.Motors):
            print(f"Motor {i}: ID={motor.get_motor_id()}, "
                f"Type={motor.get_motor_enum_type()}, "
                f"Name={motor.get_motor_name()}")

        # 读取最大力矩（在motor_count设置之后）
        try:
            if 'robot' in self.config and 'max_torque' in self.config['robot']:
                self.max_torque = np.array(self.config['robot']['max_torque'])
                print(f"最大力矩加载成功: {self.max_torque.tolist()}")
            else:
                print(f"警告: 配置文件中未找到max_torque，使用默认值[10.0] * {self.motor_count}")
                self.max_torque = np.array([10.0] * self.motor_count)
        except Exception as e:
            print(f"最大力矩加载失败: {e}，使用默认值[10.0] * {self.motor_count}")
            self.max_torque = np.array([10.0] * self.motor_count)

        # 读取速度限幅（在motor_count设置之后）
        try:
            if 'robot' in self.config and 'velocity_limits' in self.config['robot']:
                self.velocity_limits = np.array(self.config['robot']['velocity_limits'])
                print(f"速度限幅加载成功: {self.velocity_limits.tolist()}")
            else:
                print(f"警告: 配置文件中未找到velocity_limits，使用默认值[1.0] * {self.motor_count}")
                self.velocity_limits = np.array([1.0] * self.motor_count)
        except Exception as e:
            print(f"速度限幅加载失败: {e}，使用默认值[1.0] * {self.motor_count}")
            self.velocity_limits = np.array([1.0] * self.motor_count)

        # 加载URDF模型用于运动学计算
        self._load_urdf_model()

    def _load_urdf_model(self):
        """加载URDF模型用于运动学计算"""
        try:
            # 获取URDF文件路径（相对于配置文件的路径）
            urdf_relative_path = self.config['urdf']['file_path']

            # 计算URDF的绝对路径（相对于配置文件所在目录）
            config_dir = getattr(self, "config_dir", os.path.dirname(os.path.abspath(__file__)))
            urdf_path = os.path.normpath(os.path.join(config_dir, urdf_relative_path))

            # 使用pinocchio加载URDF
            self.model = pin.buildModelFromUrdf(urdf_path)
            self.data = self.model.createData()

            # 获取关节信息
            self.joint_names = self.config['kinematics']['joint_names']

            # 获取关节ID（跳过universe joint）
            for joint_name in self.joint_names:
                if self.model.existJointName(joint_name):
                    joint_id = self.model.getJointId(joint_name)
                    self.joint_ids.append(joint_id)
                else:
                    print(f"警告: 关节 {joint_name} 未在模型中找到")

            print(f"URDF加载成功: {urdf_path}")
            print(f"模型包含 {self.model.njoints - 1} 个关节（不含base）")
            print(f"配置关节数: {len(self.joint_ids)}")
        except Exception as e:
            print(f"URDF加载失败: {e}")

    def get_current_state(self):
        """获取当前关节状态"""
        state = []
        for i in range(self.motor_count):
            motor_state = self.Motors[i].get_current_motor_state()
            state.append(motor_state)
        return state

    def get_current_pos(self):
        """获取当前关节角度，返回np.ndarray"""
        joint_angles = np.zeros(self.motor_count)
        for i in range(self.motor_count):
            state = self.Motors[i].get_current_motor_state()
            joint_angles[i] = state.position
        return joint_angles

    def get_current_vel(self):
        """获取当前关节速度，返回np.ndarray"""
        joint_velocities = np.zeros(self.motor_count)
        for i in range(self.motor_count):
            state = self.Motors[i].get_current_motor_state()
            joint_velocities[i] = state.velocity
        return joint_velocities

    def get_current_torque(self):
        """获取当前关节力矩，返回np.ndarray"""
        joint_torques = np.zeros(self.motor_count)
        for i in range(self.motor_count):
            state = self.Motors[i].get_current_motor_state()
            joint_torques[i] = state.torque
        return joint_torques

    def get_current_state_gripper(self):
        """获取当前夹爪状态"""
        return self.Motors[self.gripper_id-1].get_current_motor_state()
    
    def get_current_pos_gripper(self):
        """获取当前夹爪位置"""
        state = self.Motors[self.gripper_id-1].get_current_motor_state()
        return state.position

    def get_current_vel_gripper(self):
        """获取当前夹爪速度"""
        state = self.Motors[self.gripper_id-1].get_current_motor_state()
        return state.velocity
    
    def get_current_torque_gripper(self):
        """获取当前夹爪力矩"""
        state = self.Motors[self.gripper_id-1].get_current_motor_state()
        return state.torque

    def Joint_Pos_Vel(self, pos, vel, max_tqu=None, iswait=False, tolerance=0.1, timeout=15.0):
        """
        单关节位置速度最大力矩控制（每个关节独立设置）

        参数:
            pos: 目标位置列表/数组 [joint1, joint2, ..., jointN]
            vel: 目标速度列表/数组 [joint1, joint2, ..., jointN]
            max_tqu: 最大力矩列表/数组，如果为None则使用配置文件中的默认值
            iswait: 是否等待运动完成
            tolerance: 位置容差（弧度）
            timeout: 等待超时时间（秒）

        返回:
            bool: 控制是否成功执行

        说明:
            每个关节独立设置位置和速度，适用于各关节需要不同运动速度的场景
        """
        # 如果未提供max_tqu，使用配置文件中的默认值
        if max_tqu is None:
            max_tqu = self.max_torque
        else:
            max_tqu = np.asarray(max_tqu)

        # 检查关节数量（除了夹爪电机）
        if not (len(pos) == len(vel) == len(max_tqu) == self.motor_count):
            raise ValueError(f"关节参数长度必须为{self.motor_count}")
        # 转换为numpy数组
        pos = np.asarray(pos)

        # 检查位置是否在限位范围内
        if self.joint_limits is not None:
            lower = self.joint_limits['lower']
            upper = self.joint_limits['upper']
            # 检查是否有位置超出限位
            out_of_range = np.logical_or(pos < lower, pos > upper)
            if np.any(out_of_range):
                print("\n" + "="*60)
                print("警告：检测到目标位置超出关节限位范围！")
                print(f"目标位置: {pos}")
                print(f"限位下限: {lower}")
                print(f"限位上限: {upper}")
                out_indices = np.where(out_of_range)[0]
                for idx in out_indices:
                    print(f"  关节{idx+1}: {pos[idx]:.3f} 不在 [{lower[idx]:.3f}, {upper[idx]:.3f}] 范围内")
                print("控制指令已被拒绝，保护机械臂安全")
                print("="*60 + "\n")
                return False

        # 控制关节（除了夹爪电机）
        for i in range(self.motor_count):
            motor = self.Motors[i]
            motor.pos_vel_MAXtqe(pos[i], vel[i], max_tqu[i])
        self.motor_send_cmd()
        if iswait:
            return self.wait_for_position(pos, tolerance, timeout)
        return True
    
    def Joint_Vel(self, vel):
        """
        关节速度控制

        参数:
            vel: 目标速度列表/数组 [joint1, joint2, ..., jointN] (rad/s)

        返回:
            bool: 控制是否成功执行

        说明:
            直接控制关节速度，不进行位置限位检查
            适用于需要精确速度控制的场景
            速度将被限制在配置文件设定的范围内
        """
        # 参数检查
        if len(vel) != self.motor_count:
            raise ValueError(f"目标速度长度必须为{self.motor_count}")

        # 转换为numpy数组
        vel = np.asarray(vel)

        # 速度限幅检查
        if self.velocity_limits is not None:
            # 检查是否有速度超出限幅
            abs_vel = np.abs(vel)
            out_of_limit = abs_vel > self.velocity_limits
            if np.any(out_of_limit):
                print("\n" + "="*60)
                print("警告：检测到目标速度超出限幅范围！")
                print(f"目标速度: {vel}")
                print(f"速度限幅: ±{self.velocity_limits}")
                out_indices = np.where(out_of_limit)[0]
                for idx in out_indices:
                    print(f"  关节{idx+1}: {vel[idx]:.3f} rad/s 超出限幅 ±{self.velocity_limits[idx]:.3f} rad/s")
                print("速度将被限制在安全范围内")
                print("="*60 + "\n")
                # 限幅处理
                vel = np.clip(vel, -self.velocity_limits, self.velocity_limits)

        # 控制关节（除了夹爪电机）
        for i in range(self.motor_count):
            motor = self.Motors[i]
            motor.velocity(vel[i])
        self.motor_send_cmd()
        return True

    def Joints_Sync_Arrival(self, pos, duration, max_tqu=None, iswait=False, tolerance=0.1, timeout=15.0):
        """
        多关节同步到达控制（所有关节在指定时间内同时到达目标位置）

        参数:
            pos: 目标位置列表/数组 [joint1, joint2, ..., jointN]
            duration: 运动时间（秒），所有关节将在该时间内同时到达目标位置
            max_tqu: 最大力矩列表/数组，如果为None则使用配置文件中的默认值
            iswait: 是否等待运动完成
            tolerance: 位置容差（弧度）
            timeout: 等待超时时间（秒）

        返回:
            bool: 控制是否成功执行

        说明:
            该函数通过 (目标位置 - 当前位置) / duration 计算每个关节的平均速度，
            确保所有关节在指定的时间内同时到达目标位置，适用于需要协调运动的场景
        """
        # 如果未提供max_tqu，使用配置文件中的默认值
        if max_tqu is None:
            max_tqu = self.max_torque
        else:
            max_tqu = np.asarray(max_tqu)

        # 参数检查
        if len(pos) != self.motor_count:
            raise ValueError(f"目标位置长度必须为{self.motor_count}")
        if len(max_tqu) != self.motor_count:
            raise ValueError(f"最大力矩长度必须为{self.motor_count}")
        if duration <= 0:
            raise ValueError(f"运动时间必须大于0，当前值: {duration}")

        # 转换为numpy数组
        pos = np.asarray(pos)

        # 检查位置是否在限位范围内
        if self.joint_limits is not None:
            lower = self.joint_limits['lower']
            upper = self.joint_limits['upper']
            out_of_range = np.logical_or(pos < lower, pos > upper)
            if np.any(out_of_range):
                print("\n" + "="*60)
                print("警告：检测到目标位置超出关节限位范围！")
                print(f"目标位置: {pos}")
                print(f"限位下限: {lower}")
                print(f"限位上限: {upper}")
                out_indices = np.where(out_of_range)[0]
                for idx in out_indices:
                    print(f"  关节{idx+1}: {pos[idx]:.3f} 不在 [{lower[idx]:.3f}, {upper[idx]:.3f}] 范围内")
                print("控制指令已被拒绝，保护机械臂安全")
                print("="*60 + "\n")
                return False

        # 获取当前位置
        current_pos = self.get_current_pos()

        # 计算速度: v = (目标位置 - 当前位置) / 时间
        # 这样可以确保所有关节在duration时间内同时到达目标位置
        vel = (pos - current_pos) / duration

        # 调用单关节位置速度控制
        return self.Joint_Pos_Vel(pos, vel, max_tqu, iswait, tolerance, timeout)

    def pos_vel_tqe_kp_kd(self, pos, vel, tqe, kp, kd):
        """关节五参数MIT控制模式"""
        # 检查关节数量（除了夹爪电机）
        params = [pos, vel, tqe, kp, kd]
        if not all(len(p) == self.motor_count for p in params):
            raise ValueError(f"关节参数长度必须为{self.motor_count}")

        # 转换为numpy数组
        pos = np.asarray(pos)

        # 检查位置是否在限位范围内
        if self.joint_limits is not None:
            lower = self.joint_limits['lower']
            upper = self.joint_limits['upper']
            # 检查是否有位置超出限位
            out_of_range = np.logical_or(pos < lower, pos > upper)
            if np.any(out_of_range):
                print("\n" + "="*60)
                print("警告：检测到目标位置超出关节限位范围！")
                print(f"目标位置: {pos}")
                print(f"限位下限: {lower}")
                print(f"限位上限: {upper}")
                out_indices = np.where(out_of_range)[0]
                for idx in out_indices:
                    print(f"  关节{idx+1}: {pos[idx]:.3f} 不在 [{lower[idx]:.3f}, {upper[idx]:.3f}] 范围内")
                print("控制指令已被拒绝，保护机械臂安全")
                print("="*60 + "\n")
                return False

        # 控制关节（除了夹爪电机）
        for i in range(self.motor_count):
            motor = self.Motors[i]
            motor.pos_vel_tqe_kp_kd(pos[i], vel[i], tqe[i], kp[i], kd[i])
        self.motor_send_cmd()
        return True

    def gripper_control(self, pos, vel, max_tqu=0.5):
        """夹爪控制（位置速度最大力矩模式）"""
        # 检查夹爪位置是否在限位范围内
        if self.gripper_limits is not None:
            lower = self.gripper_limits['lower']
            upper = self.gripper_limits['upper']
            # 检查位置是否超出限位
            if pos < lower or pos > upper:
                print("\n" + "="*60)
                print("警告：检测到夹爪目标位置超出限位范围！")
                print(f"目标位置: {pos}")
                print(f"限位下限: {lower}")
                print(f"限位上限: {upper}")
                print(f"夹爪位置 {pos:.3f} 不在 [{lower:.3f}, {upper:.3f}] 范围内")
                print("控制指令已被拒绝，保护夹爪安全")
                print("="*60 + "\n")
                return False

        self.Motors[self.gripper_id-1].pos_vel_MAXtqe(pos, vel, max_tqu)
        self.motor_send_cmd()
        return True

    def gripper_control_MIT(self, pos, vel, tqe, kp, kd):
        """夹爪控制（5参数MIT模式）"""
        # 检查夹爪位置是否在限位范围内
        if self.gripper_limits is not None:
            lower = self.gripper_limits['lower']
            upper = self.gripper_limits['upper']
            # 检查位置是否超出限位
            if pos < lower or pos > upper:
                print("\n" + "="*60)
                print("警告：检测到夹爪目标位置超出限位范围！")
                print(f"目标位置: {pos}")
                print(f"限位下限: {lower}")
                print(f"限位上限: {upper}")
                print(f"夹爪位置 {pos:.3f} 不在 [{lower:.3f}, {upper:.3f}] 范围内")
                print("控制指令已被拒绝，保护夹爪安全")
                print("="*60 + "\n")
                return False

        self.Motors[self.gripper_id-1].pos_vel_tqe_kp_kd(pos, vel, tqe, kp, kd)
        self.motor_send_cmd()
        return True
    
    def gripper_open(self, pos=1.6, vel=0.5, max_tqu=0.5):
        """打开夹爪"""
        self.gripper_control(pos, vel, max_tqu)

    def gripper_close(self, pos=0.0, vel=0.5, max_tqu=0.5):
        """关闭夹爪"""
        self.gripper_control(pos, vel, max_tqu)

    def check_position_reached(self, target_positions, tolerance=0.1):
        """检查前关节位置是否到达"""
        all_reached = True
        position_errors = []
        
        self.send_get_motor_state_cmd()
        self.motor_send_cmd()
        # 检查前6个关节
        for i in range(self.motor_count):
            state = self.Motors[i].get_current_motor_state()
            error = abs(state.position - target_positions[i])
            position_errors.append(error)
            if error > tolerance:
                all_reached = False
        
        return all_reached, position_errors
    
    def wait_for_position(self, target_positions, tolerance=0.01, timeout=15.0):
        """等待位置到达"""
        start_time = time.time()
        while (time.time() - start_time) < timeout:
            reached, _ = self.check_position_reached(target_positions, tolerance)
            if reached:
                return True
            time.sleep(0.02)
        return False


#######################运动学和动力学函数#######################
    def forward_kinematics(self, joint_angles=None):
        """使用pinocchio计算正运动学，返回末端位置和变换矩阵"""
        if self.model is None:
            print("模型未加载")
            return None

        # 如果未提供关节角度，获取当前角度
        if joint_angles is None:
            joint_angles = self.get_current_pos()
        
        # 创建关节配置向量
        q = np.zeros(self.model.nq)
        for i, joint_name in enumerate(self.joint_names):
            if i < len(joint_angles):
                joint_id = self.model.getJointId(joint_name)
                idx = self.model.joints[joint_id].idx_q
                q[idx] = joint_angles[i]
        
        # 计算正运动学
        pin.forwardKinematics(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)
        
        # 获取最后一个活动关节的变换矩阵
        last_joint_name = self.joint_names[-1]
        last_joint_id = self.model.getJointId(last_joint_name)
        last_joint_transform = self.data.oMi[last_joint_id]
        
        # 添加工具坐标系偏移：相对于最后一个关节在X轴方向偏移0.14m
        tool_offset = np.array([0.14, 0.0, 0.0])
        
        # 计算工具坐标系的位置和旋转
        position = last_joint_transform.translation + last_joint_transform.rotation.dot(tool_offset)
        rotation = last_joint_transform.rotation
        
        # 构建4x4变换矩阵
        T = np.eye(4)
        T[:3, :3] = rotation
        T[:3, 3] = position
        
        return {
            'position': position.tolist(),
            'rotation': rotation,
            'transform': T,
            'joint_angles': joint_angles
        }
    
    def inverse_kinematics(self, target_position, target_rotation=None, init_q=None, max_iter=1000, eps=1e-3):
        """使用pinocchio计算逆运动学"""
        if self.model is None:
            return None

        # 目标位姿
        if target_rotation is None:
            target_rotation = np.eye(3)

        # 将工具坐标系目标转换为最后一个关节坐标系目标
        # 减去工具偏移：相对于最后一个关节在X轴方向偏移0.14m
        tool_offset = np.array([0.14, 0.0, 0.0])
        target_rotation_matrix = np.array(target_rotation)

        # 计算最后关节的目标位置（减去偏移）
        last_joint_target_position = np.array(target_position) - target_rotation_matrix.dot(tool_offset)

        oMdes = pin.SE3(target_rotation_matrix, last_joint_target_position)

        # 初始关节角度
        if init_q is None:
            init_q = self.get_current_pos()

        q = np.zeros(self.model.nq)
        for i, joint_name in enumerate(self.joint_names):
            if i < len(init_q):
                joint_id = self.model.getJointId(joint_name)
                idx = self.model.joints[joint_id].idx_q
                q[idx] = init_q[i]

        # 获取最后一个活动关节ID
        last_joint_name = self.joint_names[-1]
        joint_id = self.model.getJointId(last_joint_name)

        # 迭代求解
        dt = 1e-1
        damp = 1e-12

        # 获取关节限位（如果有的话）
        lower_limits = None
        upper_limits = None
        if self.joint_limits is not None:
            lower_limits = self.joint_limits['lower']
            upper_limits = self.joint_limits['upper']

        for i in range(max_iter):
            # 计算误差
            pin.forwardKinematics(self.model, self.data, q)
            iMd = self.data.oMi[joint_id].actInv(oMdes)
            err = pin.log(iMd).vector

            if np.linalg.norm(err) < eps:
                # 提取关节角度
                result = []
                for joint_name in self.joint_names:
                    jid = self.model.getJointId(joint_name)
                    idx = self.model.joints[jid].idx_q
                    result.append(q[idx])
                return result

            # 计算雅可比和速度
            J = pin.computeJointJacobian(self.model, self.data, q, joint_id)
            J = -np.dot(pin.Jlog6(iMd.inverse()), J)
            v = -J.T.dot(np.linalg.solve(J.dot(J.T) + damp * np.eye(6), err))

            # 限制速度大小，防止数值爆炸
            v_norm = np.linalg.norm(v)
            if v_norm > 10.0:  # 如果速度过大，进行缩放
                v = v * (10.0 / v_norm)

            q_new = pin.integrate(self.model, q, v * dt)

            # 检查新的关节角度是否在限位范围内
            if lower_limits is not None and upper_limits is not None:
                # 提取关节角度进行检查
                q_check = []
                for joint_name in self.joint_names:
                    jid = self.model.getJointId(joint_name)
                    idx = self.model.joints[jid].idx_q
                    q_check.append(q_new[idx])
                q_check = np.array(q_check)

                # 检查是否超出限位
                out_of_range = np.logical_or(q_check < lower_limits, q_check > upper_limits)
                if np.any(out_of_range):
                    print("逆解迭代过程中检测到关节角度超出限位，目标位姿可能不可达")
                    print(f"当前迭代: {i+1}/{max_iter}, 误差范数: {np.linalg.norm(err):.6f}")
                    return None

            q = q_new

        print("该解未收敛，请检查是否超出工作空间")
        return None  # 未收敛

    def get_Gravity(self, q=None):
        """获取重力补偿力矩 G(q)，返回np.ndarray"""
        if q is None:
            q = self.get_current_pos()
        # 确保为numpy数组（如果已是数组则不复制）
        q = np.asarray(q)
        # 计算重力补偿
        G = pin.computeGeneralizedGravity(self.model, self.data, q)
        return G

    def get_Coriolis(self, q=None, v=None):
        """获取科氏力矩阵 C(q,v)，返回np.ndarray"""
        if q is None:
            q = self.get_current_pos()
        if v is None:
            v = self.get_current_vel()
        # 确保为numpy数组
        q = np.asarray(q)
        v = np.asarray(v)
        # 计算科氏力矩阵
        C = pin.computeCoriolisMatrix(self.model, self.data, q, v)
        return C

    def get_Coriolis_vector(self, q=None, v=None):
        """获取科氏力向量 C(q,v)*v（向后兼容），返回np.ndarray"""
        C = self.get_Coriolis(q, v)
        if v is None:
            v = self.get_current_vel()
        else:
            v = np.asarray(v)
        return C.dot(v)

    def get_Mass_Matrix(self, q=None):
        """获取完整的质量矩阵，返回np.ndarray"""
        if q is None:
            q = self.get_current_pos()
        # 确保为numpy数组
        q = np.asarray(q)
        # 计算质量矩阵
        M = pin.crba(self.model, self.data, q)
        # 返回完整的质量矩阵
        return M[:len(q), :len(q)]

    def get_Inertia_Terms(self, q=None, a=None):
        """获取惯性力矩 M(q)*a，返回np.ndarray"""
        if q is None:
            q = self.get_current_pos()
        if a is None:
            a = np.zeros(self.motor_count)
        # 确保为numpy数组
        q = np.asarray(q)
        a = np.asarray(a)
        # 计算质量矩阵
        M = pin.crba(self.model, self.data, q)
        # 计算惯性力矩 M*a
        inertia_torque = M[:len(q), :len(q)].dot(a)
        return inertia_torque

    def get_Dynamics(self, q=None, v=None, a=None):
        """获取完整动力学 tau = M(q)*a + C(q,v)*v + G(q)，返回np.ndarray"""
        if q is None:
            q = self.get_current_pos()
        if v is None:
            v = self.get_current_vel()
        if a is None:
            a = np.zeros(self.model.nv)
        # 确保为numpy数组
        q = np.asarray(q)
        v = np.asarray(v)
        a = np.asarray(a)
        # 计算完整动力学
        tau = pin.rnea(self.model, self.data, q, v, a)
        return tau
    
    def get_friction_compensation(self, vel=None, Fc=None, Fv=None, vel_threshold=0.01):
        """
        计算摩擦力补偿力矩（库伦摩擦 + 粘性摩擦模型），返回np.ndarray
        参数:
            vel: 关节速度数组 [6,] (rad/s)，如果为None则使用当前速度
            Fc: 库伦摩擦系数数组 [6,] (Nm) - 恒定摩擦力
            Fv: 粘性摩擦系数数组 [6,] (Nm·s/rad) - 速度相关摩擦系数
            vel_threshold: 速度阈值 (rad/s)，低于此值使用特殊处理避免抖动
        返回:
            tau_friction: 摩擦力补偿力矩数组 np.ndarray [6,] (Nm)
        摩擦模型:
            τ_friction = Fc * sign(vel) + Fv * vel
            当 |vel| < vel_threshold 时，只使用粘性摩擦项避免符号跳变
        """
        # 获取速度
        if vel is None:
            vel = self.get_current_vel()
        else:
            vel = np.asarray(vel)

        # 确保摩擦系数为numpy数组
        Fc = np.asarray(Fc)
        Fv = np.asarray(Fv)

        # 向量化计算摩擦力补偿
        # 计算完整的摩擦模型（库伦 + 粘性）
        full_friction = Fc * np.sign(vel) + Fv * vel

        # 低速区只使用粘性摩擦
        low_speed_friction = Fv * vel

        # 使用条件选择：|vel| < threshold 时用低速模型，否则用完整模型
        tau_friction = np.where(np.abs(vel) < vel_threshold, low_speed_friction, full_friction)

        return tau_friction
    
    def septic_interpolation(self, start_pos, end_pos, duration, current_time):
        """七次多项式插值轨迹生成（速度、加速度、加加速度连续），返回np.ndarray"""
        # 转换为numpy数组
        start_pos = np.asarray(start_pos)
        end_pos = np.asarray(end_pos)

        if current_time <= 0:
            return start_pos, np.zeros_like(start_pos), np.zeros_like(start_pos)
        if current_time >= duration:
            return end_pos, np.zeros_like(end_pos), np.zeros_like(end_pos)

        # 归一化时间
        t = current_time / duration
        t2 = t * t
        t3 = t2 * t
        t4 = t3 * t
        t5 = t4 * t
        t6 = t5 * t
        t7 = t6 * t

        # 七次多项式系数 (位置)
        a0 = 1 - 35*t4 + 84*t5 - 70*t6 + 20*t7
        a1 = 35*t4 - 84*t5 + 70*t6 - 20*t7

        # 一阶导数系数 (速度)
        da0 = -140*t3 + 420*t4 - 420*t5 + 140*t6
        da1 = 140*t3 - 420*t4 + 420*t5 - 140*t6

        # 二阶导数系数 (加速度)
        dda0 = -420*t2 + 1680*t3 - 2100*t4 + 840*t5
        dda1 = 420*t2 - 1680*t3 + 2100*t4 - 840*t5

        # 向量化计算位置、速度、加速度
        pos = a0 * start_pos + a1 * end_pos
        vel = (da0 * start_pos + da1 * end_pos) / duration
        acc = (dda0 * start_pos + dda1 * end_pos) / (duration * duration)

        return pos, vel, acc
    
    def septic_interpolation_with_velocity(self, start_pos, end_pos, start_vel, end_vel, duration, current_time):
        """
        七次多项式插值轨迹生成（指定起始和终止速度），返回np.ndarray
        可以实现非零速度的平滑过渡
        """
        # 转换为numpy数组
        start_pos = np.asarray(start_pos)
        end_pos = np.asarray(end_pos)
        start_vel = np.asarray(start_vel)
        end_vel = np.asarray(end_vel)

        if current_time <= 0:
            return start_pos, start_vel, np.zeros_like(start_pos)
        if current_time >= duration:
            return end_pos, end_vel, np.zeros_like(end_pos)

        # 归一化时间
        t = current_time / duration
        t2 = t * t
        t3 = t2 * t
        t4 = t3 * t
        t5 = t4 * t
        t6 = t5 * t
        t7 = t6 * t

        # 七次多项式系数（考虑速度边界条件）
        # p(t) = a0 + a1*t + a2*t^2 + a3*t^3 + a4*t^4 + a5*t^5 + a6*t^6 + a7*t^7
        # 边界条件：p(0)=p0, p(1)=p1, v(0)=v0, v(1)=v1, a(0)=0, a(1)=0, j(0)=0, j(1)=0

        p0 = start_pos
        p1 = end_pos
        v0 = start_vel * duration  # 转换为归一化速度
        v1 = end_vel * duration

        # 向量化系数计算（满足8个边界条件）
        a0 = p0
        a1 = v0
        a2 = np.zeros_like(p0)  # 起始加速度为0
        a3 = np.zeros_like(p0)  # 起始加加速度为0

        # 通过矩阵求解得到的系数
        a4 = 35*(p1 - p0) - 20*v0 - 15*v1
        a5 = -84*(p1 - p0) + 45*v0 + 39*v1
        a6 = 70*(p1 - p0) - 36*v0 - 34*v1
        a7 = -20*(p1 - p0) + 10*v0 + 10*v1

        # 向量化计算位置
        pos = a0 + a1*t + a2*t2 + a3*t3 + a4*t4 + a5*t5 + a6*t6 + a7*t7

        # 向量化计算速度（一阶导数）
        vel = (a1 + 2*a2*t + 3*a3*t2 + 4*a4*t3 + 5*a5*t4 + 6*a6*t5 + 7*a7*t6) / duration

        # 向量化计算加速度（二阶导数）
        acc = (2*a2 + 6*a3*t + 12*a4*t2 + 20*a5*t3 + 30*a6*t4 + 42*a7*t5) / (duration * duration)

        return pos, vel, acc

    def _precise_sleep(self, duration):
        """
        高精度延时函数（内部使用）

        参数:
            duration: 延时时间（秒）
        """
        if duration <= 0:
            return

        end_time = time.perf_counter() + duration

        # 大部分时间用sleep（留1ms余量）
        if duration > 0.001:
            time.sleep(duration - 0.001)

        # 最后用忙等待保证精度
        while time.perf_counter() < end_time:
            pass

if __name__ == "__main__":
    robot = Panthera()