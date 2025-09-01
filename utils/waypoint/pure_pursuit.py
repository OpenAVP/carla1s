import numpy as np
from matplotlib import pyplot as plt
import copy

# 角度使用弧度制

class PurePursuit:
    def __init__(self, lam, c, L):
        self.lam = lam
        self.c = c
        self.L = L

    def one_step(self, ref_path: np.ndarray, ego_state: np.ndarray) -> np.ndarray:
        """使用PurePursuit算法更新一步车辆状态

        Args:
            ref_path (np.ndarray): 参考路径(x, y).
            ego_state (np.ndarray): 当前自车状态(x, y, yaw, v).

        Returns:
            delta (float): 弧度制表示的转向角度.
        """
        ## 计算预瞄距离
        ld = self.lam * ego_state[3] + self.c

        ## 寻找预瞄点
        dis = np.linalg.norm(ref_path - ego_state[:2], 2, axis=1)
        start_idx = np.argmin(dis)
        dis = ld - dis

        ## 给距离大于ld和之前的点加mask
        dis[dis < 0] = float('inf')  
        dis[:start_idx] = float('inf')

        ref_point = ref_path[np.argmin(dis)]

        ## 计算控制指令
        alpha = np.arctan2(ref_point[1] - ego_state[1], ref_point[0] - ego_state[0]) - ego_state[2]
        delta = np.arctan2(2 * self.L * np.sin(alpha), ld)
        return delta

class SimpleTest:
    class CarModel:
        def __init__(self, L, dt, initial_state=np.array([0, 0, 0, 1]).astype(np.float64)):

            # state: x, y, yaw, v
            self._state = initial_state 
            self._L = L
            self._dt = dt

        def update_state(self, accel, delta):
            """更新车辆状态

            Args:
                accel (float): 加速度.
                delta (float): 转向角.
            """
            self._state[0] = self._state[0] + self._state[3] * np.cos(self._state[2]) * self._dt
            self._state[1] = self._state[1] + self._state[3] * np.sin(self._state[2]) * self._dt
            self._state[2] = self._state[2] + self._state[3] / self._L * np.tan(delta) * self._dt
            self._state[3] = self._state[3] + accel * self._dt

    def __init__(self, 
                 initial_state = np.array([0, 0, -0.5 * np.pi, 1]).astype(np.float64),
                 lam = 0.2, 
                 c = 2, 
                 L = 4.524, 
                 H = 2.07642,
                 dt = 0.1):
        """PurePursuit基础参数设置

        Args:
            lam (float): 预瞄系数.
            L (float): 车长.
            H (float): 车高.
            c (float): 预瞄常数.
            dt (float): 时间间隔.
        """
        self._H = H
        self._dt = dt
        self._threshold = 10
        self._model = SimpleTest.CarModel(L, dt, initial_state)
        self._pure_pursuit = PurePursuit(lam, c, L)

    def get_car_height(self):
        return self._H
    
    def get_dt(self):
        return self._dt

    def offline_test(self, path, test_cnt):
        """进行离线测试,根据参考路径输出行使轨迹

        Args:
            path (np.ndarray): 参考路径(x, y).
            test_cnt (int): 生成的车辆行驶轨迹状态数量

        Returns:
            states (list[np.array]): 车辆状态集合,每一个状态表示为(x, y, yaw, v).
        """
        states = []
        for i in range(test_cnt):
            delta = self._pure_pursuit.one_step(path, self._model._state)
            self._model.update_state(0, delta)
            new_state = copy.deepcopy(self._model._state)
            states.append(new_state)

        return states

    @staticmethod
    def simple_read(filename):
        with open(filename, 'r') as f:
            lines = f.readlines()
        path = []
        for l in lines:
            str_array = l.split('(')[1].split(')')[0].split(',')
            p = np.array([float(str_array[0]), float(str_array[1])])
            path.append(p)
        return np.array(path)
    
    def get_path(x, y):
        """将x,y路径分量拼接成路径

        Args:
            x (np.ndarray): 路径x坐标.
            y (np.ndarray): 路径y坐标.

        Returns:
            (np.ndarray): 路径(x, y)坐标.
        """
        return np.concatenate((x.reshape(-1,1),y.reshape(-1,1)),axis=1)

    @classmethod
    def read_data(cls, read_path):
        """读取npy文件保存的车辆Transform和时间戳信息

        Args:
            read_path (str): .npy文件路径
        Returns:
            transform (np.ndarray): 车辆Transform信息,格式为(x, y, z, pitch, yaw, roll).
            time_stamp (np.ndarray): 车辆时间戳信息,格式为(frame, elapsed_seconds, delta_seconds, platform_timestamp).
        """
        data = np.load(read_path, allow_pickle=True)
        data = data.reshape(-1, 10)

        # 拆分Transform和时间戳
        transform = data[:,:6]
        time_stamp = data[:,-4:]

        return transform, time_stamp

    @classmethod
    def interpolate(cls, transform, time_stamp, dT, threshold = 10):
        """根据给定的时间间隔插值读取车辆Transform信息

        Args:
            transform (np.ndarray): 车辆Transform信息,格式为(x, y, z, pitch, yaw, roll).
            time_stamp (np.ndarray): 车辆时间戳信息,格式为(frame, elapsed_seconds, delta_seconds, platform_timestamp).
            dT (float): 读取数据时间间隔

        Returns:
            x_interpolated (np.ndarray): 路径x坐标.
            y_interpolated (np.ndarray): 路径y坐标.
            z_interpolated (np.ndarray): 路径z坐标.
            pitch_interpolated (np.ndarray): 路径pitch角.
            yaw_interpolated (np.ndarray): 路径yaw角.
            roll_interpolated (np.ndarray): 路径roll角.
        """

        time_stamp = time_stamp[:,1]
        time_stamp[:] -= time_stamp[0]

        len = transform.shape[0]
        time_end = time_stamp[len-1]

        # 按照生成新的时间戳
        new_len = int(time_end / dT)
        new_time_stamp = np.arange(0, new_len * dT, dT)

        x = transform[:,0]
        y = transform[:,1]
        z = transform[:,2]
        pitch = transform[:,3]
        yaw = transform[:,4]
        roll = transform[:,5]

        # 插值平滑
        x_interpolated = np.interp(new_time_stamp, time_stamp, x)
        y_interpolated = np.interp(new_time_stamp, time_stamp, y)
        z_interpolated = np.interp(new_time_stamp, time_stamp, z) - 0.05
        pitch_interpolated = np.interp(new_time_stamp, time_stamp, pitch)
        yaw_interpolated = np.interp(new_time_stamp, time_stamp, yaw)
        roll_interpolated = np.interp(new_time_stamp, time_stamp, roll)

        # 清除部分插值结果以适应Carla仿真器的yaw角表示法
        for i in range(1,new_len-1):
            if abs(yaw_interpolated[i] - yaw_interpolated[i-1]) > threshold and abs(yaw_interpolated[i] - yaw_interpolated[i+1]) > threshold :
                yaw_interpolated[i] = yaw_interpolated[i-1]

        return x_interpolated, y_interpolated, z_interpolated, pitch_interpolated, yaw_interpolated, roll_interpolated
