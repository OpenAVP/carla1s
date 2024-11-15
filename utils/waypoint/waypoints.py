import numpy as np
from typing import List, Union
from ...tf import Transform
import matplotlib.pyplot as plt
import matplotlib
matplotlib.use('Agg')

class Waypoints:
    def __init__(self, 
                 sequence: Union[List[Transform], np.ndarray, str],
                 *,
                 delta_seconds: float,
                 forward: bool = True,
                 keep_last: bool = False):
        self._delta_seconds = delta_seconds
        self._iter_index = 0
        self._path_length = 0
        self.keep_last = keep_last
        self.forward = forward
        self._threshold = 1
        self._target_dis = 0.2

        self._sequence = self._check_format(sequence)
        
    def __len__(self):
        return self._sequence.shape[0]-1
    
    def __getitem__(self, index: int):
        item = self._sequence[index]
        return Transform(x=item[0], y=item[1], z=item[2], pitch=item[3], yaw=item[4], roll=item[5])
    
    def __iter__(self):
        return self
    
    def __next__(self) -> Transform:
        """获取下一个waypoint

        Returns:
            waypoint (Transform): Transform类.
        """
        # 如果迭代器行进至终点, 且没有显示的声明 keep_last, 则抛出 StopIteration 异常以终止迭代
        if self._iter_index > len(self):
            if not self.keep_last:
                raise StopIteration
            else:
                self._iter_index = len(self)

        waypoint = self[self._iter_index]
        self._iter_index += 1
        return waypoint

    @property
    def data(self):
        return self._data

    def reset(self):
        self._iter_index = 0

    def next(self) -> Transform:
        return next(self)

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

        plt.plot(x,y,color='orange',marker='o',label='before',markersize=1,linewidth=0.1)
        plt.savefig("before.png")
        plt.plot(x_interpolated,y_interpolated,color='blue',marker='o',label='after',markersize=1,linewidth=0.1)
        plt.savefig("after.png")
        plt.xlabel("x")
        plt.ylabel("y")

        # 清除部分插值结果以适应Carla仿真器的yaw角表示法
        for i in range(1,new_len-1):
            if abs(yaw_interpolated[i] - yaw_interpolated[i-1]) > threshold and abs(yaw_interpolated[i] - yaw_interpolated[i+1]) > threshold :
                yaw_interpolated[i] = yaw_interpolated[i-1]

        return x_interpolated, y_interpolated, z_interpolated, pitch_interpolated, yaw_interpolated, roll_interpolated

    def _check_format(self, sequence: Union[List[Transform], np.ndarray]) -> np.ndarray:
        """检查路径点格式

        Args:
            sequence (np.ndarray/List[Transform]): 路径点序列.

        Returns:
            sequence (np.ndarray): 格式转换后的路径点序列.
        """
        if isinstance(sequence, list):
            transform_list = [[sequence[i].x, sequence[i].y, sequence[i].z, sequence[i].pitch, sequence[i].yaw, sequence[i].roll, 0, i * self._delta_seconds, 0, 0] for i in range(0,len(sequence))]
            sequence = np.array(transform_list)

        return sequence

    @classmethod
    def from_file(cls, 
                  file_path: str, 
                  *, 
                  delta_seconds: float, 
                  forward: bool = True, 
                  keep_last: bool = False) -> 'Waypoints':
        """从文件读取waypoints

        Args:
            file_path (str): 读取文件地址.
            delta_seconds (float): 路径点时间间隔.
            forward (bool): 路径点是否按照前进排列.
            keep_last (bool): 是否保留最后一个点.

        Returns:
            waypoints (Waypoints): 读取到的路径点序列.
        """
        transform, time_stamp = Waypoints.read_data(file_path)
        x, y, z, pitch, yaw, roll = Waypoints.interpolate(transform, time_stamp, delta_seconds)
        sequence = np.array([(x, y, 0, 0, yaw, 0) for x, y, yaw in zip(x, y, yaw)])
        waypoints = Waypoints(sequence = sequence, delta_seconds = delta_seconds)
        return waypoints
    

