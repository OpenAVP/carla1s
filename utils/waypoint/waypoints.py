import numpy as np
import math
from typing import List, Union
from .follow_waypoint_control import calculate_yaws
from .get_tf import interpolate, read_data
from .PurePursuit import SimpleTest
from ...tf import Transform

class Waypoints:
    def __init__(self, 
                 sequence: Union[List[Transform], np.ndarray],
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
        self._ori_yaw = 0
        self._sequence = self._smoothing(sequence)
        
    def __len__(self):
        # TODO: CONFIRM IT!
        if isinstance(self._sequence, list):
            return len(self._sequence)
        elif isinstance(self._sequence, np.array):
            return self._sequence.shape[0]
    
    def __getitem__(self, index: int):
        # TODO: CONFIRM IT!

        #return self._sequence[index]
        item = self._sequence[index]
        if isinstance(item, Transform):
            return item
        elif isinstance(item, np.array):
            print("type(item):",type(item))
            return Transform(x=item[0], y=item[1], z=item[2], pitch=item[3], yaw=item[4], roll=item[5])
    
    def __iter__(self):
        return self
    
    def __next__(self) -> Transform:
        # 如果迭代器行进至终点, 且没有显示的声明 keep_last, 则抛出 StopIteration 异常以终止迭代
        if self._iter_index > len(self):
            if not self.keep_last:
                raise StopIteration
            else:
                self._iter_index = len(self)

        waypoint = self._sequence[self._iter_index]
        self._iter_index += 1

        if isinstance(waypoint, Transform):
            return waypoint
        elif isinstance(waypoint, np.array):
            waypoint = Transform(x=waypoint[0], y=waypoint[1], z=waypoint[2], pitch=waypoint[3], yaw=waypoint[4], roll=waypoint[5])
            return waypoint

    @property
    def data(self):
        return self._data

    def reset(self):
        self._iter_index = 0

    def next(self) -> Transform:
        return next(self)

    def _smoothing(self, sequence: np.ndarray) -> np.ndarray:
        if isinstance(sequence, list):
            transform_list = [[sequence[i].x, sequence[i].y, sequence[i].z, sequence[i].pitch, sequence[i].yaw, sequence[i].roll, 0, i * self._delta_seconds, 0, 0] for i in range(0,len(sequence))]
            sequence = np.array(transform_list)
            sequence = self._up_sampling(sequence)
            sequence = self._pure_pursuit(sequence)
        elif isinstance(sequence, np.array):

            # 输入的sequence格式：x y z pitch yaw roll frame elapsed_seconds delta_seconds platform_timestamp
            sequence = self._up_sampling(sequence)
            sequence = self._pure_pursuit(sequence)
        return sequence
    
    def _up_sampling(self, sequence: np.ndarray, save = False) -> np.ndarray:
        transform = sequence[:,:6]
        time_stamp = sequence[:,-4:]
        temp_x, temp_y, z, pitch, yaw, roll = interpolate(transform, time_stamp, self._delta_seconds)

        # 考虑到参考路径点可能过于稀疏，进行上采样，以方便PurePursuit算法追踪
        path_length = 0
        x_ = []
        y_ = []
        for i in range(0,len(temp_x)-1):
            dis = math.sqrt((temp_x[i]-temp_x[i+1])*(temp_x[i]-temp_x[i+1])+(temp_y[i]-temp_y[i+1])*(temp_y[i]-temp_y[i+1]))
            path_length += dis
            if dis > self._threshold:
                target_dis = self._target_dis
                target_points_num = int(dis / target_dis) + 1

                xp = np.array([temp_x[i],temp_x[i+1]])
                yp = np.array([temp_y[i],temp_y[i+1]])

                xn = np.linspace(temp_x[i], temp_x[i+1], target_points_num)
                yn = np.interp(xn, xp, yp)

                x_.extend(xn.tolist())
                y_.extend(yn.tolist())
            else:
                x_.append(temp_x[i])
                y_.append(temp_y[i])

        x = np.array(x_)
        y = np.array(y_)
        self._path_length = path_length

        yaw = calculate_yaws(x,y)
        self._ori_yaw = yaw[0]
        if save == True:
            self._sequence = [Transform(x=x, y=y, z=0, pitch=0, yaw=yaw, roll=0) for x, y, yaw in zip(x, y, yaw)]
        return np.concatenate((x.reshape(-1,1), y.reshape(-1,1)), axis = 1)
    
    def _pure_pursuit(self, sequence: np.ndarray, forward = True, save = False) -> np.ndarray:
        # 使用PurePursuit算法修正轨迹

        # 前进把速度设成1，倒车把速度设成-1
        if forward == True:
            v = 1
        else:
            v = -1
        state = np.array([sequence[0][0], sequence[0][1], np.radians(self._ori_yaw), v]).astype(np.float64)
        test = SimpleTest(initial_state = state)
        new_path = test.offline_test(sequence, math.floor(self._path_length/(v*self._delta_seconds))-1)

        x = [state[0] for state in new_path]
        y = [state[1] for state in new_path]
        yaw = [np.degrees(state[2]) for state in new_path]
        new_len = len(new_path)

        sequence = [Transform(x=x, y=y, z=0, pitch=0, yaw=yaw, roll=0) for x, y, yaw in zip(x, y, yaw)]
        if save == True:
            self._sequence = sequence
        return sequence
        # raise NotImplementedError

    @classmethod
    def from_file(cls, 
                  file_path: str, 
                  *, 
                  delta_seconds: float, 
                  forward: bool = True, 
                  keep_last: bool = False) -> 'Waypoints':
        transform, time_stamp = read_data(file_path)
        x, y, z, pitch, yaw, roll = interpolate(transform, time_stamp, delta_seconds)
        new_len = x.shape[0]
        sequence = [Transform(x=x, y=y, z=z, pitch=pitch, yaw=yaw, roll=roll) for x, y, z, pitch, yaw, roll in zip(x, y, z, pitch, yaw, roll)]
        waypoints = Waypoints(sequence = sequence, delta_seconds = delta_seconds)
        return waypoints
        # raise NotImplementedError