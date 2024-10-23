import carla
import numpy as np
from typing import Optional


class Transform:

    def __init__(self,
                 x: float = 0.0,
                 y: float = 0.0,
                 z: float = 0.0,
                 *,
                 yaw: float = 0.0,
                 pitch: float = 0.0,
                 roll: float = 0.0,
                 matrix: Optional[np.ndarray] = None):
        """变换的表示, 由位置和旋转组成.

        左手坐标系, 与 CARLA 和 UnrealEngine 保持一致.

        Args:
            x (float, optional): 表示物体在一个三维欧几里得空间中的 X 轴位置, 单位米, 右为正. 默认为 0.0.
            y (float, optional): 表示物体在一个三维欧几里得空间中的 Y 轴位置, 单位米, 后为正. 默认为 0.0.
            z (float, optional): 表示物体在一个三维欧几里得空间中的 Z 轴位置, 单位米, 上为正. 默认为 0.0.
            yaw (float, optional): 表示物体绕 Z 轴旋转的欧拉角, 单位度. 默认为 0.0.
            pitch (float, optional): 表示物体绕 Y 轴旋转的欧拉角, 单位度. 默认为 0.0.
            roll (float, optional): 表示物体绕 X 轴旋转的欧拉角, 单位度. 默认为 0.0.
            matrix (Optional[np.ndarray], optional): 变换矩阵, 4x4 的齐次变换矩阵, 如果提供了该参数, 则忽略其他参数. 默认为 None.

        Raises:
            ValueError: 如果提供的矩阵不是 4x4 的
        """
        # 如果提供了变换矩阵, 则直接使用
        if matrix is not None:
            if matrix.shape != (4, 4):
                raise ValueError("The matrix must be a 4x4 matrix.")
            self._matrix = matrix
            return

        # 将角度转换为弧度
        yaw = np.deg2rad(yaw)
        pitch = np.deg2rad(pitch)
        roll = np.deg2rad(roll)

        # 计算每个轴的旋转矩阵
        R_yaw = np.array([[np.cos(yaw), -np.sin(yaw), 0],
                          [np.sin(yaw), np.cos(yaw), 0],
                          [0, 0, 1]])
        R_pitch = np.array([[np.cos(pitch), 0, np.sin(pitch)],
                            [0, 1, 0],
                            [-np.sin(pitch), 0, np.cos(pitch)]])
        R_roll = np.array([[1, 0, 0],
                           [0, np.cos(roll), -np.sin(roll)],
                           [0, np.sin(roll), np.cos(roll)]])

        # 组合旋转矩阵 roll -> pitch -> yaw
        R = R_yaw @ R_pitch @ R_roll

        # 构建齐次变换矩阵
        self._matrix = np.eye(4)
        self._matrix[:3, :3] = R
        self._matrix[:3, 3] = [x, y, z]

    @property
    def matrix(self) -> np.ndarray:
        """
        :return: 变换矩阵, 4x4 的齐次变换矩阵
        """
        return self._matrix

    @matrix.setter
    def matrix(self, value: np.ndarray) -> None:
        """
        :param value: 新变换矩阵, 4x4 的齐次变换矩阵
        """
        self._matrix = value

    @property
    def x(self) -> float:
        """表示物体在其所在坐标系下的 X 轴上的位置, 单位米"""
        return self.matrix[0, 3].item()

    @property
    def y(self) -> float:
        """表示物体在其所在坐标系下的 X 轴上的位置, 单位米"""
        return self.matrix[1, 3].item()

    @property
    def z(self) -> float:
        """表示物体在其所在坐标系下的 X 轴上的位置, 单位米"""
        return self.matrix[2, 3].item()

    @property
    def yaw(self) -> float:
        """物体绕 Z 轴旋转的欧拉角, 单位度, 由变换矩阵计算得到"""
        return np.rad2deg(np.arctan2(self.matrix[1, 0], self.matrix[0, 0])).item()

    @property
    def pitch(self) -> float:
        """物体绕 Y 轴旋转的欧拉角, 单位度, 由变换矩阵计算得到"""
        return np.rad2deg(np.arcsin(-self.matrix[2, 0])).item()

    @property
    def roll(self) -> float:
        """物体绕 X 轴旋转的欧拉角, 单位度, 由变换矩阵计算得到"""
        return np.rad2deg(np.arctan2(self.matrix[2, 1], self.matrix[2, 2])).item()

    @property
    def quaternion(self) -> np.ndarray:
        """物体的四元数表示, 4x1 的矩阵"""
        m = self.matrix[:3, :3]
        trace = np.trace(m)
        if trace > 0:
            s = 0.5 / np.sqrt(trace + 1.0)
            qw = 0.25 / s
            qx = (m[2, 1] - m[1, 2]) * s
            qy = (m[0, 2] - m[2, 0]) * s
            qz = (m[1, 0] - m[0, 1]) * s
        else:
            if m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
                s = 2.0 * np.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2])
                qw = (m[2, 1] - m[1, 2]) / s
                qx = 0.25 * s
                qy = (m[0, 1] + m[1, 0]) / s
                qz = (m[0, 2] + m[2, 0]) / s
            elif m[1, 1] > m[2, 2]:
                s = 2.0 * np.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2])
                qw = (m[0, 2] - m[2, 0]) / s
                qx = (m[0, 1] + m[1, 0]) / s
                qy = 0.25 * s
                qz = (m[1, 2] + m[2, 1]) / s
            else:
                s = 2.0 * np.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1])
                qw = (m[1, 0] - m[0, 1]) / s
                qx = (m[0, 2] + m[2, 0]) / s
                qy = (m[1, 2] + m[2, 1]) / s
                qz = 0.25 * s

        return np.array([qw, qx, qy, qz])

    @classmethod
    def from_carla_transform_obj(cls, transform: carla.Transform) -> 'Transform':
        """从 CARLA 的变换对象创建 Transform 实例.

        Args:
            transform: CARLA 的变换对象

        Returns:
            Transform: 变换对象
        """
        return cls(x=transform.location.x,
                   y=transform.location.y,
                   z=transform.location.z,
                   yaw=transform.rotation.yaw,
                   pitch=transform.rotation.pitch,
                   roll=transform.rotation.roll)
    
    def as_carla_transform_obj(self) -> carla.Transform:
        """将当前 Transform 实例转换为 CARLA 的变换对象.

        Returns:
            carla.Transform: CARLA 的变换对象
        """
        return carla.Transform(location=carla.Location(x=self.x, y=self.y, z=self.z),
                               rotation=carla.Rotation(yaw=self.yaw, pitch=self.pitch, roll=self.roll))

    def __str__(self) -> str:
        return f'Transform(x={self.x:.2f}, y={self.y:.2f}, z={self.z:.2f}, yaw={self.yaw:.1f}, pitch={self.pitch:.1f}, roll={self.roll:.1f})'

    def __repr__(self) -> str:
        return self.__str__()
