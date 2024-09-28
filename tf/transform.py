import numpy as np


class Transform:

    def __init__(self,
                 x: float = 0.0,
                 y: float = 0.0,
                 z: float = 0.0,
                 *,
                 yaw: float = 0.0,
                 pitch: float = 0.0,
                 roll: float = 0.0):
        """
        变换的表示, 由位置和旋转组成.

        左手坐标系, 与 CARLA 和 UnrealEngine 保持一致.

        :param x: 表示物体在一个三维欧几里得空间中的 X 轴位置, 单位米, 右为正
        :param y: 表示物体在一个三维欧几里得空间中的 Y 轴位置, 单位米, 后为正
        :param z: 表示物体在一个三维欧几里得空间中的 Z 轴位置, 单位米, 上为正
        :param yaw: 表示物体绕 Z 轴旋转的角度, 单位度, 角度制
        :param pitch: 表示物体绕 Y 轴旋转的角度, 单位度, 角度制
        :param roll: 表示物体绕 X 轴旋转的角度, 单位度, 角度制

        """
        # 将角度转换为弧度
        yaw = np.deg2rad(yaw)
        pitch = np.deg2rad(pitch)
        roll = np.deg2rad(roll)

        # 计算每个轴的旋转矩阵
        # TODO: AI GENERATED CODE, VERIFY IT!
        R_yaw = np.array([[np.cos(yaw), -np.sin(yaw), 0],
                          [np.sin(yaw), np.cos(yaw), 0],
                          [0, 0, 1]])
        R_pitch = np.array([[np.cos(pitch), 0, np.sin(pitch)],
                            [0, 1, 0],
                            [-np.sin(pitch), 0, np.cos(pitch)]])
        R_roll = np.array([[1, 0, 0],
                           [0, np.cos(roll), -np.sin(roll)],
                           [0, np.sin(roll), np.cos(roll)]])

        # 组合旋转矩阵
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
        """
        :return: 物体在一个三维欧几里得空间中的 X 轴位置, 单位米, 右为正, 由变换矩阵计算得到
        """
        # TODO: AI GENERATED CODE, VERIFY IT!
        return self.matrix[0, 3].item()

    @property
    def y(self) -> float:
        """
        :return: 物体在一个三维欧几里得空间中的 Y 轴位置, 单位米, 后为正, 由变换矩阵计算得到
        """
        # TODO: AI GENERATED CODE, VERIFY IT!
        return self.matrix[1, 3].item()

    @property
    def z(self) -> float:
        """
        :return: 物体在一个三维欧几里得空间中的 Z 轴位置, 单位米, 上为正, 由变换矩阵计算得到
        """
        # TODO: AI GENERATED CODE, VERIFY IT!
        return self.matrix[2, 3].item()

    @property
    def yaw(self) -> float:
        """
        :return: 物体绕 Z 轴旋转的角度, 单位度, 角度制, 由变换矩阵计算得到
        """
        # TODO: AI GENERATED CODE, VERIFY IT!
        return np.rad2deg(np.arctan2(self.matrix[1, 0], self.matrix[0, 0])).item()

    @property
    def pitch(self) -> float:
        """
        :return: 物体绕 Y 轴旋转的角度, 单位度, 角度制, 由变换矩阵计算得到
        """
        # TODO: AI GENERATED CODE, VERIFY IT!
        return np.rad2deg(np.arcsin(-self.matrix[2, 0])).item()

    @property
    def roll(self) -> float:
        """
        :return: 物体绕 X 轴旋转的角度, 单位度, 角度制, 由变换矩阵计算得到
        """
        # TODO: AI GENERATED CODE, VERIFY IT!
        return np.rad2deg(np.arctan2(self.matrix[2, 1], self.matrix[2, 2])).item()

