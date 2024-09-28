import numpy as np

from .transform import Transform


class Point(Transform):

    def __init__(self,
                 x: float = 0.0,
                 y: float = 0.0,
                 z: float = 0.0):
        """
        空间中的点的表示, 是变换的一种特例, 它只考虑位置而不考虑旋转.

        左手坐标系, 与 CARLA 和 UnrealEngine 保持一致.

        :param x: 表示物体在一个三维欧几里得空间中的 X 轴位置, 单位米, 右为正
        :param y: 表示物体在一个三维欧几里得空间中的 Y 轴位置, 单位米, 后为正
        :param z: 表示物体在一个三维欧几里得空间中的 Z 轴位置, 单位米, 上为正
        """
        super().__init__(x, y, z)

    @property
    def matrix(self) -> np.ndarray:
        """
        :return: 变换矩阵, 4x4 的齐次变换矩阵, 旋转舍弃并重置部分为单位矩阵
        """
        matrix = super().matrix
        matrix[:3, :3] = np.eye(3)
        return matrix

    @property
    def roll(self) -> float:
        """
        :return: 由于点没有旋转, 因此返回 0
        """
        return 0.0

    @property
    def pitch(self) -> float:
        """
        :return: 由于点没有旋转, 因此返回 0
        """
        return 0.0

    @property
    def yaw(self) -> float:
        """
        :return: 由于点没有旋转, 因此返回 0
        """
        return 0.0