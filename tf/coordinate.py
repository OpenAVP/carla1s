import carla
import numpy as np
import copy
from typing import Optional

from .transform import Transform

class Coordinate:
    def __init__(self, transform: Optional[Transform]):
        self.data = copy.deepcopy(transform)

    def change_orientation(self, transform: Transform) -> 'Coordinate':
        """改变坐标系方向

        Args:
            R_matrix: 目标坐标系相对于当前坐标系的3x3旋转矩阵
        """
        R_matrix = transform.matrix[:3, :3]
        R = self.data.matrix[:3, :3]
        t = self.data.matrix[:3, 3]
        R = R.dot(R_matrix)
        Rt = np.hstack((R, t.reshape(-1, 1)))
        temp_matrix = np.vstack((Rt, np.array([0, 0, 0, 1])))
        self.data.matrix = temp_matrix
        return self

    def apply_transform(self, transform: Transform) -> 'Coordinate':
        """应用变换到另一个坐标系

        Args:
            transform: 要应用的变换

        Returns:
            CoordConverter._CoordConverterStep: 应用变换后的对象

        Raises:
            ValueError: 如果提供的变换不是 Transform 实例
        """

        # 进行变换
        temp_matrix = np.linalg.inv(transform.matrix).dot(self.data.matrix)
        temp_transform = Transform(matrix=temp_matrix)
        self.data = temp_transform
        return self
