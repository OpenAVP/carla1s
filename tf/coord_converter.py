import numpy as np
from typing import List, Union

from .transform import Transform
from .point import Point

class CoordConverter:
    """坐标系转换器, 用于将一个坐标系下的变换转换到另一个坐标系下.

    Attributes:
        TF_TO_ROS: 预定义的从 CARLA 到 ROS 的变换矩阵
    """

    # TODO: AI GENERATED CODE, VERIFY IT!
    TF_TO_ROS = Transform(matrix=np.array([[0, 0, 1, 0],
                                           [-1, 0, 0, 0],
                                           [0, -1, 0, 0],
                                           [0, 0, 0, 1]]))

    @classmethod
    def from_system(cls, *transform: Union[Transform, Point]) -> 'CoordConverter._CoordConverterStep':
        """加载同一个坐标系下的多个变换

        Args:
            *transform: 该坐标系下变换或者视作变换的点

        Returns:
            CoordConverter._CoordConverterStep: 坐标转换步骤对象

        Raises:
            ValueError: 如果没有提供任何变换
        """
        if not transform:
            raise ValueError("At least one transform must be provided.")
        return cls._CoordConverterStep(list(transform))

    class _CoordConverterStep:

        def __init__(self, transform_list: List[Transform]):
            """初始化 _CoordConverterStep 对象

            Args:
                transform_list: 变换列表

            Raises:
                ValueError: 如果变换列表为空
            """
            if not transform_list:
                raise ValueError("The list of transforms must not be empty.")
            self.data: List[Union[Point, Transform]] = transform_list

        def apply_transform(self, transform: Transform) -> 'CoordConverter._CoordConverterStep':
            """应用变换到另一个坐标系

            Args:
                transform: 要应用的变换

            Returns:
                CoordConverter._CoordConverterStep: 应用变换后的对象

            Raises:
                ValueError: 如果提供的变换不是 Transform 实例
            """
            if not isinstance(transform, Transform) or isinstance(transform, Point):
                raise ValueError("The transform must be an instance of Transform.")

            new_data: List[Union[Point, Transform]] = list()

            # 遍历所有变换，并进行变换
            for data in self.data:
                temp_matrix = transform.matrix.dot(data.matrix)
                temp_transform = Transform(matrix=temp_matrix)
                new_data.append(temp_transform)
            self.data = new_data
            return self

        def get_list(self) -> List[Union[Point, Transform]]:
            """获取转换后的变换列表

            Returns:
                List[Union[Point, Transform]]: 转换后的变换列表
            """
            return self.data

        def get_single(self) -> Union[Point, Transform]:
            """获取转换后的单个变换

            Returns:
                Union[Point, Transform]: 转换后的单个变换

            Raises:
                ValueError: 如果列表中不只包含一个元素
            """
            if len(self.data) != 1:
                raise ValueError("The list of transforms must contain only one element.")
            return self.data[0]
