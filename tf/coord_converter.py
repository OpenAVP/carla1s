import numpy as np
from typing import List, Union

from .transform import Transform
from .point import Point

class CoordConverter:
    """
    坐标系转换器, 用于将一个坐标系下的变换转换到另一个坐标系下.

    :ivar TF_TO_ROS: 预定义的从 CARLA 到 ROS 的变换矩阵
    """

    # TODO: AI GENERATED CODE, VERIFY IT!
    TF_TO_ROS = Transform(matrix=np.array([[0, 0, 1, 0],
                                           [-1, 0, 0, 0],
                                           [0, -1, 0, 0],
                                           [0, 0, 0, 1]]))

    @classmethod
    def from_system(cls, *transform: Union[Transform, Point]) -> 'CoordConverter._CoordConverterStep':
        """
        加载同一个坐标系下的多个变换
        :param transform: 该坐标系下变换或者视作变换的点
        :return:
        """
        if not transform:
            raise ValueError("At least one transform must be provided.")
        return cls._CoordConverterStep(list(transform))

    class _CoordConverterStep:

        def __init__(self, transform_list: List[Transform]):
            if not transform_list:
                raise ValueError("The list of transforms must not be empty.")
            self.data: List[Union[Point, Transform]] = transform_list

        def apply_transform(self, transform: Transform) -> 'CoordConverter._CoordConverterStep':
            """
            应用变换到另一个坐标系
            :param transform: 变换
            """
            if not isinstance(transform, Transform) or isinstance(transform, Point):
                raise ValueError("The transform must be an instance of Transform.")

            # TODO: 实现逻辑并更新注释
            return self

        def as_list(self) -> List[Union[Point, Transform]]:
            """
            :return: 以列表方式转换后的变换组
            """
            return self.data

        def as_element(self) -> Union[Point, Transform]:
            """
            :return: 以单个元素方式转换后的变换组
            """
            if len(self.data) != 1:
                raise ValueError("The list of transforms must contain only one element.")
            return self.data[0]
