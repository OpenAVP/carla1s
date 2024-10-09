import carla
import numpy as np

from ..sensor import Sensor, SensorData
from ...tf import Transform


class SemanticCamera(Sensor):
    
    def _callback(self, image: carla.Image):
        # 清除原先事件
        self.on_data_ready.clear()
        
        # 转换原始图像到语义分割
        image.convert(carla.ColorConverter.CityScapesPalette)
        
        # 将语义分割图像数据转换为 numpy 数组
        img = np.frombuffer(image.raw_data, dtype=np.uint8)
        img = img.reshape((image.height, image.width, 4))
        img = img[:, :, :3]  # 去掉 alpha 通道
        
        # 组装传感器数据
        self.data = SensorData(img, image.frame, image.timestamp, Transform.from_carla_transform_obj(image.transform))
        self.logger.debug(f'Semantic image captured, frame: {self.data.frame}, time: {self.data.timestamp}, tf: {self.data.transform}')
        
        # 触发事件
        self.on_data_ready.set()
