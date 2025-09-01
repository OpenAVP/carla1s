import carla
import numpy as np
from ..sensor import Sensor, SensorData
from ...tf import Transform


class SimpleRadar(Sensor):
    
    def _callback(self, radar_data: carla.RadarMeasurement):
        # 清除原先事件
        self.on_data_ready.clear()
        
        # 获取父实体的速度
        velocity = self.parent.entity.get_velocity()
        
        # 将雷达数据转换为 numpy 数组
        points = np.frombuffer(radar_data.raw_data, dtype=np.dtype('f4')).copy()
        points = np.reshape(points, (len(radar_data), 4))
        
        # 添加父实体的速度信息
        points = np.insert(points, 4, velocity.x, axis=1)
        points = np.insert(points, 5, velocity.y, axis=1)
        
        # 组装传感器数据
        self.data = SensorData(points, radar_data.frame, radar_data.timestamp, Transform.from_carla_transform_obj(radar_data.transform))
        self.logger.debug(f'Radar data captured, frame: {self.data.frame}, time: {self.data.timestamp}')
        
        # 触发事件
        self.on_data_ready.set()
