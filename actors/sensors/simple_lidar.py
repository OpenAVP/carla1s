import carla
import numpy as np
from ..sensor import Sensor, SensorData
from ...tf import Transform


class SimpleLidar(Sensor):
    
    def _callback(self, lidar_data: carla.LidarMeasurement):
        # 清除原先事件
        self.on_data_ready.clear()
        
        # 将激光雷达数据转换为 numpy 数组
        points = np.frombuffer(lidar_data.raw_data, dtype=np.float32)
        points = np.reshape(points, (points.shape[0] // 4, 4))
        
        # 组装传感器数据
        self.data = SensorData(points, lidar_data.frame, lidar_data.timestamp, Transform.from_carla_transform_obj(lidar_data.transform))
        self.logger.debug(f'LiDAR data captured, frame: {self.data.frame}, time: {self.data.timestamp}, tf: {self.data.transform}')
        
        # 触发事件
        self.on_data_ready.set()
