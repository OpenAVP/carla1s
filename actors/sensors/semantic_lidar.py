import carla
import numpy as np
from ..sensor import Sensor, SensorData
from ...tf import Transform


class SemanticLidar(Sensor):
    
    def _callback(self, lidar_data: carla.SemanticLidarMeasurement):
        # 清除原先事件
        self.on_data_ready.clear()
        
        # 将激光雷达数据转换为 numpy 数组
        points = np.frombuffer(lidar_data.raw_data, dtype=np.float32).copy()
        points = np.reshape(points, (points.shape[0] // 6, 6))
        
        # 提取语义标签
        label = np.frombuffer(points[:, 5].tobytes(), dtype=np.uint32).copy()
        
        # 提取对象 ID
        oid = np.frombuffer(points[:, 4].tobytes(), dtype=np.uint32).copy()
        
        # 重组数据
        points = np.concatenate((points[:, :3], label[:, None], oid[:, None]), axis=1)
        
        # 组装传感器数据
        self.data = SensorData(points, lidar_data.frame, lidar_data.timestamp, Transform.from_carla_transform_obj(lidar_data.transform))
        self.logger.debug(f'Semantic LiDAR data captured, frame: {self.data.frame}, time: {self.data.timestamp}')
        
        # 触发事件
        self.on_data_ready.set()
