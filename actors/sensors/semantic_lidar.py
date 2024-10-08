import carla
import numpy as np
from ..sensor import Sensor


class SemanticLidar(Sensor):
    
    def _callback(self, lidar_data: carla.SemanticLidarMeasurement):
        self.on_data_ready.clear()
        points = np.frombuffer(lidar_data.raw_data, dtype=np.float32)
        points = np.reshape(points, (points.shape[0] // 6, 6))
        label = np.frombuffer(points[:, 5].tobytes(), dtype=np.uint32)  # .astype('uint32')
        points = np.concatenate((points[:, :3], label[:, None]), axis=1)
        self.data = points
        self.on_data_ready.set()
