import carla
import numpy as np
from ..sensor import Sensor


class SimpleLidar(Sensor):
    
    def _callback(self, lidar_data: carla.LidarMeasurement):
        self.on_data_ready.clear()
        points = np.frombuffer(lidar_data.raw_data, dtype=np.float32)
        points = np.reshape(points, (points.shape[0] // 4, 4))
        self.data = points
        self.on_data_ready.set()
