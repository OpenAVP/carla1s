import carla
import numpy as np
from ..sensor import Sensor


class SimpleRadar(Sensor):
    
    def _callback(self, radar_data: carla.RadarMeasurement):
        self.on_data_ready.clear()
        velocity = self.parent.entity.get_velocity()
        points = np.frombuffer(radar_data.raw_data, dtype=np.dtype('f4'))
        points = np.reshape(points, (len(radar_data), 4))
        points = np.insert(points, 4, velocity.x ,axis = 1)
        points = np.insert(points, 5, velocity.y ,axis = 1)
        self.data = points
        self.on_data_ready.set()
