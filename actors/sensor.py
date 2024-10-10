import carla
import logging
import numpy as np
from typing import Optional, Dict
from threading import Event

from .actor import Actor
from ..tf import Transform


class SensorData:
    """ 传感器数据类, 用于存储传感器数据."""
    
    def __init__(self, data: np.ndarray, frame: int, timestamp: float, transform: Transform) -> None:
        self.data = data
        self.frame = frame
        self.timestamp = timestamp
        self.transform = transform

class Sensor(Actor):
    
    def __init__(self, 
                 blueprint_name: str,
                 name: str = '',
                 transform: Transform = Transform(),
                 parent: Optional['Actor'] = None,
                 attributes: Dict[str, any] = {}) -> None:
        super().__init__(blueprint_name, name, transform, parent, attributes)
        self.on_data_ready = Event()
        self.data: Optional[SensorData] = None
    
    @property
    def entity(self) -> carla.Sensor:
        """Sensor 在 CARLA Server 中的实体, 只读."""
        return self._entity
    
    def spawn(self, world: carla.World) -> 'Sensor':
        return super().spawn(world)

    def set_gravity(self, option: bool) -> 'Sensor':
        return super().set_gravity(option)

    def set_physics(self, option: bool) -> 'Sensor':
        return super().set_physics(option)

    def set_transform(self, transform: Transform) -> 'Sensor':
        return super().set_transform(transform)
    
    def listen(self) -> 'Sensor':
        """开始监听传感器数据."""
        self.entity.listen(self._callback)
        self.logger.info(f"Begin listening.")
        return self

    def stop(self) -> 'Sensor'  :
        """停止监听传感器数据."""
        if self.entity and self.entity.is_listening:
            self.entity.stop()
        self.logger.warning(f"Stop listening.")
        return self
    
    def _callback(self, data: carla.SensorData):
        pass