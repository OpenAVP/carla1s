import carla
import logging
import numpy as np
from typing import Optional, Dict
from threading import Event

from .actor import Actor
from ..tf import Transform


class Sensor(Actor):
    
    def __init__(self, 
                 blueprint_name: str,
                 name: str = '',
                 transform: Transform = Transform(),
                 parent: Optional['Actor'] = None,
                 attributes: Dict[str, any] = {}) -> None:
        super().__init__(blueprint_name, name, transform, parent, attributes)
        self.on_data_ready = Event()
        self.data: Optional[np.ndarray] = None
    
    @property
    def entity(self) -> carla.Sensor:
        """Sensor 在 CARLA Server 中的实体, 只读."""
        return self._entity
    
    def listen(self):
        """开始监听传感器数据."""
        self.entity.listen(self._callback)
        
    def stop(self):
        """停止监听传感器数据."""
        self.entity.stop()
    
    def _callback(self, data: carla.SensorData):
        pass
        
