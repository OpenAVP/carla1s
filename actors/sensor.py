import carla
import logging
import numpy as np
from typing import Optional
from threading import Event

from .actor import Actor


class Sensor(Actor):
    
    def __init__(self, 
                 blueprint: carla.ActorBlueprint, 
                 name: str, 
                 parent: Actor = None, 
                 attributes: dict = None, 
                 logger: logging.Logger = None):
        super().__init__(blueprint, name, parent, attributes, logger)
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
        
