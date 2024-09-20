import carla

from .location import Location
from .rotation import Rotation

class Transfrom:
    """变换, 包含位置和旋转"""
    
    def __init__(self,
                 location: Location = Location(),
                 rotation: Rotation = Rotation()) -> None:
        self.location = location
        self.rotation = rotation
        
    def as_carla_transform(self) -> carla.Transform:
        """转换为 carla.Transform 对象"""
        loc = carla.Location(x=self.location.x,
                             y=self.location.y,
                             z=self.location.z)
        rot = carla.Rotation(roll=self.rotation.roll,
                             pitch=self.rotation.pitch,
                             yaw=self.rotation.yaw)
        tf = carla.Transform(location=loc, rotation=rot)
        return tf
