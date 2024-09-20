from .vector3 import Vector3


class Rotation(Vector3):
    """旋转"""
    
    def __init__(self, 
                 pitch: float = 0,
                 yaw: float = 0,
                 roll: float = 0) -> None:
        super().__init__(x=roll, y=pitch, z=yaw)
        self.roll = self.x
        self.pitch = self.y
        self.yaw = self.z
