from .vector3 import Vector3


class Location(Vector3):
    """位置"""
    
    def __init__(self, 
                 x: float = 0, 
                 y: float = 0, 
                 z: float = 0) -> None:
        super().__init__(x=x, y=y, z=z)
