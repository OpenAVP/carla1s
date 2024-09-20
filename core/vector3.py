class Vector3:
    """三维向量"""
    
    def __init__(self,
                 *,
                 x: float = 0.0,
                 y: float = 0.0,
                 z: float = 0.0) -> None:
        self.x = x
        self.y = y
        self.z = z
