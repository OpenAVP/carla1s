import carla

from .actor import Actor
from ..tf import Transform


class Vehicle(Actor):
    
    def spawn(self, world: carla.World) -> 'Vehicle':
        return super().spawn(world)

    def set_gravity(self, option: bool) -> 'Vehicle':
        return super().set_gravity(option)

    def set_physics(self, option: bool) -> 'Vehicle':
        return super().set_physics(option)

    def set_transform(self, transform: Transform) -> 'Vehicle':
        return super().set_transform(transform)
    
    @property
    def entity(self) -> carla.Vehicle:
        """ Vehicle 在 CARLA Server 中的实体, 只读."""
        return self._entity

    def set_autopilot(self, option: bool) -> 'Vehicle':
        """设置是否启用自动驾驶.

        Args:
            option (bool): 是否启用 CARLA 自动驾驶.
            
        Returns:
            Vehicle: 当前 Vehicle 对象, 用于链式调用
        """
        if self.is_alive:
            self.entity.set_autopilot(option)
            self.logger.info(f'Set vehicle autopilot to {option}.')
        else:
            self._setup['set_autopilot'] = option
            self.logger.debug(f'Set vehicle autopilot to {option}. Option will be applied after spawn.')
        
        return self

