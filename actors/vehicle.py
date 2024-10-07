import carla

from .actor import Actor


class Vehicle(Actor):
    
    @property
    def entity(self) -> carla.Vehicle:
        """ Vehicle 在 CARLA Server 中的实体, 只读."""
        return self._entity

    def set_autopilot(self, option: bool) -> None:
        """设置是否启用自动驾驶.

        Args:
            option (bool): 是否启用 CARLA 自动驾驶.
        """
        if self.is_alive:
            self.entity.set_autopilot(option)
            self.logger.info(f'Set vehicle autopilot to {option}.')
        else:
            self._setup['set_autopilot'] = option
            self.logger.debug(f'Set vehicle autopilot to {option}. Option will be applied after spawn.')

