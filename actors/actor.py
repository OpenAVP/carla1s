import carla
import uuid
import logging
from typing import Optional, Dict

from ..errors import ContextError, CarlaError
from ..tf import Transform
from ..utils import get_logger


class Actor:
    """ Actor 对象, 是 CARLA 中 Actor 的超集
    """
    
    def __init__(self, 
                 blueprint_name: str,
                 name: str = '',
                 transform: Transform = Transform(),
                 parent: Optional['Actor'] = None,
                 attributes: Dict[str, any] = {}) -> None:
        """ 初始化 Actor 对象

        Args:
            blueprint_name (str): 用于生成 Actor 的蓝图名称.
            name (str, optional): 昵称.
            transfrom (Transform, optional): Actor 生成所用坐标的变换.
            parent (Optional['Actor'], optional): 当前 Actor 的父 Actor. 
            attributes (Dict[str, any], optional): 当前 Actor 的属性.
        """
        # PRIVAET
        self._id = str(uuid.uuid4().hex[:8])
        self._entity: Optional[carla.Actor] = None
        self._blueprint_name = blueprint_name
        self._name = name
        self._transform = transform
        self._parent = parent
        self._attributes = attributes
        self._setup = dict()
        # PUBLIC
        self.logger = get_logger(f'carla1s.actors.{self.name}')
        
        # 打印实例化日志
        self.logger.info(f'Actor {self.id} create with blueprint {self._blueprint_name}')

    @property
    def id(self) -> str:
        """ 标识符, 由当前实例初始化时生成的 `uuid4` 和 CARLA Server 授予 `entity` 的 id 共同组成."""
        carla_id = -1
        if self._entity:
            carla_id = self._entity.id
        return f'{self._id}({carla_id})'

    @property
    def entity(self) -> carla.Actor:
        """ Actor 在 CARLA Server 中的实体, 只读."""
        return self._entity

    @property
    def is_alive(self) -> bool:
        """ 实体是否存活."""
        if self._entity is None:
            return False
        return self._entity.is_alive

    @property
    def blueprint_name(self) -> str:
        """ Actor 的蓝图名称."""
        return self._blueprint_name
    
    @property
    def name(self) -> str:
        """ Actor 的昵称."""
        if self._name:
            return self._name
        defualt_name = self._blueprint_name.split('.')[0]
        defualt_name += f'-{self._blueprint_name.split(".")[-1]}'
        defualt_name += f' | {self.id}'
        return defualt_name
    
    @name.setter
    def name(self, name: str) -> None:
        old_name = self.name
        self._name = name
        self.logger = get_logger(f'carla1s.actors.{self.name}')
        self.logger.info(f'Actor name changed from {old_name} to {self.name}')

    @property
    def attributes(self) -> Dict[str, any]:
        """ Actor 属性的拷贝."""
        if self._entity:
            return self._entity.attributes
        else:
            return self._attributes.copy()

    @property
    def parent(self) -> Optional['Actor']:
        """ Actor 的父 Actor."""
        return self._parent

    def spawn(self, world: carla.World) -> 'Actor':
        """ 在 CARLA 中生成 Actor 实体.

        Args:
            world (carla.World): 世界对象
        
        Returns:
            Actor: 当前 Actor 对象, 用于链式调用
        """
        # 处理实体已经存在的情况
        if self._entity is not None:
            if self._entity.is_alive:
                raise ContextError(f"Actor {self.name} (ID: {self.id}) already exists.")
            else:
                self.logger.warning(f'Trying to re-spawn a non-alive Actor {self.name} (ID: {self.id}). Proceeding with spawn.')
        
        # 构造蓝图
        blueprint = world.get_blueprint_library().find(self._blueprint_name)
        
        # 如果用户设置了 Actor 的昵称, 则设置 role_name 属性
        if self._name:
            self._attributes['role_name'] = self._name

        # 设置属性
        for key, value in self._attributes.items():
            try:
                blueprint.set_attribute(key, value)
            except KeyError as e:
                self.logger.warning(f"Failed to set attribute '{key}': {e}, ignored.")
                continue
        
        # 如果父 Actor 被设置且不存在, 则抛出异常
        if self._parent and not self._parent.is_alive:
            raise ContextError(f"Parent actor {self._parent.name} does not exist or is not alive.")
        
        # 生成 Actor
        try:
            if self._parent:
                self._entity = world.spawn_actor(blueprint, self._transform.as_carla_transform_obj(), attach_to=self._parent.entity)
            else:
                self._entity = world.spawn_actor(blueprint, self._transform.as_carla_transform_obj())
        except RuntimeError as e:
            raise CarlaError(f"Failed to spawn actor: {e}") from e
        
        # Actor 生成成功后, 重新获取 logger 以更新名称
        self.logger = get_logger(f'carla1s.actors.{self.name}')
        self.logger.info(f'Actor {self.id} spawned.')

        # 应用缓存的设置
        for setup, option in self._setup.items():
            getattr(self, setup)(option)
            
        return self
    
    def destroy(self) -> None:
        """ 销毁 Actor 实体.
        """
        if self._entity:
            cache_id = self.id
            self._entity.destroy()
            self._entity = None
            self.logger.info(f'Actor {cache_id} destroyed.')
        else:
            self.logger.warning(f'Trying to destroy non-spawned actor.')
    
    def set_attribute(self, key: str, value: any) -> None:
        """ 设置 Actor 属性.

        Args:
            key (str): 属性名称
            value (any): 属性值
        """
        if self.is_alive:
            self.logger.warning(f'You can not set attribute on an alive actor, set will be ignored.')
        else:
            self._attributes[key] = value
            self.logger.debug(f'Set attribute "{key}={value}".')

    def set_gravity(self, option: bool) -> 'Actor':
        """ 设置 Actor 是否受重力影响.

        Args:
            option (bool): True 表示受重力影响, False 表示不受重力影响
            
        Returns:
            Actor: 当前 Actor 对象, 用于链式调用
        """
        if self.is_alive:
            self.entity.set_enable_gravity(option)
            self.logger.info(f'Set actor gravity to {option}.')
        else:
            self._setup['set_gravity', option]
            self.logger.debug(f'Set actor gravity to {option}. Option will be applied after spawn.')
        
        return self
    def set_physics(self, option: bool) -> 'Actor':
        """ 设置 Actor 是否受物理影响.

        Args:
            option (bool): True 表示受物理影响, False 表示不受物理影响
            
        Returns:
            Actor: 当前 Actor 对象, 用于链式调用
        """
        if self.is_alive:
            self.entity.set_enable_physics(option)
            self.logger.info(f'Set actor physics to {option}.')
        else:
            self._setup['set_physics'] = option
            self.logger.debug(f'Set actor physics to {option}. Option will be applied after spawn.')
            
        return self
    
    def set_transform(self, transform: Transform) -> 'Actor':
        """ 设置 Actor 的变换.
                
        如果 Actor 存在父 Actor, 该变换是相对父 Actor 的变换.

        Args:
            transform (Transform): 新的变换
            
        Returns:
            Actor: 当前 Actor 对象, 用于链式调用
        """
        self.logger.info(f'Set actor transform to {transform}.')
        if self.is_alive:
            self.entity.set_transform(transform)
        self._transform = transform
        
        return self
            
    def get_transform(self, relative: bool = False) -> Transform:
        """ 获取 Actor 的变换. 

        Args:
            relative (bool, optional): 是否获取相对父 Actor 的变换. 

        Returns:
            Transform: Actor 的变换
        """
        if self.is_alive:
            if relative and self.parent:
                return self._transform
            else:
                return Transform.from_carla_transform_obj(self.entity.get_transform())
        else:
            self.logger.warning(f"Trying to get transform of non-spawned actor. Return stored transform.")
            return self._transform
