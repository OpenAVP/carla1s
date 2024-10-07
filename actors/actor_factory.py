import carla
from enum import Enum
from typing import TypeVar, Type, Optional, Union, List

from .actor import Actor
from ..tf import Transform


T = TypeVar('T', bound=Actor)

class ActorFactory:
    
    def __init__(self, ref_actor_list: List[Actor]) -> None:
        self._ref_actor_list = ref_actor_list
    
    class Builder:
        def __init__(self, 
                     blueprint_name: str, 
                     actor_class: Type[T],
                     ref_actor_list: List[Actor]):
            self._actor_class = actor_class
            self._blueprint_name = blueprint_name
            self._name = ''
            self._transform = Transform()
            self._parent = None
            self._attributes = {}
            self._ref_actor_list = ref_actor_list
        
        def with_name(self, name: str) -> 'ActorFactory.Builder':
            self._name = name
            return self
        
        def with_transform(self, 
                           transform: Transform = None,
                           *,
                           x: Optional[float] = None,
                           y: Optional[float] = None,
                           z: Optional[float] = None,
                           yaw: Optional[float] = None,
                           pitch: Optional[float] = None,
                           roll: Optional[float] = None) -> 'ActorFactory.Builder':
            tf = self._transform if transform is None else transform
            
            # 使用字典来存储需要更新的属性
            updates = {}
            if x is not None:
                updates['x'] = x
            if y is not None:
                updates['y'] = y
            if z is not None:
                updates['z'] = z
            if yaw is not None:
                updates['yaw'] = yaw
            if pitch is not None:
                updates['pitch'] = pitch
            if roll is not None:
                updates['roll'] = roll
            
            # 一次性更新所有需要改变的属性
            if updates:
                tf = Transform(
                    x=updates.get('x', tf.x),
                    y=updates.get('y', tf.y),
                    z=updates.get('z', tf.z),
                    yaw=updates.get('yaw', tf.yaw),
                    pitch=updates.get('pitch', tf.pitch),
                    roll=updates.get('roll', tf.roll)
                )
    
            self._transform = tf
            return self
        
        def with_parent(self, parent: Actor) -> 'ActorFactory.Builder':
            self._parent = parent
            return self
        
        def with_attributes(self, **attributes) -> 'ActorFactory.Builder':
            self._attributes.update(attributes)
            return self
        
        def build(self) -> T:
            actor = self._actor_class(
                blueprint_name=self._blueprint_name,
                name=self._name,
                transform=self._transform,
                parent=self._parent,
                attributes=self._attributes
            )
            self._ref_actor_list.append(actor)
            return actor
        
    def create(self, 
               actor_class: Type[T], 
               *,
               from_blueprint: Union[str, carla.ActorBlueprint, Enum] = None) -> 'Builder':
        # 获取 blueprint_name
        if isinstance(from_blueprint, Enum):
            blueprint_name = from_blueprint.value
        elif isinstance(from_blueprint, carla.ActorBlueprint):
            blueprint_name = from_blueprint.id
        else:
            blueprint_name = from_blueprint
        
        return self.Builder(blueprint_name, actor_class, self._ref_actor_list)
