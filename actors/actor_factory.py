import carla
from enum import Enum
from typing import TypeVar, Type, Optional, Union, List, Tuple, Dict

from .actor import Actor
from .sensors.simple_lidar import SimpleLidar
from .sensors.semantic_lidar import SemanticLidar
from .sensors.simple_radar import SimpleRadar
from .sensors.rgb_camera import RgbCamera
from .sensors.depth_camera import DepthCamera
from .sensors.semantic_camera import SemanticCamera
from .actor_template import ActorTemplate
from ..tf import Transform
from ..utils.logging import get_logger


T = TypeVar('T', bound=Actor)

class ActorFactory:
    
    ACTOR_CLASS_TO_BLUEPRINT = {
        RgbCamera: 'sensor.camera.rgb',
        SimpleLidar: 'sensor.lidar.ray_cast',
        SemanticLidar: 'sensor.lidar.ray_cast_semantic',
        SimpleRadar: 'sensor.other.radar',
        DepthCamera: 'sensor.camera.depth',
        SemanticCamera: 'sensor.camera.semantic_segmentation'
    }
    
    def __init__(self, ref_actor_list: List[Actor]) -> None:
        self._ref_actor_list = ref_actor_list
        self.logger = get_logger('carla1s.actors.actor_factory')
    
    class Builder:
        def __init__(self, 
                     blueprint_name: str,
                     actor_class: Type[T],
                     ref_actor_list: List[Actor],
                     attributes: Dict[str, any] = dict()):
            self._actor_class = actor_class
            self._blueprint_name = blueprint_name
            self._name = ''
            self._transform = Transform()
            self._parent = None
            self._attributes = attributes
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
        
        def build_many(self, count: int) -> Tuple[T]:
            return tuple(self.build() for _ in range(count))
        
    def create(self, 
               actor_class: Type[T], 
               *,
               from_blueprint: Union[str, carla.ActorBlueprint, Enum] = None,
               from_template: Union[ActorTemplate, Enum] = None) -> 'Builder':
        # 阻止同时使用 from_blueprint 和 from_template
        if from_blueprint is not None and from_template is not None:
            raise ValueError('Cannot use both from_blueprint and from_template.')
        
        # 获取 blueprint_name 和 attributes
        blueprint_name = None
        attributes = {}
        
        # 对精确类进行豁免, 要求双空
        if from_blueprint is None and from_template is None:
            blueprint_name = self.ACTOR_CLASS_TO_BLUEPRINT.get(actor_class)

        # 确定 blueprint_name 和 attributes
        if from_blueprint is not None:
            if isinstance(from_blueprint, Enum):
                blueprint_name = from_blueprint.value
            elif isinstance(from_blueprint, carla.ActorBlueprint):
                blueprint_name = from_blueprint.id
                attributes = from_blueprint.attributes
            elif isinstance(from_blueprint, str):
                blueprint_name = from_blueprint
        elif from_template is not None:
            if isinstance(from_template, ActorTemplate):
                blueprint_name = from_template.blueprint_name
                attributes = from_template.attributes
            elif isinstance(from_template, Enum):
                blueprint_name = from_template.value.blueprint_name
                attributes = from_template.value.attributes

        if blueprint_name is None:
            raise ValueError("Invalid blueprint or template provided")
        # 打印创建日志
        self.logger.info(f'Creating actor {actor_class.__name__} with blueprint {blueprint_name}')
        
        return self.Builder(blueprint_name, actor_class, self._ref_actor_list, attributes)
