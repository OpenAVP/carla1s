import carla
import logging
from typing import Optional, List, Union

from .errors import ContextError
from .utils import get_logger
from .actors import ActorFactory, Actor, Sensor
from .registry import AvailableActors, AvailableMaps, AvailableVehicles, AvailableSensors, ActorTemplates
from .tf import Transform


class Context:
    """CARLA1S 的上下文聚合, 管理 CARLA 任务的生命周期.

    该类实现了上下文管理器协议, 可以使用 with 语句来管理 CARLA 任务的生命周期.
    """
    
    def __init__(self,
                 host: str = '127.0.0.1',
                 port: int = 2000,
                 timeout_sec: float = 1.0,
                 log_level: int = logging.INFO,
                 logger: Optional[logging.Logger] = None) -> None:
        """初始化 Context 实例.

        Args:
            host (str, optional): CARLA 服务端的 IP 地址. 默认为 '127.0.0.1'.
            port (int, optional): CARLA 服务端的端口号. 默认为 2000.
            timeout_sec (float, optional): CARLA 客户端的超时时间. 默认为 1.0.
            log_level (int, optional): 日志记录的级别. 默认为 logging.INFO.
            logger (Optional[logging.Logger], optional): 用于记录日志的 Logger 实例. 如果为 None, 则会创建一个新的 RichLogger 实例. 默认为 None.
        """
        # PRIVATE
        self._host = host
        self._port = port
        self._timeout_sec = timeout_sec

        self._client: Optional[carla.Client] = None
        self._actors: List[Actor] = list()

        # LOGGING 
        logging.basicConfig(level=log_level)
        logging.getLogger().handlers.clear()
        self.logger = get_logger('carla1s.context') if logger is None else logger
        
        # PUBLIC
        self.actor_factory = ActorFactory(self.actors)
        self.available_actors = AvailableActors
        self.available_maps = AvailableMaps
        self.available_vehicles = AvailableVehicles
        self.available_sensors = AvailableSensors
        self.actor_templates = ActorTemplates
    def __enter__(self) -> 'Context':
        # 尝试连接到 CARLA 服务端
        try:
            self.connect()
            self.logger.info('Context begin.')
        except RuntimeError as e:
            self.logger.critical('Context initialization failed due to connection error.', exc_info=e)
            raise ContextError('Context initialization failed due to connection error.') from e
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb) -> None:
        still_connected = self.test_connection()
        self.disconnect()
        if exc_type is not None:
            self.logger.error('Context exit with exception.', exc_info=(exc_type, exc_val, exc_tb))
        elif not still_connected:
            self.logger.warning('Context exit with failed connection.')
        else:
            self.logger.info('Context exit.')
        return None

    @property
    def initialized(self) -> bool:
        """当前上下文是否已经通过 with 语句初始化."""
        return self._initialized
    
    @property
    def client(self) -> carla.Client:
        """当前上下文中的 CARLA 客户端实例."""
        return self._client

    @property
    def world(self) -> carla.World:
        """当前上下文中的 CARLA 世界实例. 该实例可能会因为地图切换而发生变化."""
        return self.client.get_world()
    
    @property
    def actors(self) -> List[Actor]:
        """当前上下文中的所有 Actor 实例."""
        return self._actors
    
    def connect(self) -> 'Context':
        """执行与 CARLA 服务端的连接.

        Returns:
            Context: 当前 Context 实例, 用于链式调用.

        Raises:
            RuntimeError: 如果连接 CARLA 服务端失败.
        """
        # 如果 CARLA 客户端已经存在并且可以被连接, 则直接返回
        if self.client is not None and self.test_connection():
            return self

        # 构造 CARLA 客户端实例
        self._client = carla.Client(host=self._host, port=self._port)
        self._client.set_timeout(self._timeout_sec)
        
        # 测试与 CARLA 服务端的连接
        if not self.test_connection():
            raise RuntimeError(f'Failed to connect to CARLA server at {self._host}:{self._port}.')
        return self
        
    def disconnect(self) -> 'Context':
        """断开与 CARLA 服务端的连接.

        Returns:
            Context: 当前 Context 实例, 用于链式调用.
        """
        self.stop_all_sensors()
        self.destroy_all_actors()
        self._client = None
        return self
    
    def spawn_all_actors(self) -> 'Context':
        """对所上下文中注册的执行 spawn 操作.

        Returns:
            Context: 当前 Context 实例, 用于链式调用.
        """
        # 对 actors 列表依据父子关系重排, 构建关系树
        actors_tree = dict()
        for actor in self.actors:
            if actor.parent is None:
                actors_tree[actor] = []
            else:
                if actor.parent not in actors_tree:
                    actors_tree[actor.parent] = []
                actors_tree[actor.parent].append(actor)

        # 按照树的层次结构重新排序 actors 列表
        sorted_actors = []
        def dfs(node):
            sorted_actors.append(node)
            for child in actors_tree.get(node, []):
                dfs(child)
        
        for root in [actor for actor in self.actors if actor.parent is None]:
            dfs(root)

        # 打印日志
        self.logger.info(f'Spawn all actors with sequence: {sorted_actors}')

        # 执行 spawn 操作
        for actor in sorted_actors:
            actor.spawn(self.world)
        
        return self
    
    def destroy_all_actors(self) -> 'Context':
        """销毁当前上下文中的所有 Actor 实例.

        Returns:
            Context: 当前 Context 实例, 用于链式调用.
        """
        for actor in self.actors:
            actor.destroy()
        return self

    def test_connection(self, test_timeout_sec: float = 0.1) -> bool:
        """测试与 CARLA 服务端的连接.

        Args:
            test_timeout_sec (float, optional): 测试连接的超时时间. 默认为 0.1.

        Returns:
            bool: 如果连接成功返回 True, 否则返回 False.
        """
        if self.client is None:
            return False
        try:
            self.client.set_timeout(test_timeout_sec)
            self.client.get_server_version()
            return True
        except RuntimeError:
            return False
        finally:
            self.client.set_timeout(self._timeout_sec)

    def reload_world(self, map_name: Union[str, AvailableMaps, None] = None, reset_actor_list: bool = True) -> 'Context':
        """重新加载 CARLA 世界.

        Args:
            map_name (Union[str, AvailableMaps, None], optional): 需要加载的地图名称. 默认为 None. 
            如果为 None, 则执行重载而不变更地图.
            reset_actor_list (bool, optional): 是否重置 Actor 列表. 默认为 True.
        Returns:
            Context: 当前 Context 实例, 用于链式调用.
        """
        # 获取地图名
        if isinstance(map_name, AvailableMaps):
            map_name = map_name.value
        
        # 重新加载世界, settings 与 Executor 有关，不重置
        if map_name is not None:
            self.client.load_world(map_name, reset_settings=False)
        else:
            self.client.reload_world(reset_settings=False)
            
        # 重置 Actor 列表
        if reset_actor_list:
            self._actors = list()

        return self

    def get_spawn_point(self, index: int = 0) -> Transform:
        """获取指定索引的 spawn point.

        Args:
            index (int, optional): spawn point 的索引. 默认为 0.

        Returns:
            Transform: 指定索引的 spawn point.
        """
        carla_tf = self.world.get_map().get_spawn_points()[index]
        return Transform.from_carla_transform_obj(carla_tf)

    def listen_all_sensors(self) -> 'Context':
        """监听所有传感器.

        Returns:
            Context: 当前 Context 实例, 用于链式调用.
        """
        for actor in self.actors:
            if isinstance(actor, Sensor):
                actor.listen()
        return self

    def stop_all_sensors(self) -> 'Context':
        """停止所有传感器.

        Returns:
            Context: 当前 Context 实例, 用于链式调用.
        """
        for actor in self.actors:
            if isinstance(actor, Sensor):
                actor.stop()
        return self