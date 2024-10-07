import carla
import logging
from functools import wraps
from typing import Optional, NamedTuple, List
from rich.logging import RichHandler

from .errors import ContextError
from .utils import ProjectFormatter
from .actors import ActorFactory, Actor


def context_func(method):
    """
    将一个方法包装为上下文方法, 该方法只有在上下文初始化且对 CARLA 连接成功时才会执行.
    """
    @wraps(method)
    def wrapper(self, *args, **kwargs):
        if not self._initialized:
            self.logger.warning(f'Method {method.__name__}() skipped because the context is not initialized, '
                                f'use with statement to initialize the context.')
            return None
        if not self.test_connection():
            self.logger.warning(f'Method {method.__name__}() skipped because the CARLA connection is not established.')
            return None
        return method(self, *args, **kwargs)
    return wrapper

class Context:
    """
    CARLA1S 的上下文聚合, 管理 CARLA 任务的生命周期.

    该类实现了上下文管理器协议, 可以使用 with 语句来管理 CARLA 任务的生命周期.
    """
    
    def __init__(self,
                 host: str = '127.0.0.1',
                 port: int = 2000,
                 timeout_sec: float = 1.0,
                 log_level: int = logging.INFO,
                 logger: Optional[logging.Logger] = None) -> None:
        """
        :param host: CARLA 服务端的 IP 地址.
        :param port: CARLA 服务端的端口号.
        :param timeout_sec: CARLA 客户端的超时时间.
        :param log_level: 日志记录的级别.
        :param logger: 用于记录日志的 Logger 实例, 如果为 None, 则会创建一个新的 RichLogger 实例.
        """
        self._host = host
        self._port = port
        self._timeout_sec = timeout_sec
        self._log_level = log_level

        self._initialized = False

        self._client: Optional[carla.Client] = None
        self._actors: List[Actor] = list()

        self.logger = self._create_logger() if logger is None else logger
        self.actor_factory = ActorFactory(self.actors)
    
    def __enter__(self) -> 'Context':
        # 只有当程序使用 with 语句时, 才会标记 _initialized 为 True
        self._initialized = True

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
        """
        :return: 当前上下文是否已经通过 with 语句初始化.
        """
        return self._initialized
    
    @property
    def client(self) -> carla.Client:
        """
        :return: 当前上下文中的 CARLA 客户端实例.
        """
        return self._client

    @property
    @context_func
    def world(self) -> carla.World:
        """
        :return: 当前上下文中的 CARLA 世界实例. 该实例可能会因为地图切换而发生变化.
        """
        return self.client.get_world()
    
    @property
    def actors(self) -> list:
        """
        :return: 当前上下文中的所有 Actor 实例.
        """
        return self._actors
            
    def _create_logger(self) -> logging.Logger:
        """
        创建一个默认的 logging.Logger 实例, 并应用 RichHandler.
        :return: logging.Logger 实例.
        """
        logger = logging.getLogger('carla1s.Context')
        logger.setLevel(self._log_level)
        handler = RichHandler(rich_tracebacks=True, markup=True)
        handler.setFormatter(ProjectFormatter('[on blue][%(shortname)s][/] %(message)s'))
        logger.addHandler(handler)
        return logger
    
    def connect(self) -> 'Context':
        """
        执行与 CARLA 服务端的连接.
        :return: 链式调用.
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
        """
        断开与 CARLA 服务端的连接.
        :return: 链式调用.
        """
        self.destroy_all_actors()
        self._client = None
        return self
        
    def destroy_all_actors(self) -> 'Context':
        """
        销毁当前上下文中的所有 Actor 实例.
        :return: None.
        """
        for actor in self.actors:
            actor.destroy()
        return self

    def test_connection(self, test_timeout_sec: float = 0.1) -> bool:
        """
        测试与 CARLA 服务端的连接.
        :return: None.
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

    @context_func
    def test(self):
        self.logger.info('OK')