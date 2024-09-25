import carla
import logging
from typing import Optional, NamedTuple
from rich.logging import RichHandler


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

        self._client: Optional[carla.Client] = None
        self._actors = list()
        
        self.logger = self._create_logger() if logger is None else logger
    
    def __enter__(self) -> 'Context':
        self.logger.info('Context begin.')
        self.connect()
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb) -> None:
        self.disconnect()
        self.logger.info('Context end.')
        return None
    
    @property
    def client(self) -> carla.Client:
        """
        :return: 当前上下文中的 CARLA 客户端实例.
        """
        return self._client
    
    @property
    def actor(self) -> list:
        """
        :return: 当前上下文中的所有 Actor 实例.
        """
        return self._actors
            
    def _create_logger(self) -> logging.Logger:
        """
        创建一个默认的 logging.Logger 实例, 并应用 RichHandler.
        :return: logging.Logger 实例.
        """
        logger = logging.getLogger('carla1s')
        logger.setLevel(self._log_level)
        handler = RichHandler(rich_tracebacks=True)
        logger.addHandler(handler)
        return logger
    
    def connect(self) -> 'Context':
        """
        执行与 CARLA 服务端的连接.
        :return: 链式调用.
        """
        # 构造 CARLA 客户端实例
        self._client = carla.Client(host=self._host, port=self._port)
        self._client.set_timeout(self._timeout_sec)
        
        # 测试与 CARLA 服务端的连接
        self._client.get_server_version()
        
    def disconnect(self) -> 'Context':
        """
        断开与 CARLA 服务端的连接.
        :return: 链式调用.
        """
        self._client = None
        
    def destroy_all_actors(self) -> None:
        """
        销毁当前上下文中的所有 Actor 实例.
        :return: None.
        """
        cmd = [carla.command.DestroyActor(x) for x in self.__actor_registry]
        self.client.apply_batch_sync(cmd)

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
