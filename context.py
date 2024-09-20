import carla
import logging
from typing import Optional, NamedTuple
from rich.logging import RichHandler


class Context:
    """CARLA1S 的上下文聚合, 管理 CARLA 任务的生命周期.
    
    强烈建议使用 with 语句来创建上下文, 以确保资源的正确释放.
    
    Example:
    ```python
        with Context() as ctx:
            ctx.do()
    ```
    
    """
    
    def __init__(self,
                 host: str = '127.0.0.1',
                 port: int = 2000,
                 timeout_sec: float = 1.0,
                 log_level: int = logging.INFO,
                 logger: Optional[logging.Logger] = None) -> None:
        """
        Args:
            host (str, optional): CARLA 服务端的 IP 地址. 默认为 '127.0.0.1'.
            port (int, optional): CARLA 服务端的端口号. 默认为 2000.
            timeout_sec (float, optional): CARLA 服务端的超时秒数. 默认为 1.0.
            log_level (int, optional): 日志记录器的日志级别. 默认为 logging.INFO.
            logger (Optional[logging.Logger], optional): 自定义的日志记录器实例. 为 None 时默认创建一个.
        """
        self.__host = host
        self.__port = port
        self.__timeout_sec = timeout_sec
        self.__log_level = log_level

        self.__client: Optional[carla.Client] = None
        self.__actors = list()
        
        self.logger = self.__create_logger() if logger is None else logger
    
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
        """返回 CARLA 客户端实例."""
        return self.__client
    
    @property
    def actor(self) -> list:
        """返回当前上下文中的 Actor 列表."""
        return self.__actors
            
    def __create_logger(self) -> logging.Logger:
        """创建一个用于记录日志的 Logger 实例."""
        logger = logging.getLogger('carla1s')
        logger.setLevel(self.__log_level)
        handler = RichHandler(rich_tracebacks=True)
        logger.addHandler(handler)
        return logger
    
    def connect(self) -> 'Context':
        """连接到 CARLA 服务端."""
        # 构造 CARLA 客户端实例
        self.__client = carla.Client(host=self.__host, port=self.__port)
        self.__client.set_timeout(self.__timeout_sec)
        
        # 测试与 CARLA 服务端的连接
        self.__client.get_server_version()
        
    def disconnect(self) -> 'Context':
        """断开与 CARLA 服务端的连接."""
        self.__client = None
        
    def destroy_all_actors(self) -> None:
        """使用 CARLA 批处理 API 销毁所有注册在 Context 下的 Actor."""
        cmd = [carla.command.DestroyActor(x) for x in self.__actor_registry]
        self.client.apply_batch_sync(cmd)
