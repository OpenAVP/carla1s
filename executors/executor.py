from logging import Logger
from abc import ABC, abstractmethod

import carla

from ..context import Context


class Executor(ABC):
    """执行器基类. 定义了 CARLA 仿真进行过程的控制方法.
    """

    def __init__(self, ctx: Context):
        self._context = ctx
        self.test_connection = ctx.test_connection

    @abstractmethod
    def __enter__(self):
        raise NotImplementedError

    @abstractmethod
    def __exit__(self, exc_type, exc_val, exc_tb):
        raise NotImplementedError

    @property
    def context(self) -> Context:
        """当前执行器的上下文"""
        return self._context

    @property
    def logger(self) -> Logger:
        """当前执行器的日志记录器."""
        return self.context.logger
    
    @property
    def is_synchronous_mode(self) -> bool:
        """当前 CARLA 服务端是否处于同步模式."""
        settings: carla.WorldSettings = self.context.client.get_world().get_settings()
        return settings.synchronous_mode

    @abstractmethod
    def tick(self) -> None:
        """要求 CARLA 仿真器进行一次 ``tick()``."""
        raise NotImplementedError

    @abstractmethod
    def spin(self, show_progress: bool = False) -> None:
        """阻塞当前线程, 直到接收到 KeyboardInterrupt 信号.

        Args:
            show_progress (bool, optional): 是否打印等待进度日志. 默认为 False.
        """
        raise NotImplementedError

    @abstractmethod
    def wait_real_seconds(self, seconds: float, show_progress: bool = False) -> None:
        """等待一定真实世界的秒数

        Args:
            seconds (float): 需要等待的真实时间的秒数
            show_progress (bool, optional): 是否打印等待进度日志. 默认为 False.
        """
        raise NotImplementedError

    @abstractmethod
    def wait_sim_seconds(self, seconds: float, show_progress: bool = False) -> None:
        """等待一定仿真世界的秒数

        Args:
            seconds (float): 需要等待的仿真时间的秒数
            show_progress (bool, optional): 是否打印等待进度日志. 默认为 False.
        """
        raise NotImplementedError

    @abstractmethod
    def wait_ticks(self, ticks: int, show_progress: bool = False):
        """等待一定的 Tick 次数.

        Args:
            ticks (int): CARLA 服务端执行的 Tick 数
            show_progress (bool, optional): 是否打印等待进度日志. 默认为 False.
        """
        raise NotImplementedError
