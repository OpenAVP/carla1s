from logging import Logger
from abc import ABC, abstractmethod

import carla

from ..context import Context, context_func


class Executor(ABC):
    """
    执行器基类. 定义了 CARLA 仿真进行过程的控制方法.
    """

    def __init__(self, ctx: Context):
        self._context = ctx
        self.test_connection = ctx.test_connection
        self._initialized = ctx.initialized

    @abstractmethod
    def __enter__(self):
        raise NotImplementedError

    @abstractmethod
    def __exit__(self, exc_type, exc_val, exc_tb):
        raise NotImplementedError

    @property
    def context(self) -> Context:
        """
        :return: 当前执行器的上下文.
        """
        return self._context

    @property
    def logger(self) -> Logger:
        """
        :return: 当前执行器的日志记录器.
        """
        return self.context.logger

    @abstractmethod
    @context_func
    def tick(self):
        raise NotImplementedError

    @abstractmethod
    @context_func
    def wait_real_seconds(self, seconds: float):
        raise NotImplementedError

    @abstractmethod
    @context_func
    def wait_sim_seconds(self, seconds: float):
        raise NotImplementedError

    @abstractmethod
    @context_func
    def wait_ticks(self, ticks: int):
        raise NotImplementedError

    @context_func
    def is_synchronous_mode(self) -> bool:
        """
        :return: 当前 CARLA 服务端是否处于同步模式.
        """
        settings: carla.WorldSettings = self.context.client.get_world().get_settings()
        return settings.synchronous_mode