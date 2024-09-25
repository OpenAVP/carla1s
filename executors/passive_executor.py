import time
from threading import Thread

from .executor import Executor


class PassiveExecutor(Executor):
    """
    从动执行器, 执行器将不会控制 CARLA 仿真进行过程, 只会等待仿真进行.

    从动执行器适用于：

    - CARLA 服务端处于异步模式时
    - CARLA 服务端处于同步模式，但本客户端不控制仿真进行过程时
    """

    def __enter__(self):
        self.logger.info('Passive Executor begin.')

        # 如果 CARLA 服务端处于同步模式则进行一次告警
        if self.is_synchronous_mode():
            self.logger.warning('CARLA is in synchronous mode, '
                                'and the passive executor is activated in current client. '
                                'Make sure there is another CARLA client running the tick.')

        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.logger.info('Passive Executor exit.')

    def tick(self):
        self.logger.warning(f'You are trying to tick the CARLA server in Passive Executor, '
                            f'so nothing will happen.')

    def wait_real_seconds(self, seconds: float, show_progress: bool = False):
        """
        等待一定真实世界的秒数.
        :param seconds: 需要等待的真实时间的秒数.
        :param show_progress: 打印等待进度日志. True 时在 INFO 级别打印, False 时在 DEBUG 级别打印.
        """
        self.logger.info(f'Waiting for {seconds} seconds in real world.')
        start_wait = time.perf_counter()
        logger = self.logger.info if show_progress else self.logger.debug
        show = True

        def _progress_logger():
            while show:
                time.sleep(1)
                if not show:
                    break
                now_wait = time.perf_counter() - start_wait
                logger(f'Waiting for {seconds} seconds in real world: progress: {now_wait:.3f} / {seconds:.3f}' )

        thread = Thread(target=_progress_logger, daemon=True)
        thread.start()

        time.sleep(seconds)
        show=False
        return

    def wait_sim_seconds(self, seconds: float, show_progress: bool = False):
        """
        等待一定的仿真世界的秒数.
        :param seconds: 需要等待的仿真时间的秒数.
        :param show_progress: 打印等待进度日志. True 时在 INFO 级别打印, False 时在 DEBUG 级别打印.
        """
        self.logger.info(f'Waiting for {seconds} seconds in simulation world')
        start_wait = self.context.world.get_snapshot().timestamp.elapsed_seconds
        logger = self.logger.info if show_progress else self.logger.debug
        show = True

        def _progress_logger():
            while show:
                time.sleep(1)
                if not show:
                    break
                now_wait = self.context.world.get_snapshot().timestamp.elapsed_seconds - start_wait
                logger(f'Waiting for {seconds} seconds in simulation world: progress: {now_wait:.3f} / {seconds:.3f}' )

        thread = Thread(target=_progress_logger, daemon=True)
        thread.start()

        while self.context.world.get_snapshot().timestamp.elapsed_seconds - start_wait < seconds:
            time.sleep(0.001)  # 防止 CPU 过载
        show = False
        return

    def wait_ticks(self, ticks: int, show_progress: bool = False):
        """
        等待一定的 Tick 次数.
        :param ticks: CARLA 服务端执行的 Tick 数
        :param show_progress: 打印等待进度日志. True 时在 INFO 级别打印, False 时在 DEBUG 级别打印.
        """
        self.logger.info(f'Waiting for {ticks} ticks.')
        wait_count = 0
        logger = self.logger.info if show_progress else self.logger.debug
        show = True

        def _progress_logger():
            while show:
                time.sleep(1)
                if not show:
                    break
                logger(f'Waiting for {ticks} ticks: progress: {wait_count} / {ticks}' )

        thread = Thread(target=_progress_logger, daemon=True)
        thread.start()

        for _ in range(ticks):
            wait_count += 1
            self.context.world.wait_for_tick()
        show = False
        return