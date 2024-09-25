import time

from .executor import Executor
from ..logger import ProgressLogger


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

    def spin(self, show_progress: bool = False):
        self.logger.info('Continue spin until KeyboardInterrupt signal received (press Ctrl-C to stop).')
        logger = self.logger.info if show_progress else self.logger.debug
        time_begin_spin = time.perf_counter()
        try:
            while True:
                time.sleep(1)
                time_passed = time.perf_counter() - time_begin_spin
                logger(f'Continue spinning: keep {time_passed:.4f} seconds.')
        except KeyboardInterrupt:
            self.logger.warning('Received KeyboardInterrupt signal, stop spinning.')

    def wait_real_seconds(self, seconds: float, show_progress: bool = False):
        self.logger.info(f'Waiting for {seconds} seconds in real world.')
        time_begin_wait = time.perf_counter()

        progress = ProgressLogger(
            logger=self.logger.info if show_progress else self.logger.debug,
            header='Waiting for seconds',
            unit='seconds',
            total=seconds,
        ).start()

        time_spent = time.perf_counter() - time_begin_wait
        while time_spent < seconds:
            time.sleep(0.001)
            progress.update(time_spent)
        else:
            progress.stop()

    def wait_sim_seconds(self, seconds: float, show_progress: bool = False):
        self.logger.info(f'Waiting for {seconds} seconds in simulation world')
        time_begin_wait = self.context.world.get_snapshot().timestamp.elapsed_seconds

        progress = ProgressLogger(
            logger=self.logger.info if show_progress else self.logger.debug,
            header='Waiting for seconds',
            unit='seconds',
            total=seconds,
        ).start()

        time_spent = self.context.world.get_snapshot().timestamp.elapsed_seconds - time_begin_wait
        while time_spent < seconds:
            time.sleep(0.001)  # 防止 CPU 过载
            progress.update(time_spent)
        else:
            progress.stop()

    def wait_ticks(self, ticks: int, show_progress: bool = False):
        self.logger.info(f'Waiting for {ticks} ticks.')
        count_wait_ticks = 0

        progress = ProgressLogger(
            logger=self.logger.info if show_progress else self.logger.debug,
            header='Waiting for ticks',
            unit='ticks',
            total=ticks,
        ).start()

        for _ in range(ticks):
            count_wait_ticks += 1
            self.context.world.wait_for_tick()
        else:
            progress.stop()
