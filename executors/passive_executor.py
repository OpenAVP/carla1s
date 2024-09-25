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

    def spin(self, show_progress: bool = False):
        self.logger.info('Continue spin for KeyboardInterrupt signal.')
        logger = self.logger.info if show_progress else self.logger.debug
        time_begin_spin = time.perf_counter()
        try:
            while True:
                time.sleep(1)
                time_passed = time.perf_counter() - time_begin_spin
                logger(f'Continue spinning: keep {time_passed:.4f} seconds.')
        except KeyboardInterrupt:
            self.logger.info('Received KeyboardInterrupt signal, stop spinning.')

    def wait_real_seconds(self, seconds: float, show_progress: bool = False):
        self.logger.info(f'Waiting for {seconds} seconds in real world.')
        time_begin_wait = time.perf_counter()
        logger = self.logger.info if show_progress else self.logger.debug
        flag_show_log = True

        def _progress_logger():
            while flag_show_log:
                time.sleep(1)
                if not flag_show_log:
                    break
                now_wait = time.perf_counter() - time_begin_wait
                logger(f'Waiting for {seconds} seconds in real world: progress: {now_wait:.3f} / {seconds:.3f}' )

        thread_log = Thread(target=_progress_logger, daemon=True)
        thread_log.start()

        time.sleep(seconds)
        flag_show_log=False
        return

    def wait_sim_seconds(self, seconds: float, show_progress: bool = False):
        self.logger.info(f'Waiting for {seconds} seconds in simulation world')
        time_begin_wait = self.context.world.get_snapshot().timestamp.elapsed_seconds
        logger = self.logger.info if show_progress else self.logger.debug
        flag_show_log = True

        def _progress_logger():
            while flag_show_log:
                time.sleep(1)
                if not flag_show_log:
                    break
                now_wait = self.context.world.get_snapshot().timestamp.elapsed_seconds - time_begin_wait
                logger(f'Waiting for {seconds} seconds in simulation world: progress: {now_wait:.3f} / {seconds:.3f}' )

        thread_log = Thread(target=_progress_logger, daemon=True)
        thread_log.start()

        while self.context.world.get_snapshot().timestamp.elapsed_seconds - time_begin_wait < seconds:
            time.sleep(0.001)  # 防止 CPU 过载
        flag_show_log = False
        return

    def wait_ticks(self, ticks: int, show_progress: bool = False):
        self.logger.info(f'Waiting for {ticks} ticks.')
        count_wait_ticks = 0
        logger = self.logger.info if show_progress else self.logger.debug
        flag_show_log = True

        def _progress_logger():
            while flag_show_log:
                time.sleep(1)
                if not flag_show_log:
                    break
                logger(f'Waiting for {ticks} ticks: progress: {count_wait_ticks} / {ticks}')

        thread_log = Thread(target=_progress_logger, daemon=True)
        thread_log.start()

        for _ in range(ticks):
            count_wait_ticks += 1
            self.context.world.wait_for_tick()
        flag_show_log = False
        return