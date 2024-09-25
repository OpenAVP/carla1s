import carla
import time
from threading import Event, Thread

from ..context import Context, context_func
from ..exceptions import ExecutorError
from ..logger import ProgressLogger
from .executor import Executor


class ManualExecutor(Executor):
    """
    手动执行器, 执行器将会通过 ``tick()`` 控制 CARLA 仿真进行进行过程.

    手动执行器适用于：

    - 需要确保 CARLA 的数据传输和处理过程的正确性时
    - 对仿真器在环性能不敏感时
    """

    def __init__(self,
                 context: Context,
                 fixed_delta_seconds: float = 0.05,
                 min_tick_wait_seconds: float = 0.0):
        """
        :param context: CARLA 的上下文实例
        :param fixed_delta_seconds: 每次执行 ``tick()`` 时的仿真器的固定步长, 单位为秒.
        :param min_tick_wait_seconds: 每次执行 ``tick()`` 时的最小等待时间, 单位为秒.
            设置为 ``0.0`` 时取与 ``fixed_delta_seconds`` 相同的值.
            设置为负值时将不进行任何等待 (可能会造成 CARLA 崩溃）.
        """
        super().__init__(context)
        self._fixed_delta_seconds = fixed_delta_seconds
        self._set_min_tick_wait_seconds(min_tick_wait_seconds)

        self._last_tick = 0.0

    def __enter__(self):
        self.logger.info('Manual Executor begin.')

        # 如果 CARLA 服务端已经处于同步模式, 则引发报错并阻止进一步动作
        if self.is_synchronous_mode():
            msg = ('CARLA is in synchronous mode, and the manual executor is activated in current context. '
                   'Please deactivate the synchronous mode first.')
            self.logger.critical(msg)
            raise ExecutorError(msg)

        # 启动同步模式
        self.set_synchronous_mode(True)
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        # 退出时关闭同步模式
        self.set_synchronous_mode(False)

        self.logger.info('Manual Executor exit.')

    @property
    def fixed_delta_seconds(self) -> float:
        """
        :return: 每次执行 ``tick()`` 时的 CARLA 服务端执行的固定步长, 单位为秒.
        """
        return self._fixed_delta_seconds

    @fixed_delta_seconds.setter
    def fixed_delta_seconds(self, value: float):
        old_val = self._fixed_delta_seconds
        self._fixed_delta_seconds = value
        new_val = self._fixed_delta_seconds
        self.logger.warning(f'Fixed delta seconds changed from {old_val} to {new_val}, input: {value}.')

    @property
    def min_tick_wait_seconds(self) -> float:
        """
        :return: 每次执行 ``tick()`` 时的最小等待时间, 单位为秒.
        """
        return self._min_tick_wait_seconds

    @min_tick_wait_seconds.setter
    def min_tick_wait_seconds(self, value: float):
        old_val = self._min_tick_wait_seconds
        self._set_min_tick_wait_seconds(value)
        new_val = self._min_tick_wait_seconds
        self.logger.warning(f'Min tick wait seconds changed from {old_val} to {new_val}, input: {value}.')

    def _set_min_tick_wait_seconds(self, value: float):
        """
        设置最小等待时间.
        :param value: 如果输入值小于 0.0, 则设置为 0.0;
            如果输入值等于 0.0, 则设置为 ``fixed_delta_seconds`` 的值;
            否则设置为输入值.
        """
        if value < 0.0:
            self._min_tick_wait_seconds = 0.0
        elif value == 0.0:
            self._min_tick_wait_seconds = self.fixed_delta_seconds
        else:
            self._min_tick_wait_seconds = value

    def tick(self):
        # 计算需要等待的时间
        delta_tick_seconds = time.perf_counter() - self._last_tick
        wait_seconds = self.min_tick_wait_seconds - delta_tick_seconds

        # 如果需要等待, 则进行等待
        if wait_seconds > 0.0:
            time.sleep(wait_seconds)

        # 执行 tick
        self.context.world.tick()
        self._last_tick = time.perf_counter()
        return

    def spin(self, show_progress: bool = False):
        self.logger.info('Continue spin until KeyboardInterrupt signal received (press Ctrl-C to stop).')
        logger = self.logger.info if show_progress else self.logger.debug
        time_spin_begin = time.perf_counter()
        count_tick = 0
        flag_show_log = True

        def _progress_logger():
            while flag_show_log:
                time_last_log = time.perf_counter()
                count_tick_last_log = count_tick
                time.sleep(1)
                if not flag_show_log:
                    break
                delta_time = time.perf_counter() - time_spin_begin
                fps = (count_tick - count_tick_last_log) / (time.perf_counter() - time_last_log)
                logger(f'Continue spinning: keep {delta_time:.4f} seconds, and {count_tick} ticks. FPS: {fps:.2f} Hz.')

        thread_log = Thread(target=_progress_logger, daemon=True)
        thread_log.start()

        try:
            while True:
                self.tick()
                count_tick += 1
        except KeyboardInterrupt:
            flag_show_log = False
            self.logger.warning('Received KeyboardInterrupt signal, stop spinning.')

    def wait_real_seconds(self, seconds: float, show_progress: bool = False):
        self.logger.info(f'Waiting for {seconds} real seconds.')
        time_begin_wait = time.perf_counter()
        time_spent = 0

        progress = ProgressLogger(
            logger=self.logger.info if show_progress else self.logger.debug,
            header='Waiting for real seconds',
            unit='seconds',
            total=seconds,
        ).start()

        while time_spent < seconds:
            self.tick()
            time_spent = time.perf_counter() - time_begin_wait
            progress.update(time_spent)
        else:
            progress.stop()

    def wait_sim_seconds(self, seconds: float, show_progress: bool = False):
        self.logger.info(f'Waiting for {seconds} simulation seconds.')
        time_begin_wait = self.context.world.get_snapshot().timestamp.elapsed_seconds
        time_spent = 0

        progress = ProgressLogger(
            logger=self.logger.info if show_progress else self.logger.debug,
            header='Waiting for simulation seconds',
            unit='seconds',
            total=seconds,
        ).start()

        while time_spent < seconds:
            self.tick()
            time_spent = self.context.world.get_snapshot().timestamp.elapsed_seconds - time_begin_wait
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

        while count_wait_ticks < ticks:
            self.tick()
            count_wait_ticks += 1
            progress.update(count_wait_ticks)
        else:
            progress.stop()

    @context_func
    def set_synchronous_mode(self, option: bool):
        # 调整同步模式设定
        setting: carla.WorldSettings = self.context.world.get_settings()
        setting.synchronous_mode = option
        setting.fixed_delta_seconds = self._fixed_delta_seconds if option else 0.0

        # 应用同步模式设定并强制进行一次 tick 以应用
        self.context.world.apply_settings(setting)
        self.context.world.tick()
        self.logger.info(f'Set synchronous mode to {option}.')
