import logging
import time
from threading import Thread
from typing import Union

class ProgressLogger(Thread):
    """
    进度日志记录器
    """

    def __init__(self,
                 logger: callable(logging.Logger),
                 header: str,
                 unit: str,
                 total: Union[int, float],
                 period: float = 1.0):
        super().__init__()
        self.daemon = True

        self.logger = logger
        self.header = header
        self.unit = unit
        self.total = total
        self.current = 0
        self.flag_show_log = True
        self.period = period

    def run(self):
        while self.flag_show_log:
            time.sleep(self.period)

            # 睡眠完立即再次检查 show_log 标志, 防止在睡眠期间被设置为 False
            if not self.flag_show_log:
                break

            # 打印日志
            progress = self.current / self.total
            msg = f'{self.header}: {self.current} / {self.total} {self.unit} ({progress:.2%})'
            self.logger(msg)

    def start(self) -> 'ProgressLogger':
        self.flag_show_log = True
        super().start()
        return self

    def stop(self) -> 'ProgressLogger':
        self.flag_show_log = False
        return self

    def update(self, value: Union[int, float]) -> 'ProgressLogger':
        self.current = value
        return self