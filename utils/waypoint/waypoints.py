import numpy as np
from typing import List, Union

from ...tf import Transform


class Waypoints:
    def __init__(self, 
                 sequence: Union[List[Transform], np.ndarray],
                 *,
                 delta_seconds: float,
                 forward: bool = True,
                 keep_last: bool = False):
        self._delta_seconds = delta_seconds
        self._sequence = self._smoothing(sequence)
        self._iter_index = 0
        self.keep_last = keep_last
        self.forward = forward
        
    def __len__(self):
        # TODO: CONFIRM IT!
        return len(self._sequence)
    
    def __getitem__(self, index: int):
        # TODO: CONFIRM IT!
        return self._sequence[index]
    
    def __iter__(self):
        return self
    
    def __next__(self) -> Transform:
        # 如果迭代器行进至终点, 且没有显示的声明 keep_last, 则抛出 StopIteration 异常以终止迭代
        if self._iter_index > len(self):
            if not self.keep_last:
                raise StopIteration
            else:
                self._iter_index = len(self)

        waypoint = self._sequence[self._iter_index]
        self._iter_index += 1
        
        # TODO: Convert to Transform Object
        
        return waypoint

    @property
    def data(self):
        return self._data

    def reset(self):
        self._iter_index = 0

    def next(self) -> Transform:
        return next(self)

    def _smoothing(self, sequence: np.ndarray) -> np.ndarray:
        sequence = self._up_sampling(sequence)
        sequence = self._pure_pursuit(sequence)
        return sequence
    
    def _up_sampling(self, sequence: np.ndarray) -> np.ndarray:
        raise NotImplementedError
    
    def _pure_pursuit(self, sequence: np.ndarray) -> np.ndarray:
        raise NotImplementedError

    @classmethod
    def from_file(cls, 
                  file_path: str, 
                  *, 
                  delta_seconds: float, 
                  forward: bool = True, 
                  keep_last: bool = False) -> 'Waypoints':
        raise NotImplementedError
