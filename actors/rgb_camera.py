import carla
import numpy as np
from .sensor import Sensor


class RgbCamera(Sensor):
    
    def _callback(self, image: carla.Image):
        img = np.frombuffer(image.raw_data, dtype=np.uint8)
        img = img.reshape(
            (image.height, image.width, img.shape[0] // image.height // image.width)
        )
        img = img[:, :, :3]
        img = img[:, :, ::-1]
        self.logger.info(f'Received image with shape {img.shape}')
        self.data = img
        self.on_data_ready.set()
    