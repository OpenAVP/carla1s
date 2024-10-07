from enum import Enum

from ..actors import ActorTemplate as Template


class ActorTemplates(Enum):
    DEMO_VEHICLE = Template('vehicle.tesla.model3', color='255,255,255')

