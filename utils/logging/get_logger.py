import logging
from rich.logging import RichHandler

from .project_formatter import ProjectFormatter


def get_logger(name: str) -> logging.Logger:
    logger = logging.getLogger(name)
    handler = RichHandler(rich_tracebacks=True, markup=False)
    handler.setFormatter(ProjectFormatter('[%(shortname)s] %(message)s'))
    logger.handlers.clear()
    logger.addHandler(handler)
    return logger