from abc import ABC, abstractmethod

from debugging import DebugInfo


class Device(ABC):
    """
    Classes responsible for interacting with the robot. It includes classes
    that get data from robot (they are sensors) and classes that execute
    actions on the robot (they are actuators).
    """

    @abstractmethod
    def __init__(self, robot, debug_info: DebugInfo, *args, **kwargs) -> None:
        pass
