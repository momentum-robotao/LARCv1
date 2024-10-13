from abc import ABC, abstractmethod

from controller import Robot as WebotsRobot  # type: ignore

from debugging import DebugInfo


class Device(ABC):
    """
    Classes responsible for interacting with the robot. It includes classes
    that get data from robot (they are sensors) and classes that execute
    actions on the robot (they are actuators).
    """

    @abstractmethod
    def __init__(
        self, robot: WebotsRobot, debug_info: DebugInfo, *args, **kwargs
    ) -> None:
        pass
