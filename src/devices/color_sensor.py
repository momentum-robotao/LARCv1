import os

from controller import Robot as WebotsRobot  # type: ignore

from debugging import RobotLogger, System
from types_and_constants import RGB, SPECIAL_TILE_COLOR_MAPPER, ColoredSpecialTile

from .device import Device


class ColorSensor(Device):
    def __init__(
        self,
        robot: WebotsRobot,
        logger: RobotLogger,
        color_sensor_name: str = "colour_sensor",
        time_step: int = int(os.getenv("TIME_STEP", 32)),
    ) -> None:
        self.logger = logger

        self._color_sensor = robot.getDevice(color_sensor_name)
        self._color_sensor.enable(time_step)

    def get_color(self) -> bytes:
        color = self._color_sensor.getImage()
        self.logger.info(f"Cor reconhecida: {color}", System.color_sensor_measures)
        return color

    def get_RGB_color(self) -> RGB:
        image = self._color_sensor.getImage()

        red = self._color_sensor.imageGetRed(image, 1, 0, 0)
        green = self._color_sensor.imageGetGreen(image, 1, 0, 0)
        blue = self._color_sensor.imageGetBlue(image, 1, 0, 0)
        color = RGB(red, green, blue)
        self.logger.info(f"Cor RGB reconhecida: {color}", System.color_sensor_measures)
        return color

    def check_colored_special_tile(self) -> ColoredSpecialTile | None:
        color = self.get_RGB_color()
        self.logger.info(f"Cor do chão: {color}", System.check_tile_color)

        for test_color, special_tile_type in SPECIAL_TILE_COLOR_MAPPER.items():
            if test_color == color:
                self.logger.info(f"É {special_tile_type}", System.check_tile_color)
                return special_tile_type
        self.logger.info("Não é nada", System.check_tile_color)
        return None
