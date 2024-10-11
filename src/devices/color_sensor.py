import os

from debugging import DebugInfo, System
from types_and_constants import DEBUG, HOLE_COLOR, RGB

from .device import Device


class ColorSensor(Device):
    def __init__(
        self,
        robot,
        debug_info: DebugInfo,
        color_sensor_name: str = "colour_sensor",
        time_step: int = int(os.getenv("TIME_STEP", 32)),
    ) -> None:
        self.debug_info = debug_info

        self._color_sensor = robot.getDevice(color_sensor_name)
        self._color_sensor.enable(time_step)

    def get_color(self) -> bytes:
        color = self._color_sensor.getImage()
        if DEBUG:
            self.debug_info.send(
                f"Cor reconhecida: {color}", System.color_sensor_measures
            )
        return color

    def get_RGB_color(self) -> RGB:
        image = self._color_sensor.getImage()

        red = self._color_sensor.imageGetRed(image, 1, 0, 0)
        green = self._color_sensor.imageGetGreen(image, 1, 0, 0)
        blue = self._color_sensor.imageGetBlue(image, 1, 0, 0)
        color = RGB(red, green, blue)
        if DEBUG:
            self.debug_info.send(
                f"Cor RGB reconhecida: {color}", System.color_sensor_measures
            )
        return color

    def has_hole(self, hole_color: bytes = HOLE_COLOR) -> bool:
        color = self.get_color()
        has_hole = color == hole_color
        if DEBUG:
            self.debug_info.send(
                f"Buraco {'' if has_hole else 'não '}reconhecido.",
                System.color_sensor_detections,
            )
        return has_hole
