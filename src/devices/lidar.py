import os
from typing import Literal
import numpy as np
import math 

from controller import Robot as WebotsRobot  # type: ignore

from debugging import DebugInfo, System
from helpers import cyclic_angle
from types_and_constants import (
    CENTRAL_SIDE_ANGLE_OF_SIDE,
    DEBUG,
    DEGREE_IN_RAD,
    EXPECTED_WALL_DISTANCE,
    KP,
    MAX_WALL_DISTANCE,
    ROBOT_RADIUS,
    WALL_COLLISION_DISTANCE,
    Numeric,
    Side,
)

from .device import Device


class Lidar(Device):
    def __init__(
        self,
        robot: WebotsRobot,
        debug_info: DebugInfo,
        lidar_name: str = "lidar",
        time_step: int = int(os.getenv("TIME_STEP", 32)),
    ) -> None:
        self._lidar = robot.getDevice(lidar_name)
        self._lidar.enable(time_step)

        # LIDAR attributes
        self.horizontal_resolution = self._lidar.getHorizontalResolution()
        self.number_of_layers = self._lidar.getNumberOfLayers()
        self.min_range = self._lidar.getMinRange()
        self.max_range = self._lidar.getMaxRange()
        self.field_of_view = self._lidar.getFov()

        self.debug_info = debug_info

    def _get_line_distances(self, line: int) -> list[float]:
        """
        :return: A list with the distances measured by lidar in the
        corresponding line.
        """
        return self._lidar.getLayerRangeImage(line).copy()

    def _get_measure_side_angle(self, measure_idx: int) -> float:
        angle = measure_idx * (self.field_of_view / self.horizontal_resolution)
        return angle

    # TODO-: unify
    def _get_range_image_(self):
        return self._lidar.getRangeImage()

    def show_initialization_information(self) -> None:
        if DEBUG:
            self.debug_info.send("\nSobre o LIDAR", System.initialization)
            self.debug_info.send(
                f"\tA distância percebida é [{self.min_range};{self.max_range}]",
                System.initialization,
            )
            self.debug_info.send(
                f"\tHá {self.horizontal_resolution} medições na horizontal "
                f"e {self.number_of_layers} layers na vertical",
                System.initialization,
            )
            self.debug_info.send(
                f"\tO FOV, entre 0 e 2*PI é: {self.field_of_view}",
                System.initialization,
            )

    def get_distances(self) -> list[float]:
        """
        Gets the distance in each angle of the robot measured in
        meters by lidar and corresponds to the distances from the
        robot sides.

        OBS:
        - Zero angle depends of the value set in the robot file
        (probably, it is the front of the robot);
        - Angle of measures are side angles.
        - The angle of the measurements increase clockwise.
        """
        # Gets minimum to get the most similar to perpendicular measures
        distances = [float("inf") for _ in range(self.horizontal_resolution)]
        for line in range(self.number_of_layers):
            line_distances = self._get_line_distances(line)
            for col in range(self.horizontal_resolution):
                distances[col] = min(distances[col], line_distances[col] - ROBOT_RADIUS)

        if DEBUG:
            self.debug_info.send(f"{distances=}", System.lidar_measures)
        return distances
    
    
    def get_distances_by_side_angle(self) -> dict[float, float]: 
        """
        :return: A dict where `dict[angle] = distance measured in this side angle`.
        """
        distances = self.get_distances()
        distances_by_side_angle = dict()
        for measure_idx, dist in enumerate(distances):
            side_angle = round(self._get_measure_side_angle(measure_idx), 2)
            distances_by_side_angle[side_angle] = dist

        if DEBUG:
            self.debug_info.send(
                f"Medições da distância em função do ângulo lateral: {distances_by_side_angle}",
                System.lidar_measures,
            )

        return distances_by_side_angle

    
    def get_distances_by_side_angle_AUGUSTO(self) -> dict[float, float]:
        """
        :return: Um dicionário com distâncias brutas do LiDAR (ângulo em radianos → distância em metros)
        """
        distances_by_side_angle = {}
        num_layers = self.number_of_layers
        horizontal_res = 512  # Confirmar no Webots
        total_points = horizontal_res * num_layers
        range_image = self._lidar.getRangeImage()

        for angle_deg in range(360):
            n = (angle_deg * horizontal_res) // 360
            final_distance = float('inf')
            
            # Procura a primeira medida válida em qualquer camada
            for layer in range(num_layers):
                idx = n + (layer * horizontal_res)
                if idx < len(range_image):
                    distance = range_image[idx]
                    if distance != float('inf'):
                        final_distance = distance  # Mantém valor bruto do sensor
                        break

            angle_rad = round(math.radians(angle_deg), 4)
            distances_by_side_angle[angle_rad] = round(final_distance, 4)

        if DEBUG:
            self.debug_info.send(
                f"Medições LiDAR brutas: {distances_by_side_angle}",
                System.lidar_measures,
            )
        
        return distances_by_side_angle
    

    def get_distances_of_range(
        self, initial_side_angle: float, end_side_angle: float
    ) -> list[float]:
        """
        Gets the distance in each angle of the robot measured in
        meters by lidar and corresponds to the distances from the
        robot sides. The measures corresponds only to the ones of
        the angles of specified range.
        OBS:
        - Zero angle depends of the value set in the robot file
        (probably, it is the front of the robot)
        - If `end_angle < initial_angle`, the range of measures are
        get from angles increasing and coming back to angle 0 and then
        increasing until `initial_angle`.
        Ex: if angles of measures were [1, 2, 3, 4, 5], `initial_angle = 4`
        and `end_angle = 2`, the measures would correspond to these angles:
        [4, 5, 1, 2]
        - The angle of the measurements increase clockwise.
        """
        distances_by_side_angle = self.get_distances_by_side_angle()
        distances_of_range = []
        if initial_side_angle <= end_side_angle:
            for side_angle, dist in distances_by_side_angle.items():
                if initial_side_angle <= side_angle and side_angle <= end_side_angle:
                    distances_of_range.append(dist)
        else:
            for side_angle, dist in distances_by_side_angle.items():
                if initial_side_angle <= side_angle:
                    distances_of_range.append(dist)
            for side_angle, dist in distances_by_side_angle.items():
                if side_angle <= end_side_angle:
                    distances_of_range.append(dist)

        if DEBUG:
            self.debug_info.send(
                f"Pegas medições do intervalo cíclico baseado nos ângulos: "
                f"[{initial_side_angle};{end_side_angle}]. Medições: {distances_of_range}",
                System.lidar_range_measures,
            )

        return distances_of_range

    def get_side_distance(
        self,
        side: Side | Numeric,
        field_of_view: float = 10 * DEGREE_IN_RAD,
        remove_inf_measures: bool = True,
        use_min: bool = False,
    ) -> float:
        """
        IMPORTANT! Note that angles in `side` must be side angles.

        :param side: If it is a `Side`, the measures are centralized in
        this side of the robot, but if it is a `float` or `int` it is
        centralized on this side angle, that should be from [0;2*PI] (in rad).

        :return: The average distance from `field_of_view` centralized in
        a side of the robot. Excluding INF values.
        """
        if isinstance(side, str):
            central_angle = CENTRAL_SIDE_ANGLE_OF_SIDE[side]
            start_angle = central_angle * DEGREE_IN_RAD - field_of_view / 2
            end_angle = central_angle * DEGREE_IN_RAD + field_of_view / 2
        elif isinstance(side, (int, float)):
            start_angle = side - field_of_view / 2
            end_angle = side + field_of_view / 2

        distances = self.get_distances_of_range(
            cyclic_angle(start_angle), cyclic_angle(end_angle)
        )
        if min(distances) != float("inf") and remove_inf_measures:
            distances = [dist for dist in distances if dist != float("inf")]

        if len(distances) == 0:
            return float("inf")

        average_distance = sum(distances) / len(distances)

        if use_min:
            average_distance = list(sorted(distances))[0]
            if len(distances) >= 3:
                average_distance = list(sorted(distances))[2]

        if DEBUG:
            self.debug_info.send(
                f"Medidas correspondentes à {side} "
                f"(com campo de visão de {field_of_view} rad): {distances}; "
                f"Dando média de: {average_distance}m",
                System.lidar_side_measures,
            )

        return average_distance

    def has_wall(
        self,
        side: Literal["front", "back", "left", "right"],
        max_wall_distance: float = MAX_WALL_DISTANCE,
        use_min: bool = False,
        field_of_view: float = 10 * DEGREE_IN_RAD,
    ) -> bool:
        """
        :return: If there is a wall in the actual tile, assuming that
        the robot is on the center of the tile on the corresponding axis.
        """
        side_distance = self.get_side_distance(
            side, field_of_view=field_of_view, use_min=use_min
        )
        has_wall = side_distance <= max_wall_distance

        if DEBUG:
            self.debug_info.send(
                f"{'Tem' if has_wall else 'Não tem'} parede em {side}. "
                f"Limite de parede: {max_wall_distance}.",
                System.lidar_wall_detection,
            )

        return has_wall

    def wall_collision(
        self,
        side: Literal["front", "back"],
        wall_collision_dist: float = WALL_COLLISION_DISTANCE,
    ) -> bool:
        wall_dist = self.get_side_distance(side, use_min=True)
        wall_collision = wall_dist <= wall_collision_dist
        if DEBUG:
            if wall_collision:
                self.debug_info.send(
                    f"Colisão com parede detectada há: {wall_dist}m",
                    System.lidar_wall_detection,
                )
        return wall_collision

    def get_rotation_angle_error(
        self,
        expected_wall_distance: float = EXPECTED_WALL_DISTANCE,
        kp: float = KP,
    ) -> float:
        rotation_angle_error = 0.0
        if self.has_wall("left"):
            rotation_angle_error = (
                self.get_side_distance("left") - expected_wall_distance
            ) * kp
        if self.has_wall("right"):
            rotation_angle_error = (
                (self.get_side_distance("right") - expected_wall_distance) * kp * -1
            )

        if DEBUG:
            self.debug_info.send(
                "Erro do ângulo de rotação do robô para ser corrigido: "
                f"{rotation_angle_error}. Com {kp=}, com distância alvo "
                f"da parede de: {expected_wall_distance}",
                System.rotation_angle_correction,
            )

        return rotation_angle_error
