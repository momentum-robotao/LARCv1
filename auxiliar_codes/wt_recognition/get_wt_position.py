from math import cos, sin

import cv2
import numpy as np
from controller import Robot  # type: ignore
from ultralytics import YOLO  # type: ignore[import-untyped]

ROBOT_RADIUS = 0.0355
PI = 3.14159265359
DEGREE_IN_RAD = 0.0174533

Numeric = float | int
FOV = 0.5708
MODEL_PATH = (
    r"/home/gustavo/Documentos/competicoes/olimpiadas/OBR/LARC/"
    r"rescue2024/github/auxiliar_codes/wt_recognition/weights/best.pt"
)


def cyclic_angle(angle: Numeric) -> Numeric:
    while angle < 0:
        angle += 2 * PI
    while angle >= 2 * PI:
        angle -= 2 * PI

    return angle


model = YOLO(MODEL_PATH)

robot = Robot()

time_step = int(robot.getBasicTimeStep())

camera = robot.getDevice("camera2")
camera.enable(time_step)

imu = robot.getDevice("inertial_unit")
imu.enable(time_step)

gps = robot.getDevice("gps")
gps.enable(time_step)

DIST_CHANGE_MAPPER: dict[int, tuple[float, float]] = {}
SQRT_2 = 1.414213562373
ordered_deltas: list[tuple[float, float]] = [
    (0, 1),  # 0°
    (-1.0 / SQRT_2, 1.0 / SQRT_2),  # 45°
    (-1, 0),  # 90°
    (-1.0 / SQRT_2, -1.0 / SQRT_2),
    (0, -1),
    (1.0 / SQRT_2, -1.0 / SQRT_2),
    (1, 0),
    (1.0 / SQRT_2, 1.0 / SQRT_2),
]


def get_adjusted_angle() -> float:
    return imu.getRollPitchYaw()[2]


def round_if_almost_0(value: float):
    return 0 if round(value, 2) == 0 else value


def get_signal(val: float) -> int:
    if round_if_almost_0(val) == 0:
        return 0
    return 1 if val > 0 else -1


def get_signal_delta(delta: tuple[float, float]) -> tuple[int, int]:
    return (get_signal(delta[0]), get_signal(delta[1]))


def set_dist_change_mapper(
    delta_0_raw: tuple[float, float], delta_45_raw: tuple[float, float]
) -> None:
    delta_0 = get_signal_delta(delta_0_raw)
    delta_45 = get_signal_delta(delta_45_raw)

    pivot = 0
    reindexed_deltas = []
    for i in range(len(ordered_deltas)):
        if get_signal_delta(ordered_deltas[i]) == delta_0:
            pivot = i
    if (
        get_signal_delta(ordered_deltas[(pivot + 1) % (len(ordered_deltas))])
        == delta_45
    ):
        for j in range(pivot, len(ordered_deltas)):
            reindexed_deltas.append(ordered_deltas[j])
        for j in range(pivot):
            reindexed_deltas.append(ordered_deltas[j])
    else:
        for j in range(pivot, -1, -1):  # j, j-1, j-2...0
            reindexed_deltas.append(ordered_deltas[j])
        for j in range(len(ordered_deltas) - 1, pivot, -1):  # n-1, n-2...pivot+1
            reindexed_deltas.append(ordered_deltas[j])
    for ang in range(0, 360, 45):
        DIST_CHANGE_MAPPER[ang] = reindexed_deltas[int(ang / 45)]
    DIST_CHANGE_MAPPER[360] = DIST_CHANGE_MAPPER[0]


def setup_delta_coordinate_mapper(robot: Robot, maze: Maze):
    robot.imu.get_rotation_angle()
    initial_position = robot.gps.get_position()
    robot.run(Move("forward", 0.01, maze=maze, correction_move=True))
    delta_0_cord = robot.gps.get_position() - initial_position
    delta_0 = (delta_0_cord.x, delta_0_cord.y)
    robot.run(Move("backward", 0.01, maze=maze, correction_move=True))
    robot.run(Rotate("right", 45 * DEGREE_IN_RAD, correction_rotation=True))
    initial_position = robot.gps.get_position()
    robot.run(Move("forward", 0.01, maze=maze, correction_move=True))
    delta_45_cord = robot.gps.get_position() - initial_position
    delta_45 = (delta_45_cord.x, delta_45_cord.y)
    robot.run(Move("backward", 0.01, maze=maze, correction_move=True))
    robot.run(Rotate("left", 45 * DEGREE_IN_RAD, correction_rotation=True))
    set_dist_change_mapper(delta_0, delta_45)


class Lidar:
    def __init__(
        self,
        lidar_name: str = "lidar",
    ) -> None:
        self._lidar = robot.getDevice(lidar_name)
        self._lidar.enable(time_step)

        # LIDAR attributes
        self.horizontal_resolution = self._lidar.getHorizontalResolution()
        self.number_of_layers = self._lidar.getNumberOfLayers()
        self.min_range = self._lidar.getMinRange()
        self.max_range = self._lidar.getMaxRange()
        self.field_of_view = self._lidar.getFov()

    def _get_line_distances(self, line: int) -> list[float]:
        """
        :return: A list with the distances measured by lidar in the
        corresponding line.
        """
        return self._lidar.getLayerRangeImage(line).copy()

    def _get_measure_side_angle(self, measure_idx: int) -> float:
        angle = measure_idx * (self.field_of_view / self.horizontal_resolution)
        return angle

    def get_distances(self) -> list[float]:
        # Gets minimum to get the most similar to perpendicular measures
        distances = [float("inf") for _ in range(self.horizontal_resolution)]
        for line in range(self.number_of_layers):
            line_distances = self._get_line_distances(line)
            for col in range(self.horizontal_resolution):
                distances[col] = min(distances[col], line_distances[col] - ROBOT_RADIUS)

        return distances

    def get_distances_by_side_angle(self) -> dict[float, float]:
        distances = self.get_distances()
        distances_by_side_angle = dict()
        for measure_idx, dist in enumerate(distances):
            side_angle = round(self._get_measure_side_angle(measure_idx), 2)
            distances_by_side_angle[side_angle] = dist

        return distances_by_side_angle

    def get_distances_of_range(
        self, initial_side_angle: float, end_side_angle: float
    ) -> list[float]:
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

        return distances_of_range

    def get_side_distance(
        self,
        side: int | float,
        field_of_view: float = 10 * DEGREE_IN_RAD,
        remove_inf_measures: bool = True,
        use_min: bool = False,
    ) -> float:
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
        return average_distance


lidar = Lidar()


while robot.step(time_step) != -1:
    camera_image = camera.getImage()

    width = camera.getWidth()
    height = camera.getHeight()

    image_argb = np.frombuffer(camera_image, dtype=np.uint8).reshape((height, width, 4))

    image = np.copy(image_argb[:, :, :3])  # ? From ARGB to RGB
    results = model(image)

    class_names = model.names

    for result in results:
        for box in result.boxes:
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            conf = float(box.conf[0])
            class_id = int(box.cls[0])
            class_name = class_names[class_id]

            print(f"Found {class_name} in {x1},{y1} - {x2},{y2}")

            side_angle = ((x1 + x2) / 2) / width * FOV - FOV / 2
            print(f"ângulo: {side_angle}")
            angle = side_angle + get_adjusted_angle()

            positions = gps.getValues()
            x, _z, y = positions
            print(f"Posição atual: {x} {y}")

            dist = (
                lidar.get_side_distance(angle, field_of_view=2 * DEGREE_IN_RAD)
                + ROBOT_RADIUS
            )

            dx = dist * cos(angle)
            dy = dist * sin(angle)
            print(f"Deltas do WT: {dx=}, {dy=}")

            print(f"Está em: {x + dx}, {y + dy}")

            cv2.rectangle(image, (x1, y1), (x2, y2), (0, 255, 0), 2)
            text = f"{class_name}: {conf:.2f}"
            cv2.putText(
                image,
                text,
                (x1, y1 - 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 255, 0),
                2,
            )
    from time import sleep

    sleep(2)
