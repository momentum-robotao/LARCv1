import math

import pytest

from utils import cyclic_angle_difference

DEGREE_IN_RAD = math.pi / 180


@pytest.mark.parametrize(
    "measured_angle_degree, expected_angle_degree, expected_difference_degree",
    [
        (0, 10, -10),
        (10, 0, 10),
        (350, 10, -20),
        (10, 350, 20),
        (0, 360, 0),
    ],
)
def test_cyclic_angle_difference(
    measured_angle_degree, expected_angle_degree, expected_difference_degree
):
    measured_angle = measured_angle_degree * DEGREE_IN_RAD
    expected_angle = expected_angle_degree * DEGREE_IN_RAD
    expected_difference = expected_difference_degree * DEGREE_IN_RAD
    result = cyclic_angle_difference(measured_angle, expected_angle)
    assert result == pytest.approx(
        expected_difference, abs=1e-5
    ), f"Failed for {measured_angle_degree=}, {expected_angle_degree=}"
