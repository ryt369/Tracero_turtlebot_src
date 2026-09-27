import pytest

from sensor_msgs.msg import LaserScan

from tracero_agent.safety_controller import clamp_towards, front_minimum


def test_clamp_towards_limits_deceleration_step():
    assert clamp_towards(0.2, 0.0, 0.05) == pytest.approx(0.15)
    assert clamp_towards(0.0, 0.2, 0.05) == pytest.approx(0.05)


def test_front_minimum_ignores_out_of_fov_and_invalid_ranges():
    scan = LaserScan()
    scan.angle_min = -0.3
    scan.angle_increment = 0.3
    scan.range_min = 0.05
    scan.range_max = 10.0
    scan.ranges = [0.2, 0.4, float('inf'), 0.1, 0.3]
    assert front_minimum(scan, 0.3) == pytest.approx(0.2)
