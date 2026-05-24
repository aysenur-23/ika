"""Base controller hiz donusumu testi.

Twist (linear.x, angular.z) -> (v_left, v_right) donusumu skid-steer formul:
  v_left  = linear.x - (angular.z * wheel_base / 2)
  v_right = linear.x + (angular.z * wheel_base / 2)
"""
import pytest


def twist_to_wheels(linear_x: float, angular_z: float, wheel_base: float):
    v_left = linear_x - (angular_z * wheel_base / 2.0)
    v_right = linear_x + (angular_z * wheel_base / 2.0)
    return v_left, v_right


def test_straight():
    l, r = twist_to_wheels(0.2, 0.0, 0.3)
    assert l == pytest.approx(0.2)
    assert r == pytest.approx(0.2)


def test_pure_rotation():
    l, r = twist_to_wheels(0.0, 1.0, 0.3)
    assert l == pytest.approx(-0.15)
    assert r == pytest.approx(0.15)


def test_combined():
    l, r = twist_to_wheels(0.2, 0.5, 0.3)
    assert l == pytest.approx(0.2 - 0.075)
    assert r == pytest.approx(0.2 + 0.075)


def test_negative_motion():
    l, r = twist_to_wheels(-0.1, 0.0, 0.3)
    assert l == pytest.approx(-0.1)
    assert r == pytest.approx(-0.1)


def test_clamping_simulation():
    # Hizlar max'i asarsa orijinal kod max'a clamp eder.
    max_v = 0.30
    l, r = twist_to_wheels(0.5, 0.0, 0.3)
    l = max(-max_v, min(max_v, l))
    r = max(-max_v, min(max_v, r))
    assert l == 0.30 and r == 0.30
