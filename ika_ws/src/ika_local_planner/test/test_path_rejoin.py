"""path_rejoin unit testleri."""
from __future__ import annotations

import math

from ika_local_planner.local_planner_logic import Pose2D
from ika_local_planner.path_rejoin import (
    RejoinConfig, RejoinCommand,
    compute_rejoin_command, should_finish_rejoin,
)


_CFG = RejoinConfig()


# ─── y sapması → düzeltici yön ────────────────────────────────────────

def test_robot_left_of_path_turns_right():
    # path_y=0, robot y=0.5 → y_err = -0.5 (path - pose)
    # heading_offset = atan(kp_y * -0.5) → negatif → sağa dön
    pose = Pose2D(0.0, 0.5, 0.0)
    cmd = compute_rejoin_command(pose, path_y=0.0, target_heading=0.0)
    assert cmd.angular_z < 0.0
    assert cmd.linear_x > 0.0
    assert cmd.done is False


def test_robot_right_of_path_turns_left():
    pose = Pose2D(0.0, -0.5, 0.0)
    cmd = compute_rejoin_command(pose, path_y=0.0, target_heading=0.0)
    assert cmd.angular_z > 0.0
    assert cmd.linear_x > 0.0


# ─── yaw clamp ───────────────────────────────────────────────────────

def test_angular_z_is_clamped():
    # Aşırı büyük y-hata → max_angular_z sınırı
    pose = Pose2D(0.0, 10.0, 0.0)
    cmd = compute_rejoin_command(pose, path_y=0.0, target_heading=0.0)
    assert abs(cmd.angular_z) <= _CFG.max_angular_z + 1e-9


def test_large_yaw_error_reduces_linear():
    # Çok büyük yaw hatası → ileri hız kısılır
    pose = Pose2D(0.0, 0.0, math.pi)  # 180° ters yön
    cmd = compute_rejoin_command(pose, path_y=0.0, target_heading=0.0)
    assert cmd.linear_x < _CFG.forward_speed_mps


# ─── done koşulu ─────────────────────────────────────────────────────

def test_done_when_within_tolerance():
    pose = Pose2D(0.0, 0.05, 0.05)  # y=0.05 < 0.15, yaw=0.05 < 0.10
    cmd = compute_rejoin_command(pose, path_y=0.0, target_heading=0.0)
    assert cmd.done is True
    assert cmd.angular_z == 0.0
    assert should_finish_rejoin(pose, 0.0, 0.0) is True


def test_not_done_outside_tolerance_y():
    pose = Pose2D(0.0, 0.5, 0.0)
    cmd = compute_rejoin_command(pose, path_y=0.0, target_heading=0.0)
    assert cmd.done is False
    assert should_finish_rejoin(pose, 0.0, 0.0) is False


def test_not_done_outside_tolerance_yaw():
    pose = Pose2D(0.0, 0.05, math.radians(20))
    cmd = compute_rejoin_command(pose, path_y=0.0, target_heading=0.0)
    assert cmd.done is False


# ─── yaw_error wrap_pi davranışı ────────────────────────────────────

def test_yaw_error_wraps_pi():
    pose = Pose2D(0.0, 0.05, math.pi - 0.01)
    cmd = compute_rejoin_command(pose, path_y=0.0,
                                  target_heading=-math.pi + 0.01)
    # Beklenen: wrap'lı yaw_err ~0.02 (küçük)
    assert abs(cmd.yaw_error) < 0.1


# ─── Sıfır config + finite çıktı ────────────────────────────────────

def test_command_fields_are_finite():
    pose = Pose2D(0.0, 0.3, 0.1)
    cmd = compute_rejoin_command(pose, path_y=0.0, target_heading=0.0)
    for v in (cmd.linear_x, cmd.angular_z, cmd.y_error, cmd.yaw_error):
        assert math.isfinite(v)


# ─── heading_offset clamp ───────────────────────────────────────────

def test_heading_offset_clamp_does_not_exceed_60deg():
    # Çok büyük y-hata bile heading_offset max ±60°'de kalmalı
    pose = Pose2D(0.0, 100.0, 0.0)
    cmd = compute_rejoin_command(pose, path_y=0.0, target_heading=0.0)
    # yaw_error = -clamped(atan(kp_y*y_err)) but ≤ 60° = ~1.047
    assert abs(cmd.yaw_error) <= math.radians(60) + 0.01
