"""planner_metrics cekirdegi icin birim testler (ROS'suz)."""
import math

from ika_rl_planner.planner_metrics import (
    goal_reached,
    mean_abs_curvature,
    min_clearance,
    path_length,
    summarize_run,
)


# -----------------------------------------------------------------------
# path_length
# -----------------------------------------------------------------------
def test_path_length_straight():
    pts = [(0.0, 0.0), (1.0, 0.0), (3.0, 0.0)]
    assert abs(path_length(pts) - 3.0) < 1e-9

def test_path_length_single_point_zero():
    assert path_length([(1.0, 1.0)]) == 0.0

def test_path_length_diagonal():
    assert abs(path_length([(0.0, 0.0), (3.0, 4.0)]) - 5.0) < 1e-9


# -----------------------------------------------------------------------
# goal_reached
# -----------------------------------------------------------------------
def test_goal_reached_within_tol():
    assert goal_reached((1.0, 1.0), (1.1, 1.0), 0.25) is True

def test_goal_not_reached():
    assert goal_reached((0.0, 0.0), (5.0, 0.0), 0.25) is False


# -----------------------------------------------------------------------
# min_clearance
# -----------------------------------------------------------------------
def test_min_clearance_basic():
    pts = [(0.0, 0.0), (2.0, 0.0)]
    obs = [(1.0, 1.0)]   # (1,0) noktasina 1.0 m
    assert abs(min_clearance(pts, obs) - 1.0) < 1e-9

def test_min_clearance_no_obstacles_is_inf():
    assert math.isinf(min_clearance([(0.0, 0.0)], []))


# -----------------------------------------------------------------------
# mean_abs_curvature
# -----------------------------------------------------------------------
def test_curvature_straight_is_zero():
    pts = [(0.0, 0.0), (1.0, 0.0), (2.0, 0.0), (3.0, 0.0)]
    assert mean_abs_curvature(pts) < 1e-9

def test_curvature_turn_positive():
    # 90 derece donus -> sifirdan buyuk egrilik
    pts = [(0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (1.0, 2.0)]
    assert mean_abs_curvature(pts) > 0.0

def test_curvature_too_few_points():
    assert mean_abs_curvature([(0.0, 0.0), (1.0, 0.0)]) == 0.0


# -----------------------------------------------------------------------
# summarize_run
# -----------------------------------------------------------------------
def test_summarize_success_straight_run():
    pts = [(0.0, 0.0), (1.0, 0.0), (2.0, 0.0), (3.0, 0.0)]
    r = summarize_run(pts, duration_s=12.0, goal_xy=(3.0, 0.0),
                      obstacles=[(1.5, 1.0)], goal_tolerance_m=0.25)
    assert r.success is True
    assert abs(r.path_length_m - 3.0) < 1e-6
    assert abs(r.min_clearance_m - 1.0) < 1e-6
    assert r.mean_abs_curvature < 1e-6
    assert abs(r.avg_speed_mps - 0.25) < 1e-3
    assert r.num_points == 4

def test_summarize_failure_when_short():
    pts = [(0.0, 0.0), (1.0, 0.0)]
    r = summarize_run(pts, duration_s=5.0, goal_xy=(10.0, 0.0))
    assert r.success is False

def test_summarize_serializable():
    import json
    r = summarize_run([(0.0, 0.0), (1.0, 0.0)], duration_s=1.0,
                      goal_xy=(1.0, 0.0))
    json.dumps(r.as_dict())  # raise etmemeli
