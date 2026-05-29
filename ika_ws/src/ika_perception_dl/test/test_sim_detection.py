"""sim_detection cekirdegi icin birim testler (ROS'suz)."""
import math

from ika_perception_dl.sim_detection import (
    SimObstacle,
    in_fov,
    obstacle_position_at,
    simulate_detections,
    world_to_base,
)


# -----------------------------------------------------------------------
# obstacle_position_at
# -----------------------------------------------------------------------
def test_static_obstacle_does_not_move():
    obs = SimObstacle('chair', 'STATIC', 2.0, 1.0)
    assert obstacle_position_at(obs, 5.0) == (2.0, 1.0)

def test_moving_obstacle_linear():
    obs = SimObstacle('person', 'DYNAMIC', 0.0, 0.0, vx=0.5, vy=-0.2)
    x, y = obstacle_position_at(obs, 2.0)
    assert abs(x - 1.0) < 1e-9 and abs(y - (-0.4)) < 1e-9


# -----------------------------------------------------------------------
# world_to_base
# -----------------------------------------------------------------------
def test_world_to_base_robot_at_origin_facing_x():
    # Robot orijinde, +x'e bakiyor; engel (2,0) -> base (2,0)
    x_b, y_b = world_to_base(2.0, 0.0, 0.0, 0.0, 0.0)
    assert abs(x_b - 2.0) < 1e-9 and abs(y_b - 0.0) < 1e-9

def test_world_to_base_robot_rotated_90():
    # Robot orijinde +y'ye bakiyor (yaw=90); world (0,2) onunde olmali
    x_b, y_b = world_to_base(0.0, 2.0, 0.0, 0.0, math.pi / 2)
    assert abs(x_b - 2.0) < 1e-6 and abs(y_b - 0.0) < 1e-6

def test_world_to_base_translation():
    # Robot (1,1) +x'e bakiyor; engel (3,1) -> base (2,0)
    x_b, y_b = world_to_base(3.0, 1.0, 1.0, 1.0, 0.0)
    assert abs(x_b - 2.0) < 1e-9 and abs(y_b - 0.0) < 1e-9


# -----------------------------------------------------------------------
# in_fov
# -----------------------------------------------------------------------
def test_in_fov_straight_ahead():
    assert in_fov(2.0, 0.0, hfov_rad=1.2, min_range_m=0.2, max_range_m=6.0)

def test_behind_not_in_fov():
    assert not in_fov(-2.0, 0.0, hfov_rad=1.2, min_range_m=0.2, max_range_m=6.0)

def test_too_far_not_in_fov():
    assert not in_fov(8.0, 0.0, hfov_rad=1.2, min_range_m=0.2, max_range_m=6.0)

def test_outside_angle_not_in_fov():
    # 60 derece yana (~1.05 rad); hfov 1.2 -> yari aci 0.6 rad disinda
    x_b = 1.0
    y_b = math.tan(1.05) * x_b
    assert not in_fov(x_b, y_b, hfov_rad=1.2, min_range_m=0.2, max_range_m=6.0)


# -----------------------------------------------------------------------
# simulate_detections (entegrasyon)
# -----------------------------------------------------------------------
def test_crossing_pedestrian_detected_when_in_front():
    # Yayanin t=0'da onumuzden gecisi: world (2, -1) -> (2, +1), vy=0.5
    obs = SimObstacle('person', 'DYNAMIC', 2.0, -1.0, vy=0.5)
    # t=2 -> y=0 (tam onde)
    dets = simulate_detections([obs], 2.0, 0.0, 0.0, 0.0)
    assert len(dets) == 1
    d = dets[0]
    assert d.label == 'person' and d.hazard == 'DYNAMIC'
    assert abs(d.x - 2.0) < 1e-3 and abs(d.y - 0.0) < 1e-3

def test_pedestrian_behind_not_detected():
    obs = SimObstacle('person', 'DYNAMIC', -2.0, 0.0)
    assert simulate_detections([obs], 0.0, 0.0, 0.0, 0.0) == []

def test_multiple_obstacles_only_visible_returned():
    obstacles = [
        SimObstacle('person', 'DYNAMIC', 2.0, 0.0),    # onde, gorunur
        SimObstacle('car', 'DYNAMIC', -3.0, 0.0),      # arkada, gorunmez
        SimObstacle('chair', 'STATIC', 1.5, 0.3),      # onde, gorunur
    ]
    dets = simulate_detections(obstacles, 0.0, 0.0, 0.0, 0.0)
    labels = sorted(d.label for d in dets)
    assert labels == ['chair', 'person']
