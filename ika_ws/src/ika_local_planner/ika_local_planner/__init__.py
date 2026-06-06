"""IKA local planner core — pure-Python algorithmic layer.

Modules:
    local_costmap        : scan + semantic detections → 2D cost grid
    semantic_policy      : known/unknown obstacle class → behavior decision
    local_planner_logic  : candidate-corridor scoring + local waypoint selection
    path_rejoin          : bypass sonrası ana rotaya kontrollü dönüş

TASK-4A: ROS bağımsız. Hiçbir modül rclpy import etmemeli.
"""
