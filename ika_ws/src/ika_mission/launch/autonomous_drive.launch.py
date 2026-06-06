"""IKA — Tam otonom reaktif engel kacinma launch (plain Node).

Calistirma:
    ros2 launch ika_mission autonomous_drive.launch.py

obstacle_avoider plain Node oldugu icin lifecycle_manager gerekmez.
on init + start_delay sonra otomatik DRIVING phase'e gecer.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),

        Node(
            package='ika_mission', executable='obstacle_avoider',
            name='obstacle_avoider', output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'forward_speed_mps': 0.25,       # max guvenli hiz (encoder eklenince artirilabilir)
                'turn_speed_rps': 0.6,
                # KULLANICI ISTEGI (2026-06-05): "Engele biraz daha yaklastiktan
                # sonra donsun, daha yaklasmadan erken donuyor."
                # 0.50 -> 0.35 m. Robot 0.25 m/s × 1.4s = 35 cm icin tepki suresi
                # var (collision_monitor StopZone 30 cm — son savunma asla
                # tetiklenmez normal kosulda). Dinamik tepki gorseli icin ideal.
                'obstacle_distance_m': 0.35,
                'release_distance_m': 0.60,
                # Camera DL: lidar'dan biraz uzak tetikleyici
                # (CLAUDE.md: gercek robotta Pi Camera + IPM kullanilir)
                'camera_detection_distance_m': 0.50,
                # Heading correction DRIVING'de KAPATILDI (sacma sapma fix)
                'heading_kp': 0.0,
                'max_heading_correction_rps': 0.0,
                'heading_critical_err_rad': 9999.0,
                # PASSING: engelin yanindan ne kadar surulsun
                'pass_clear_distance_m': 0.40,
                'release_distance_m': 0.60,     # hysteresis: 0.35 girer, 0.60 cikar
                'pass_clear_distance_m': 0.40,  # engelin yanindan 40 cm sur
                'front_arc_deg': 50.0,          # 40 -> 50 — yakin engelleri kacirmasin
                'target_distance_m': 10000.0,   # pratikte sonsuz — durmadan sür
                'yaw_tolerance_rad': 0.15,      # gevsek (realigning'de takilmasin)
                'control_rate_hz': 20.0,
                'start_delay_s': 3.0,
            }],
        ),
    ])
