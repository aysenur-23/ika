"""Path rejoin — bypass sonrası ana rotaya kontrollü dönüş.

Mevcut avoider'ın REALIGNING fazının matematiksel çekirdeğinin yerine
geçecek. y_error + yaw_error birlikte clamp'lenmiş PD.
"""
from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class RejoinConfig:
    kp_y: float = 0.8           # y-hata → desired heading delta (rad/m)
    kp_yaw: float = 1.2         # yaw-hata → angular_z
    max_angular_z: float = 0.5  # rad/s
    forward_speed_mps: float = 0.18
    y_tolerance_m: float = 0.15
    yaw_tolerance_rad: float = 0.10
    max_heading_correction_rad: float = math.pi / 3.0  # ±60°


@dataclass
class RejoinCommand:
    linear_x: float
    angular_z: float
    y_error: float
    yaw_error: float
    done: bool
    reason: str = ""


def _wrap_pi(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def _clip(v: float, lo: float, hi: float) -> float:
    return lo if v < lo else (hi if v > hi else v)


def compute_rejoin_command(
    pose,                         # has .x, .y, .yaw
    path_y: float,                # ana rotanın y koordinatı (dünya)
    target_heading: float,        # ana rotanın yönü (dünya rad)
    config: RejoinConfig = None,
) -> RejoinCommand:
    """y-hatasını ve yaw-hatasını birleştirip clamp'li komut üret.

    Kural:
        y_err = path_y - pose.y      (+y_err → robot sağa kaymış, sola dönmeli)
        desired_heading = target_heading + atan(kp_y * y_err)
        yaw_err = wrap_pi(desired_heading - pose.yaw)
        angular_z = clip(kp_yaw * yaw_err, ±max_angular_z)
        linear_x = forward_speed
    """
    cfg = config or RejoinConfig()
    y_err = float(path_y) - float(pose.y)
    heading_offset = math.atan(cfg.kp_y * y_err)
    # ±60° korkuluk — aşırı dönüşü engelle
    heading_offset = _clip(heading_offset,
                            -cfg.max_heading_correction_rad,
                            cfg.max_heading_correction_rad)
    desired_heading = _wrap_pi(float(target_heading) + heading_offset)
    yaw_err = _wrap_pi(desired_heading - float(pose.yaw))

    angular_z = _clip(cfg.kp_yaw * yaw_err,
                      -cfg.max_angular_z, cfg.max_angular_z)
    # Eğer çok büyük yaw hatasıysa, ileri hızı kıs (yerinde dönmeye yaklaş)
    if abs(yaw_err) > math.radians(45):
        linear_x = cfg.forward_speed_mps * 0.3
    else:
        linear_x = cfg.forward_speed_mps

    done = (abs(y_err) <= cfg.y_tolerance_m
            and abs(_wrap_pi(float(target_heading) - float(pose.yaw)))
            <= cfg.yaw_tolerance_rad)
    if done:
        linear_x = cfg.forward_speed_mps
        angular_z = 0.0

    return RejoinCommand(
        linear_x=linear_x,
        angular_z=angular_z,
        y_error=y_err,
        yaw_error=yaw_err,
        done=done,
        reason=(f"y_err={y_err:+.2f}m yaw_err={yaw_err:+.2f}rad "
                f"{'DONE' if done else 'REJOINING'}"),
    )


def should_finish_rejoin(
    pose,
    path_y: float,
    target_heading: float,
    config: RejoinConfig = None,
) -> bool:
    """Rejoin tamamlandı mı? (y ve yaw tolerans içinde)"""
    cfg = config or RejoinConfig()
    y_err = float(path_y) - float(pose.y)
    yaw_err = _wrap_pi(float(target_heading) - float(pose.yaw))
    return (abs(y_err) <= cfg.y_tolerance_m
            and abs(yaw_err) <= cfg.yaw_tolerance_rad)
