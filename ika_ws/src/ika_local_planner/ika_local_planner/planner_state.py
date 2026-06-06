"""Planner runtime state — behavior smoothing + deadlock detection.

Saf-Python, ROS bağımsız. Node tarafı sadece time + odom besler;
state machine kararları burada test edilebilir biçimde tutulur.

Eklenenler (TASK-4B-2):
    - BehaviorSmoother: N-tick agreement gerektiren mod geçişi
    - DeadlockDetector: progress window + escape side flip + cooldown
"""
from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field
from typing import Optional, Tuple


# ════════════════════════════════════════════════════════════════════════
# Behavior smoothing
# ════════════════════════════════════════════════════════════════════════

@dataclass
class BehaviorSmoother:
    """Raw behavior_mode değişikliklerini N ardışık tick onayı ile filtrele.

    Güvenlik bypass: 'hold' veya `force_bypass=True` ise smoothing atlanır,
    raw direkt yansır (örn. reflex_active durumunda).
    """
    confirm_ticks: int = 3
    current: str = "drive"
    _candidate: Optional[str] = None
    _count: int = 0
    switch_count: int = 0

    def update(self, raw: str, force_bypass: bool = False) -> str:
        """Yeni raw moda göre smoothed mode döndür."""
        if force_bypass or raw == "hold":
            if raw != self.current:
                self.switch_count += 1
            self.current = raw
            self._candidate = None
            self._count = 0
            return self.current

        if raw == self.current:
            self._candidate = None
            self._count = 0
            return self.current

        if raw == self._candidate:
            self._count += 1
        else:
            self._candidate = raw
            self._count = 1

        if self._count >= self.confirm_ticks:
            self.current = raw
            self._candidate = None
            self._count = 0
            self.switch_count += 1
        return self.current

    def candidate_count(self) -> int:
        return self._count


# ════════════════════════════════════════════════════════════════════════
# Deadlock detector
# ════════════════════════════════════════════════════════════════════════

@dataclass
class DeadlockConfig:
    window_s: float = 3.0
    progress_m: float = 0.08
    escape_s: float = 3.0
    cooldown_s: float = 5.0


@dataclass
class DeadlockState:
    """Forward progress takibi + tarafsal kilit / cooldown.

    Test stratejisi: real time yerine `now` zaman damgası dışarıdan beslenir.
    """
    config: DeadlockConfig = field(default_factory=DeadlockConfig)
    _history: deque = field(default_factory=deque)
    active: bool = False
    _escape_until_t: float = 0.0
    _cooldown_until_t: float = 0.0
    forced_side: Optional[str] = None

    def update(self, now_s: float, x: float, y: float,
               current_side: str = "auto") -> None:
        """Konum güncellemesi — gerekirse deadlock'u tetikle/temizle."""
        self._history.append((now_s, x, y))
        # window dışına düşenleri at
        cutoff = now_s - self.config.window_s
        while self._history and self._history[0][0] < cutoff:
            self._history.popleft()

        # Escape süresi dolduysa kilidi aç
        if self.active and now_s >= self._escape_until_t:
            self.active = False
            self.forced_side = None
            self._cooldown_until_t = now_s + self.config.cooldown_s

        # Cooldown sırasında yeni deadlock tetikleme
        if now_s < self._cooldown_until_t:
            return

        # Yeterli geçmiş yoksa karar verme
        if len(self._history) < 2:
            return
        t0, x0, y0 = self._history[0]
        t1, x1, y1 = self._history[-1]
        if (t1 - t0) < self.config.window_s * 0.66:  # en az 2/3 pencere
            return
        progress = ((x1 - x0) ** 2 + (y1 - y0) ** 2) ** 0.5
        if progress < self.config.progress_m and not self.active:
            self.active = True
            self._escape_until_t = now_s + self.config.escape_s
            # Forced side: mevcut tarafın tersi
            if current_side == "left":
                self.forced_side = "right"
            elif current_side == "right":
                self.forced_side = "left"
            else:
                # auto → sağ deneyelim (bias-free seçim)
                self.forced_side = "right"

    def escape_remaining(self, now_s: float) -> float:
        if not self.active:
            return 0.0
        return max(0.0, self._escape_until_t - now_s)

    def last_progress(self) -> float:
        if len(self._history) < 2:
            return 0.0
        t0, x0, y0 = self._history[0]
        t1, x1, y1 = self._history[-1]
        return ((x1 - x0) ** 2 + (y1 - y0) ** 2) ** 0.5


# ════════════════════════════════════════════════════════════════════════
# Lateral offset hysteresis (state container)
# ════════════════════════════════════════════════════════════════════════

@dataclass
class OffsetMemory:
    """Önceki seçilen lateral offset + chosen_side hatırlama."""
    last_offset_y: Optional[float] = None
    last_chosen_side: str = "none"
    switch_count: int = 0

    def record(self, offset_y: float, chosen_side: str) -> None:
        if self.last_offset_y is not None:
            # Side değişimi sayılır
            if (chosen_side != self.last_chosen_side
                    and chosen_side != "none"
                    and self.last_chosen_side != "none"):
                self.switch_count += 1
        self.last_offset_y = offset_y
        self.last_chosen_side = chosen_side
