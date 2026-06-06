"""planner_state.py unit testleri — BehaviorSmoother + DeadlockState +
OffsetMemory. Saf-Python, ROS bağımsız."""
from __future__ import annotations

from ika_local_planner.planner_state import (
    BehaviorSmoother, DeadlockConfig, DeadlockState, OffsetMemory,
)


# ════════════════════════════════════════════════════════════════════════
# BehaviorSmoother
# ════════════════════════════════════════════════════════════════════════

def test_smoother_starts_at_drive():
    s = BehaviorSmoother(confirm_ticks=3)
    assert s.current == 'drive'
    assert s.candidate_count() == 0


def test_smoother_does_not_switch_on_single_tick():
    s = BehaviorSmoother(confirm_ticks=3)
    out = s.update('generic_bypass')
    assert out == 'drive'
    assert s.candidate_count() == 1


def test_smoother_switches_after_n_ticks():
    s = BehaviorSmoother(confirm_ticks=3)
    s.update('generic_bypass')
    s.update('generic_bypass')
    out = s.update('generic_bypass')
    assert out == 'generic_bypass'
    assert s.switch_count == 1


def test_smoother_resets_candidate_on_interruption():
    s = BehaviorSmoother(confirm_ticks=3)
    s.update('generic_bypass')   # candidate count = 1
    s.update('drive')            # reset
    s.update('generic_bypass')   # candidate count = 1 again
    assert s.current == 'drive'
    assert s.candidate_count() == 1


def test_smoother_hold_bypasses_smoothing():
    s = BehaviorSmoother(confirm_ticks=3)
    out = s.update('hold')
    assert out == 'hold'
    assert s.current == 'hold'


def test_smoother_force_bypass_immediate():
    s = BehaviorSmoother(confirm_ticks=3)
    out = s.update('stop_and_bypass', force_bypass=True)
    assert out == 'stop_and_bypass'


def test_smoother_unchanged_raw_keeps_candidate_zero():
    s = BehaviorSmoother(confirm_ticks=3)
    for _ in range(5):
        out = s.update('drive')
    assert out == 'drive'
    assert s.candidate_count() == 0
    assert s.switch_count == 0


# ════════════════════════════════════════════════════════════════════════
# DeadlockState
# ════════════════════════════════════════════════════════════════════════

def test_deadlock_not_active_with_progress():
    st = DeadlockState(config=DeadlockConfig(window_s=3.0, progress_m=0.08))
    for i in range(15):
        st.update(now_s=float(i), x=float(i) * 0.2, y=0.0,
                  current_side='auto')
    assert st.active is False


def test_deadlock_activates_after_window_without_progress():
    cfg = DeadlockConfig(window_s=3.0, progress_m=0.08,
                          escape_s=3.0, cooldown_s=5.0)
    st = DeadlockState(config=cfg)
    # Robot oturmuş
    for t in (0.0, 1.0, 2.0, 3.0, 4.0):
        st.update(now_s=t, x=5.0, y=0.0, current_side='left')
    assert st.active is True
    assert st.forced_side == 'right'  # left'in tersi


def test_deadlock_forced_side_inverts_current():
    cfg = DeadlockConfig(window_s=3.0, progress_m=0.08)
    st = DeadlockState(config=cfg)
    for t in (0.0, 1.0, 2.0, 3.0, 4.0):
        st.update(now_s=t, x=5.0, y=0.0, current_side='right')
    assert st.active is True
    assert st.forced_side == 'left'


def test_deadlock_escape_clears_after_duration():
    cfg = DeadlockConfig(window_s=3.0, progress_m=0.08,
                          escape_s=3.0, cooldown_s=5.0)
    st = DeadlockState(config=cfg)
    # Deadlock tetikle
    for t in (0.0, 1.0, 2.0, 3.0, 4.0):
        st.update(now_s=t, x=5.0, y=0.0, current_side='left')
    assert st.active is True
    # 4s sonra escape süresi (3s) doldu → cooldown başladı
    st.update(now_s=10.0, x=5.0, y=0.0, current_side='auto')
    assert st.active is False


def test_deadlock_cooldown_prevents_immediate_retrigger():
    cfg = DeadlockConfig(window_s=3.0, progress_m=0.08,
                          escape_s=3.0, cooldown_s=5.0)
    st = DeadlockState(config=cfg)
    for t in (0.0, 1.0, 2.0, 3.0, 4.0):
        st.update(now_s=t, x=5.0, y=0.0, current_side='left')
    assert st.active is True
    # Escape sonrası cooldown
    st.update(now_s=8.0, x=5.0, y=0.0, current_side='auto')
    assert st.active is False
    # Cooldown bittikten ÖNCE tekrar tetikleme denenmemeli
    # (8.0'da deactivate → cooldown_until=13.0). t=10.0 cooldown içinde.
    for t in (9.0, 10.0, 11.0, 12.0):
        st.update(now_s=t, x=5.0, y=0.0, current_side='auto')
    assert st.active is False


# ════════════════════════════════════════════════════════════════════════
# OffsetMemory
# ════════════════════════════════════════════════════════════════════════

def test_offset_memory_starts_empty():
    om = OffsetMemory()
    assert om.last_offset_y is None
    assert om.last_chosen_side == 'none'
    assert om.switch_count == 0


def test_offset_memory_records_first():
    om = OffsetMemory()
    om.record(0.5, 'left')
    assert om.last_offset_y == 0.5
    assert om.last_chosen_side == 'left'
    # İlk kayıt switch sayılmaz
    assert om.switch_count == 0


def test_offset_memory_counts_side_switches():
    om = OffsetMemory()
    om.record(0.5, 'left')
    om.record(0.6, 'left')   # same side
    om.record(-0.5, 'right') # switch
    om.record(0.5, 'left')   # switch
    assert om.switch_count == 2


def test_offset_memory_ignores_none_transitions():
    om = OffsetMemory()
    om.record(0.5, 'left')
    om.record(0.0, 'none')   # → none, switch sayılmaz
    om.record(0.5, 'left')   # ← none'dan dönüş, switch sayılmaz
    assert om.switch_count == 0
