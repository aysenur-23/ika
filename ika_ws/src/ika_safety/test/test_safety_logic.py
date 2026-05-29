"""Safety Supervisor karar mantigi testleri.

Karar mantigi artik safety_logic.decide_action'da saf fonksiyon olarak yasiyor;
bu test o GERCEK fonksiyonu cagirir (eskiden mantigin kopyasi test ediliyordu).
Terrain + dinamik nesne fuzyonu ika_fusion/test_hazard_fusion.py'de test edilir.
"""
import pytest

from ika_safety.safety_logic import decide_action


@pytest.mark.parametrize('hazard,expected', [
    ('CLEAR', 'pass'),
    ('SLOW', 'slow'),
    ('STOP', 'stop'),
])
def test_action_maps_hazard(hazard, expected):
    assert decide_action(hazard, e_stop_active=False) == expected


def test_estop_overrides_everything():
    for hazard in ('CLEAR', 'SLOW', 'STOP'):
        assert decide_action(hazard, e_stop_active=True) == 'stop'


def test_unknown_hazard_is_failsafe_stop():
    # Bozuk/eksik aksiyon guvenli tarafta DUR vermeli
    assert decide_action('GARBAGE', e_stop_active=False) == 'stop'
    assert decide_action('', e_stop_active=False) == 'stop'
