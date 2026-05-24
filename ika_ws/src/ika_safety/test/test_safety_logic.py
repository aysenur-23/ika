"""Safety Supervisor mantik testleri.

Pure-fonksiyon olarak ayristirilmis filtreleme/karar mantigi olmadigi icin
bu modul ROS spin etmeden sadece terrain class -> aksiyon esleme tablosunu
test eder. Daha derin mantik integrasyon testlerine birakildi.
"""
import pytest


# Bu tablo SafetySupervisorNode._on_cmd_vel icindeki davranisla ayni olmali.
EXPECTED_ACTION = {
    'DROPOFF_DANGER': 'stop',
    'IMPASSABLE': 'stop',
    'CAUTION': 'slow',
    'UNKNOWN': 'slow',
    'SAFE': 'pass',
}


def classify_action(terrain_class: str,
                    stop_classes=('DROPOFF_DANGER', 'IMPASSABLE'),
                    slow_classes=('CAUTION', 'UNKNOWN')) -> str:
    """SafetySupervisorNode._on_cmd_vel'deki filtreleme mantiginin saf kopyasi."""
    if terrain_class in stop_classes:
        return 'stop'
    if terrain_class in slow_classes:
        return 'slow'
    return 'pass'


@pytest.mark.parametrize('cls,action', EXPECTED_ACTION.items())
def test_classify_action(cls, action):
    assert classify_action(cls) == action


def test_unknown_class_defaults_to_pass():
    assert classify_action('SOMETHING_ELSE') == 'pass'
