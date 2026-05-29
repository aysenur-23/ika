"""IKA - Safety Supervisor karar mantigi (ROS'suz, test edilebilir).

Hibrit fuzyon (ika_fusion) artik terrain + dinamik nesne kararini /hazard_state
uzerinden birlesik bir aksiyon olarak veriyor. Safety supervisor bu aksiyonu
+ kendi e_stop durumunu cmd_vel filtre aksiyonuna indirger. Mantik burada saf
fonksiyon halinde tutulur ki node spin etmeden test edilebilsin.
"""
from __future__ import annotations

# Fusion'in urettigi gecerli aksiyonlar (ika_fusion.hazard_fusion ile uyumlu).
VALID_HAZARD_ACTIONS = ('CLEAR', 'SLOW', 'STOP')


def decide_action(hazard_action: str, e_stop_active: bool) -> str:
    """Birlesik hazard aksiyonu + e_stop -> cmd_vel filtre aksiyonu.

    Donus:
      'stop' -> sifir hiz yayinla
      'slow' -> hizi slowdown_speed_factor ile olcekle
      'pass' -> komutu oldugu gibi gecir

    e_stop her seyi ezer. Taninmayan/eksik hazard_action guvenli tarafta
    kalmak icin 'stop' verir (fail-safe).
    """
    if e_stop_active:
        return 'stop'
    if hazard_action == 'STOP':
        return 'stop'
    if hazard_action == 'SLOW':
        return 'slow'
    if hazard_action == 'CLEAR':
        return 'pass'
    return 'stop'
