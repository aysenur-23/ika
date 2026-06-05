#!/usr/bin/env python3
"""Faz 2 — nav2_params.yaml ablasyon yaması (Python YAML, in-place).

Kullanım:
  patch_nav2_params.py apply  A2.1   # backup + edit
  patch_nav2_params.py apply  A2.2
  patch_nav2_params.py apply  A2.3
  patch_nav2_params.py restore       # backup'tan geri yaz

Yamalar:
  A2.1 (collision_monitor only): BaseObstacle.scale=0, inflation_radius=0.05
  A2.2 (DWB only)             : default nav2 (yama yok); launch'ta bypass_collision_monitor=true
  A2.3 (inflation only)       : BaseObstacle.scale=0, inflation=0.55 (default); launch'ta bypass=true
  A2.4 (baseline = all)       : default nav2

NOTE: kullanım için sadece A2.1 nav2_params.yaml editi gerektirir.
A2.2/A2.3/A2.4 için bu script `apply` sırasında sadece BACKUP alır (no edit),
restore symmetric çalışsın diye.
"""
import argparse
import shutil
import sys
from pathlib import Path

try:
    import yaml
except ImportError:
    print("HATA: PyYAML yok. pip install pyyaml veya apt install python3-yaml",
          file=sys.stderr)
    sys.exit(1)


# Workspace root
SCRIPT_DIR = Path(__file__).resolve().parent
PROJECT_ROOT = SCRIPT_DIR.parent.parent.parent
PARAMS_FILE = PROJECT_ROOT / 'ika_ws' / 'src' / 'ika_navigation' / 'config' / 'nav2_params.yaml'
BACKUP_FILE = PARAMS_FILE.with_suffix('.yaml.ablation_backup')


def patch_a2_1(params: dict) -> dict:
    """A2.1 — sadece collision_monitor.
    DWB engel maliyetini sıfırla, inflation'ı çok küçült. collision_monitor
    son anda durdursun beklenir.
    """
    # DWB BaseObstacle ağırlığı 0
    fp = params['controller_server']['ros__parameters']['FollowPath']
    fp['BaseObstacle.scale'] = 0.0
    # Local + global inflation çok küçük
    for cm_key in ('local_costmap', 'global_costmap'):
        infl = params[cm_key][cm_key]['ros__parameters']['inflation_layer']
        infl['inflation_radius'] = 0.05
        infl['cost_scaling_factor'] = 100.0
    return params


def patch_a2_2(params: dict) -> dict:
    """A2.2 — sadece DWB (default değerler). Hiçbir nav2 değişikliği gerekmez.
    Bypass launch'tan sağlanır.
    """
    return params  # no-op


def patch_a2_3(params: dict) -> dict:
    """A2.3 — sadece planner inflation.
    DWB BaseObstacle 0, inflation default (0.55) kalır. Planner path
    engelin etrafından geçecek; DWB sadece path takip eder.
    """
    fp = params['controller_server']['ros__parameters']['FollowPath']
    fp['BaseObstacle.scale'] = 0.0
    return params


PATCHES = {
    'A2.1': patch_a2_1,
    'A2.2': patch_a2_2,
    'A2.3': patch_a2_3,
    'A2.4': lambda p: p,  # baseline = no change
}


def cmd_apply(name: str):
    if name not in PATCHES:
        print(f"Bilinmeyen ablation: {name}. Geçerli: {list(PATCHES)}",
              file=sys.stderr)
        sys.exit(1)
    if not PARAMS_FILE.exists():
        print(f"Bulunamadı: {PARAMS_FILE}", file=sys.stderr)
        sys.exit(1)
    # Backup (yalnızca daha önce yoksa — restore garantisi)
    if not BACKUP_FILE.exists():
        shutil.copy2(PARAMS_FILE, BACKUP_FILE)
        print(f"Backup: {BACKUP_FILE}")
    else:
        print(f"Backup zaten var: {BACKUP_FILE} (önceki run restore edilmedi)")
        # Önce eski backup'tan restore et — temiz başlangıç
        shutil.copy2(BACKUP_FILE, PARAMS_FILE)
        print(f"Backup'tan restore -> temiz baseline")

    with open(PARAMS_FILE) as f:
        params = yaml.safe_load(f)

    params = PATCHES[name](params)

    with open(PARAMS_FILE, 'w') as f:
        yaml.safe_dump(params, f, default_flow_style=False, sort_keys=False)
    print(f"Yama uygulandı: {name} -> {PARAMS_FILE}")


def cmd_restore():
    if not BACKUP_FILE.exists():
        print(f"Backup yok: {BACKUP_FILE} — restore atladı", file=sys.stderr)
        return
    shutil.copy2(BACKUP_FILE, PARAMS_FILE)
    BACKUP_FILE.unlink()
    print(f"Restore: {PARAMS_FILE} <- backup (silindi)")


def main():
    ap = argparse.ArgumentParser()
    sub = ap.add_subparsers(dest='cmd', required=True)
    sub_apply = sub.add_parser('apply')
    sub_apply.add_argument('name')
    sub.add_parser('restore')
    args = ap.parse_args()

    if args.cmd == 'apply':
        cmd_apply(args.name)
    elif args.cmd == 'restore':
        cmd_restore()


if __name__ == '__main__':
    main()
