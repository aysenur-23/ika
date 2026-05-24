#!/usr/bin/env bash
# IKA - Pre-commit / pre-push workspace saglik kontrolu.
#
# Yaptiklari:
#   - .py dosyalarinda ast.parse (syntax)
#   - .yaml ve package.xml dosyalarinda parse
#   - .xacro / .sdf XML well-formedness
#   - .sh dosyalarinda 'bash -n' syntax
#   - Unit testler (Pi'siz, sadece numpy)
#
# Calistirma:
#   chmod +x scripts/check_workspace.sh
#   ./scripts/check_workspace.sh

set -u

G="\033[0;32m"; R="\033[0;31m"; Y="\033[0;33m"; B="\033[0;34m"; NC="\033[0m"
step() { echo -e "\n${B}== $1 ==${NC}"; }
ok()   { echo -e "${G}[OK]${NC} $1"; }
err()  { echo -e "${R}[!!]${NC} $1"; }

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
ROOT="$( cd "$SCRIPT_DIR/.." && pwd )"
cd "$ROOT"

# Pi'de python3, Windows'ta python - hangisi varsa
if command -v python3 >/dev/null 2>&1; then PY=python3
elif command -v python  >/dev/null 2>&1; then PY=python
else echo "Python bulunamadi"; exit 1; fi

FAIL=0

step "Python syntax (ast.parse)"
PYRES=$($PY -c "
import ast, sys
from pathlib import Path
errs = []
for py in Path('ika_ws/src').rglob('*.py'):
    try: ast.parse(py.read_text(encoding='utf-8'))
    except SyntaxError as e: errs.append(f'{py}: {e}')
sys.exit(0 if not errs else (print('\n'.join(errs)) or 1))
" 2>&1)
if [[ -z "$PYRES" ]]; then ok "Tum .py temiz"; else err "Python syntax hatasi:"; echo "$PYRES"; FAIL=1; fi

step "YAML + package.xml"
YRES=$($PY -c "
import yaml, xml.etree.ElementTree as ET, sys
from pathlib import Path
errs = []
for f in list(Path('ika_ws/src').rglob('*.yaml')) + list(Path('ika_ws/src').rglob('*.yml')):
    try: yaml.safe_load(f.read_text(encoding='utf-8'))
    except yaml.YAMLError as e: errs.append(f'{f}: {e}')
for f in Path('ika_ws/src').rglob('package.xml'):
    try: ET.parse(f)
    except ET.ParseError as e: errs.append(f'{f}: {e}')
sys.exit(0 if not errs else (print('\n'.join(errs)) or 1))
" 2>&1)
if [[ -z "$YRES" ]]; then ok "Tum yaml/xml temiz"; else err "$YRES"; FAIL=1; fi

step "Xacro + SDF XML"
XRES=$($PY -c "
import xml.etree.ElementTree as ET, sys
from pathlib import Path
errs = []
for f in list(Path('ika_ws/src').rglob('*.xacro')) + list(Path('ika_ws/src').rglob('*.sdf')):
    try: ET.parse(f)
    except ET.ParseError as e: errs.append(f'{f}: {e}')
sys.exit(0 if not errs else (print('\n'.join(errs)) or 1))
" 2>&1)
if [[ -z "$XRES" ]]; then ok "Tum xacro/sdf temiz"; else err "$XRES"; FAIL=1; fi

step "Shell scriptler (bash -n)"
for f in scripts/*.sh; do
  if bash -n "$f" 2>/tmp/sh_err; then
    ok "$f"
  else
    err "$f: $(cat /tmp/sh_err)"; FAIL=1
  fi
done

step "Birim testler (python -m pytest)"
for pkg in ika_terrain ika_safety ika_base_controller; do
  if (cd "ika_ws/src/$pkg" && $PY -m pytest test/ -q 2>&1 | tail -2 | head -1); then
    ok "$pkg"
  else
    err "$pkg testleri basarisiz"; FAIL=1
  fi
done

echo
if [[ $FAIL -eq 0 ]]; then
  echo -e "${G}=== TUM KONTROLLER YESIL ===${NC}"
  exit 0
else
  echo -e "${R}=== HATALAR VAR - yukariya bak ===${NC}"
  exit 1
fi
