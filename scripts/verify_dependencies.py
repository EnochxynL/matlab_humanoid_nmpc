#!/usr/bin/env python3
"""Verify exact dependency commits and unmodified tracked upstream source."""
import json
import subprocess
from pathlib import Path
root = Path(__file__).resolve().parents[1]
lock = json.loads((root / 'third_party/dependencies.lock.json').read_text())
for name, spec in lock.items():
    directory = root / spec['source_directory']
    sha = subprocess.check_output(['git', '-C', str(directory), 'rev-parse', 'HEAD'], text=True).strip()
    dirty = subprocess.check_output(['git', '-C', str(directory), 'status', '--porcelain', '--untracked-files=no'], text=True)
    if sha != spec['commit'] or dirty:
        raise SystemExit(f'{name}: commit mismatch or modified upstream source')
    print(f'{name}: PASS {sha}')
