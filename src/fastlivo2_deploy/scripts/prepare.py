#!/usr/bin/env python3
"""Restore pinned upstream sources and local patches without replacing dirty trees."""
import json
from pathlib import Path
import subprocess


root = Path(__file__).resolve().parents[1]
for repo in json.loads((root / 'repos.lock.json').read_text()):
    destination = root / repo['path']
    if not destination.exists():
        subprocess.run(['git', 'init', str(destination)], check=True)
        subprocess.run(['git', '-C', str(destination), 'remote', 'add', 'origin', repo['url']], check=True)
        subprocess.run(['git', '-C', str(destination), 'fetch', '--depth=1', 'origin', repo['commit']], check=True)
        subprocess.run(['git', '-C', str(destination), 'checkout', '--detach', repo['commit']], check=True)
    head = subprocess.check_output(['git', '-C', str(destination), 'rev-parse', 'HEAD'], text=True).strip()
    if head != repo['commit']:
        raise SystemExit(f'{destination}: different revision {head}; left untouched')
    patch = root / 'patches' / (repo['path'] + '.patch')
    if patch.exists():
        command = ['git', '-C', str(destination), 'apply']
        applied = subprocess.run(command + ['--reverse', '--check', str(patch)], capture_output=True).returncode == 0
        if not applied:
            check = subprocess.run(command + ['--check', str(patch)], capture_output=True, text=True)
            if check.returncode:
                raise SystemExit(f'{destination}: patch conflicts; left untouched\n{check.stderr}')
            subprocess.run(command + [str(patch)], check=True)
        print(f'{repo["path"]}: pinned sources and patch ready')
    else:
        print(f'{repo["path"]}: pinned sources ready')
(root / 'Sophus' / 'COLCON_IGNORE').touch()
