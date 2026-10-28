"""Sync this feature without overwriting the board's saved scan_web.yaml."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import tarfile
import time

ROOT = Path(__file__).resolve().parents[1]
OUTPUT = ROOT / 'test-results/web-publication'
HOST = 'neardi@192.168.20.199'
REMOTE = '/home/neardi/ws_livox/src/mid360_suspended_object_detection'


def ssh(script):
    result = subprocess.run(['ssh', '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=10', HOST,
                             'python3 -'], input=script, text=True, capture_output=True, check=True)
    return json.loads(result.stdout)


def sources():
    files = [ROOT / name for name in ('CMakeLists.txt', 'package.xml', 'README.md', 'requirements.txt')]
    for folder in ('src', 'scan_web', 'tests', 'tools', 'launch', 'msg', 'config'):
        files.extend(path for path in (ROOT / folder).rglob('*') if path.is_file()
                     and '__pycache__' not in path.parts and path.suffix != '.pyc'
                     and path.relative_to(ROOT).as_posix() != 'config/scan_web.yaml')
    return {p.relative_to(ROOT).as_posix(): hashlib.sha256(p.read_bytes()).hexdigest() for p in files}


def inspect(manifest):
    return ssh(f'''import pathlib,hashlib,json
root=pathlib.Path({REMOTE!r})
names={list(manifest)!r}
hashes={{name:hashlib.sha256((root/name).read_bytes()).hexdigest() if (root/name).exists() else None for name in names}}
print(json.dumps(dict(hashes=hashes,yaml=hashlib.sha256((root/'config/scan_web.yaml').read_bytes()).hexdigest())))
''')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('action', choices=['upload', 'check', 'fetch'])
    args = parser.parse_args()
    OUTPUT.mkdir(parents=True, exist_ok=True)
    if args.action == 'fetch':
        subprocess.run(['scp', '-q', '-r', f'{HOST}:{REMOTE}/test-results/web-publication/.', str(OUTPUT)], check=True)
        return
    manifest = sources()
    if args.action == 'upload':
        before = inspect(manifest)
        stamp = time.strftime('%Y%m%d_%H%M%S')
        backup = f'/home/neardi/mid360_backups/before_web_publication_{stamp}.tar.gz'
        ssh(f'''import pathlib,subprocess,json
target=pathlib.Path({backup!r});target.parent.mkdir(exist_ok=True)
subprocess.run(['tar','-czf',str(target),'--exclude=mid360_suspended_object_detection/build','--exclude=mid360_suspended_object_detection/test-results','--exclude=__pycache__','-C','/home/neardi/ws_livox/src','mid360_suspended_object_detection'],check=True)
print(json.dumps(dict(backup=str(target))))
''')
        (OUTPUT / 'pre-sync.json').write_text(json.dumps(dict(backup=backup, yaml=before['yaml'],
            differences=[name for name in manifest if before['hashes'][name] != manifest[name]]), indent=2))
        archive = OUTPUT / 'sources.tar.gz'
        with tarfile.open(archive, 'w:gz') as output:
            for name in manifest:
                output.add(ROOT / name, arcname=name)
        remote_archive = f'/tmp/mid360_web_publication_{stamp}.tar.gz'
        subprocess.run(['scp', '-q', str(archive), f'{HOST}:{remote_archive}'], check=True)
        ssh(f'''import subprocess,json,pathlib
subprocess.run(['tar','-xzf',{remote_archive!r},'-C',{REMOTE!r}],check=True)
pathlib.Path({remote_archive!r}).unlink()
print(json.dumps(dict(extracted=True)))
''')
    state = inspect(manifest)
    saved = json.loads((OUTPUT / 'pre-sync.json').read_text())
    assert state['yaml'] == saved['yaml'], 'Saved board YAML changed'
    assert state['hashes'] == manifest, 'Source hash mismatch'
    report = dict(matched_files=len(manifest), saved_yaml_sha256=state['yaml'], yaml_preserved=True,
                  backup=saved['backup'], hashes=manifest)
    (OUTPUT / 'sync-verification.json').write_text(json.dumps(report, indent=2))
    print(json.dumps(dict(matched_files=len(manifest), yaml_preserved=True, backup=saved['backup'])))


if __name__ == '__main__':
    main()
