"""Package an exact committed source revision, preserving ST linked paths."""
import argparse
import hashlib
import io
import json
from pathlib import Path, PurePosixPath
import re
import stat
import subprocess
import xml.etree.ElementTree as ET
import zipfile

APP = 'Projects/STM32L476JG-SensorTile/Applications/ALLMEMS1'
PROJECT = APP + '/STM32CubeIDE'
REQUIRED = ('README.md', 'SUBMISSION.md', 'VALIDATION.md',
            PROJECT + '/.project', PROJECT + '/.cproject',
            PROJECT + '/STM32L476RGTX.ld', APP + '/Src/main.c',
            APP + '/Src/p2_motion.c', APP + '/Inc/p2_config.h')
EXCLUDED = {'.git', '.metadata', 'Debug', 'Release', '__pycache__',
            '__MACOSX', 'logs', '.zig-cache', 'zig-cache', '.venv', 'venv'}
SUFFIXES = {'.o', '.obj', '.d', '.su', '.cyclo', '.pyc', '.log', '.zip',
            '.elf', '.map', '.exe', '.pdb'}


def include(name):
    path = PurePosixPath(name)
    if path.is_absolute() or '..' in path.parts:
        raise ValueError('Unsafe archive path: ' + name)
    if any(part in EXCLUDED for part in path.parts):
        return False
    if path.name == '.DS_Store' or path.suffix.lower() in SUFFIXES:
        return False
    return path.suffix.lower() != '.bin' or name.startswith('Utilities/BootLoader/')


def verify_layout(files):
    for name in REQUIRED:
        if name not in files:
            raise ValueError('Missing required source: ' + name)
    project = ET.fromstring(files[PROJECT + '/.project'])
    for link in project.findall('./linkedResources/link'):
        uri = link.findtext('locationURI', '')
        if uri.startswith('virtual:'):
            continue
        match = re.fullmatch(r'PARENT-(\d+)-PROJECT_LOC/(.+)', uri)
        if not match:
            raise ValueError('Unsupported/nonportable linked resource: ' + uri)
        parents = int(match[1])
        base = PurePosixPath(PROJECT)
        if parents < 1 or parents > len(base.parts):
            raise ValueError('Linked resource escapes source root: ' + uri)
        target = str(base.parents[parents - 1] / match[2])
        if link.findtext('type') == '2':
            exists = any(name.startswith(target.rstrip('/') + '/') for name in files)
        else:
            exists = target in files
        if not exists:
            raise ValueError('Missing linked resource: ' + target)


def package(root, target, group, revision='HEAD'):
    root, target = Path(root), Path(target)
    commit = subprocess.check_output(
        ['git', 'rev-parse', '--verify', '--end-of-options', revision + '^{commit}'],
        cwd=root, text=True).strip()
    archive_bytes = subprocess.check_output(
        ['git', '-c', 'core.autocrlf=false', '-c', 'core.eol=lf',
         'archive', '--format=zip', commit], cwd=root)
    files = {}
    with zipfile.ZipFile(io.BytesIO(archive_bytes)) as source:
        for item in source.infolist():
            if item.is_dir() or not include(item.filename):
                continue
            if stat.S_ISLNK(item.external_attr >> 16):
                raise ValueError('Symlink requires explicit packaging review: ' + item.filename)
            files[item.filename] = source.read(item)
    verify_layout(files)
    manifest = {'group': f'{int(group):02d}', 'source_commit': commit,
                'source_policy': 'Committed Git tree only; working changes excluded',
                'files': {name: hashlib.sha256(data).hexdigest()
                          for name, data in sorted(files.items())}}
    with zipfile.ZipFile(target, 'x', zipfile.ZIP_DEFLATED) as archive:
        for name, data in sorted(files.items()):
            archive.writestr('COMSYS704/' + name, data)
        archive.writestr('COMSYS704/SUBMISSION_MANIFEST.json',
                         json.dumps(manifest, indent=2) + '\n')
    with zipfile.ZipFile(target) as archive:
        bad = archive.testzip()
        if bad:
            raise RuntimeError('CRC failure: ' + bad)
    digest = hashlib.sha256(target.read_bytes()).hexdigest()
    print(f'{target.resolve()}: {len(files)} source files; CRC PASS')
    print('Source commit:', commit)
    print('ZIP SHA256:', digest)
    return manifest


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--group', required=True)
    parser.add_argument('--revision', default='HEAD')
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    if not args.group.isdigit() or not 1 <= int(args.group) <= 99:
        parser.error('group must be 01..99')
    root = Path(__file__).resolve().parents[1]
    target = args.output or root.parent / f'project2_group{int(args.group):02d}.zip'
    package(root, target, args.group, args.revision)


if __name__ == '__main__':
    main()
