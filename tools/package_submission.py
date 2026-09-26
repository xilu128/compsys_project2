"""Create a source-only submission ZIP; preserve ST's linked directory layout.
Usage: python tools/package_submission.py --group 02 --output ../project2_group02.zip
"""
import argparse
from pathlib import Path
import zipfile

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--group',required=True)
    p.add_argument('--output',type=Path)
    args=p.parse_args()
    if not args.group.isdigit() or not 1<=int(args.group)<=99:
        p.error('group must be 01..99')
    root=Path(__file__).resolve().parents[1]
    target=args.output or root.parent/f'project2_group{int(args.group):02d}.zip'
    excluded={'.git','.metadata','Debug','Release','__pycache__','__MACOSX','logs'}
    suffixes={'.o','.d','.su','.cyclo','.pyc','.log','.zip'}
    # Exclusive creation protects any prior submission artifact.
    with zipfile.ZipFile(target,'x',zipfile.ZIP_DEFLATED) as archive:
        for file in sorted(root.rglob('*')):
            rel=file.relative_to(root)
            if file.is_file() and not any(x in excluded for x in rel.parts) and file.suffix not in suffixes and file.name!='.DS_Store':
                archive.write(file,Path('COMSYS704')/rel)
    with zipfile.ZipFile(target) as archive:
        bad=archive.testzip()
        if bad: raise RuntimeError(f'CRC failure: {bad}')
        print(f'{target.resolve()}: {len(archive.namelist())} files; CRC PASS')

if __name__=='__main__':
    main()
