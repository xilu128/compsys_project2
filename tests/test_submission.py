"""Prove the source ZIP uses committed bytes and preserves linked dependencies."""
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
import zipfile

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
import package_submission as pack


class SubmissionTests(unittest.TestCase):
    def test_committed_source_and_safe_exclusions(self):
        with tempfile.TemporaryDirectory(prefix='p2-package-test-') as tmp:
            root = Path(tmp) / 'source'
            root.mkdir()
            def git(*args):
                return subprocess.check_output(['git', '-C', str(root), *args],
                                               stderr=subprocess.STDOUT)
            git('init', '-q')
            for name in pack.REQUIRED:
                file = root / name
                file.parent.mkdir(parents=True, exist_ok=True)
                file.write_text('committed\n', encoding='utf-8')
            project = root / pack.PROJECT / '.project'
            project.write_text('<projectDescription><linkedResources><link>'
                               '<name>driver</name><type>1</type>'
                               '<locationURI>PARENT-5-PROJECT_LOC/Drivers/test.c</locationURI>'
                               '</link></linkedResources></projectDescription>', encoding='utf-8')
            for name in ('Drivers/test.c', 'Drivers/LICENSE.txt', 'tests/data/sample.csv',
                         'Utilities/BootLoader/original.bin', 'Debug/output.elf',
                         'logs/private.csv', '.zig-cache/cache', 'unwanted.bin'):
                file = root / name
                file.parent.mkdir(parents=True, exist_ok=True)
                file.write_bytes(b'fixture')
            git('add', '.')
            git('-c', 'user.name=Package Test', '-c', 'user.email=test@example.invalid',
                'commit', '-qm', 'fixture')
            (root / 'README.md').write_text('local dirty data', encoding='utf-8')
            (root / 'untracked-secret.txt').write_text('must not be exported', encoding='utf-8')
            output = Path(tmp) / 'submission.zip'
            manifest = pack.package(root, output, '02')
            with zipfile.ZipFile(output) as z:
                self.assertEqual(z.read('COMSYS704/README.md'), git('show', 'HEAD:README.md'))
                for name in ('Drivers/LICENSE.txt', 'tests/data/sample.csv',
                             'Utilities/BootLoader/original.bin'):
                    self.assertIn('COMSYS704/' + name, z.namelist())
                for name in ('Debug/output.elf', 'logs/private.csv', '.zig-cache/cache',
                             'unwanted.bin', 'untracked-secret.txt'):
                    self.assertNotIn('COMSYS704/' + name, z.namelist())
                self.assertEqual(json.loads(z.read('COMSYS704/SUBMISSION_MANIFEST.json')), manifest)
                for name, digest in manifest['files'].items():
                    self.assertEqual(hashlib.sha256(z.read('COMSYS704/' + name)).hexdigest(), digest)
            original_zip = output.read_bytes()
            with self.assertRaises(FileExistsError):
                pack.package(root, output, '02')
            self.assertEqual(output.read_bytes(), original_zip)
            git('rm', '-q', 'Drivers/test.c')
            git('-c', 'user.name=Package Test', '-c', 'user.email=test@example.invalid',
                'commit', '-qm', 'missing dependency')
            with self.assertRaisesRegex(ValueError, 'Missing linked resource'):
                pack.package(root, Path(tmp) / 'invalid.zip', '02')
            self.assertFalse((Path(tmp) / 'invalid.zip').exists())

    def test_traversal_rejected(self):
        for name in ('../secret', '/absolute/file'):
            with self.assertRaises(ValueError):
                pack.include(name)


if __name__ == '__main__':
    unittest.main()
