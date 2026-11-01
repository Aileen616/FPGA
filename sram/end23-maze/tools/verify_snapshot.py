"""核对原样SRAM归档；不构建、不下载、不访问硬件。"""
from pathlib import Path
import hashlib
import json


def sha(path):
    """功能：计算摘要；path为非空现有文件；返回SHA256十六进制字符串。"""
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    """功能：核对清单及源码和ROM身份；无参数；成功返回无，任何不一致抛异常。"""
    root = Path(__file__).resolve().parents[1]
    manifest = json.loads((root / 'MANIFEST.json').read_text(encoding='utf-8'))
    expected = manifest['files']
    actual = {p.relative_to(root).as_posix() for p in root.rglob('*')
              if p.is_file() and p != root / 'MANIFEST.json'}
    assert actual == set(expected), ('file list differs', actual ^ set(expected))
    for name, entry in expected.items():
        path = root / name
        assert path.stat().st_size == entry['bytes'] and sha(path) == entry['sha256'], name
    origin = json.loads((root / 'provenance/source_snapshot.json').read_text(encoding='utf-8'))
    for name, value in origin['files'].items():
        assert sha(root / name) == value, name
    assert sha(root / 'program_files/task4_maze_corner_fix_full.fs') == manifest['fs_sha256']
    binary = (root / 'firmware/firmware.bin').read_bytes()
    rom = b''.join(int(v, 16).to_bytes(4, 'little') for v in
                   (root / 'firmware/ram32.hex').read_text().splitlines() if v.strip())
    assert len(binary) == 65144 and rom[:len(binary)] == binary
    print('SNAPSHOT_VERIFIED', len(expected), 'files;', manifest['identity'])
    print('FS_SHA256', manifest['fs_sha256'])


if __name__ == '__main__':
    main()
