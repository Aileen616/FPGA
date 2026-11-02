"""功能：只读核对Gowin3V4.2完整归档；不编译、不连接硬件。"""
from pathlib import Path
import hashlib
import json


def sha(path):
    """功能：计算摘要。参数：path为现有文件。返回：SHA256字符串。"""
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    """功能：核对文件集合、源码、固件和ROM绑定。参数：无。返回：无，异常即失败。"""
    root=Path(__file__).resolve().parents[1]
    manifest=json.loads((root/'MANIFEST.json').read_text(encoding='utf-8'))
    expected=manifest['files']
    actual={p.relative_to(root).as_posix() for p in root.rglob('*')
            if p.is_file() and p.name!='MANIFEST.json' and '__pycache__' not in p.parts}
    assert actual==set(expected),('File set differs',actual^set(expected))
    for name,item in expected.items():
        path=root/name
        assert path.stat().st_size==item['bytes'] and sha(path)==item['sha256'],name
    origin=json.loads((root/'provenance/source_snapshot.json').read_text(encoding='utf-8'))
    for name,digest in origin['files'].items():assert sha(root/name)==digest,name
    assert len(origin['files'])==246
    assert sha(root/'program_files/task4_corner30_full.fs')==manifest['fs_sha256']
    binary=(root/'firmware/firmware.bin').read_bytes()
    rom=b''.join(int(line,16).to_bytes(4,'little') for line in (root/'firmware/ram32.hex').read_text().splitlines() if line.strip())
    assert len(binary)==68220 and len(rom)==131072 and rom[:len(binary)]==binary
    assert sha(root/'firmware/firmware.bin')==origin['firmware_sha256']
    print('SNAPSHOT_VERIFIED',len(expected),'files;',manifest['label'],manifest['identity'])
    print('FS_SHA256',manifest['fs_sha256'])


if __name__=='__main__':main()
