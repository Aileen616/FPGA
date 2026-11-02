"""功能：重定位冻结C构建命令到独立输出目录，不改变归档文件或访问硬件。"""
from pathlib import Path,PureWindowsPath
import argparse
import hashlib
import json
import subprocess


def main():
    """功能：执行原编译命令并对照固件摘要。参数：工具bin目录、独立新输出目录。返回：无，构建失败抛异常。"""
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--toolchain',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    root=Path(__file__).resolve().parents[1]
    output=args.output.resolve()
    assert output!=root and root not in output.parents,'Output must be outside the frozen snapshot'
    output.mkdir(parents=True,exist_ok=False)
    old='F:/FPGA/build/soc/task4_corner30'
    commands=json.loads((root/'firmware_commands.json').read_text(encoding='utf-8'))
    for index,row in enumerate(commands):
        name=PureWindowsPath(row['command'][0]).name
        executable=args.toolchain/name
        if not executable.exists() and name.endswith('.exe'):executable=args.toolchain/name[:-4]
        assert executable.is_file(),str(executable)
        command=[str(executable)]
        for part in row['command'][1:]:
            part=part.replace('\\','/').replace(old+'/firmware/',output.as_posix()+'/').replace(old+'/',root.as_posix()+'/')
            command.append(part)
        done=subprocess.run(command,cwd=root,capture_output=True,timeout=150)
        (output/('command_'+str(index)+'.log')).write_bytes(done.stdout+done.stderr)
        assert done.returncode==0,('Build command failed',index)
    raw=(output/'firmware.bin').read_bytes()
    assert 0<len(raw)<131072
    padded=raw.ljust(131072,b'\0')
    (output/'ram32.hex').write_text(''.join(f'{int.from_bytes(padded[i:i+4],"little"):08x}\n' for i in range(0,len(padded),4)),encoding='ascii',newline='\n')
    origin=json.loads((root/'provenance/source_snapshot.json').read_text(encoding='utf-8'))
    digest=hashlib.sha256(raw).hexdigest()
    result=dict(firmware_bytes=len(raw),sha256=digest,matches_archived_binary=digest==origin['firmware_sha256'],hardware_access=False)
    (output/'rebuild_result.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
    print(json.dumps(result))


if __name__=='__main__':main()
