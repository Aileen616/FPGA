"""功能：电脑侧SSH连接与只读遥测；远端代码仅在内存执行。"""
import base64
import importlib
import json
import logging
import queue
from pathlib import Path
import socket
import sys
import threading
import time
from datetime import datetime


def ssh_library():
    """功能：加载现有SSH依赖；无参数；返回paramiko模块，缺失时抛出明确异常。"""
    logger = logging.getLogger("paramiko.transport")
    if not logger.handlers:
        logger.addHandler(logging.NullHandler())
    logger.propagate = False
    try:
        return importlib.import_module("paramiko")
    except ImportError:
        existing = Path.home() / "Documents/Codex/orangepi_bx72_vision_20260922/.ssh_lib"
        if existing.is_dir():
            sys.path.insert(0, str(existing))
            return importlib.import_module("paramiko")
        raise RuntimeError("当前Python缺少paramiko，请使用项目现有Python环境或安装依赖")


def remote_command(samples=8):
    """功能：生成固定采集器启动命令；samples为1至1000；返回安全编码命令文本。"""
    if type(samples) is not int or not 1 <= samples <= 1000:
        raise ValueError("采样数越界")
    code = Path(__file__).with_name("readonly_agent.py").read_text(encoding="utf-8")
    code += "\nentry(" + repr({"samples": samples, "period_s": 2.0}) + ")\n"
    payload = base64.b64encode(code.encode("utf-8")).decode("ascii")
    return "python3 -u -c \"import base64;exec(compile(base64.b64decode('" + payload + "'),'<pc-readonly-agent>','exec'))\""


def map_remote_command():
    """功能：生成固定地图采集器；无参数；返回SSH执行命令，不包含任意用户指令。"""
    folder=Path(__file__).parent
    code="\n".join((folder/name).read_text(encoding="utf-8") for name in ("readonly_agent.py","map_protocol.py","mapping_stream_agent.py","task_control.py"))
    payload=base64.b64encode((code+"\ntask_entry()\n").encode()).decode("ascii")
    return "python3 -u -c \"import base64;exec(compile(base64.b64decode('"+payload+"'),'<pc-map-agent>','exec'))\""


class ReadOnlyLink:
    """默认执行有限只读诊断；地图模式单独启用，只有显式目标请求才发送导航命令。"""

    def __init__(self):
        """功能：创建连接；无参数；返回无。"""
        self.stop_event = threading.Event()
        self.client = None
        self._lock = threading.Lock()

    def close(self):
        """功能：取消本会话并释放SSH资源；无参数；返回无，不改变车辆状态。"""
        self.stop_event.set()
        with self._lock:
            if self.client is not None:
                self.client.close()

    def run(self, host, callback, samples=8, mapping=False, requests=None):
        """功能：执行诊断或地图通信；host为地址、callback收事件、samples为组数、mapping选地图、requests可空；返回报告。"""
        host = host.strip()
        if not host or any(c.isspace() for c in host) or len(host) > 253:
            raise ValueError("请输入有效的香橙派IP或主机名")
        command = map_remote_command() if mapping else remote_command(samples)
        paramiko = ssh_library()

        class TrustedDevice(paramiko.MissingHostKeyPolicy):
            """仅接受与已有香橙派公钥完全相同的新地址，不保存新信任。"""
            def missing_host_key(inner, client, hostname, key):
                """功能：核对历史已信任设备公钥；参数由SSH库提供；返回无，错配拒绝连接。"""
                for alias in ("192.168.50.1", "192.168.1.106", "orangepi-wired", "orangepi-wifi"):
                    trusted = client.get_host_keys().lookup(alias)
                    if trusted and trusted.get(key.get_name()) == key:
                        return
                raise paramiko.SSHException("主机公钥未匹配已信任香橙派，未修改信任记录")

        result = {"status": "CONNECTING", "records": [], "motion_commands": 0, "firmware_writes": 0,
                  "remote_files_written": 0}
        started = time.monotonic()
        client = paramiko.SSHClient()
        client.load_host_keys(str(Path.home() / ".ssh/known_hosts"))
        client.set_missing_host_key_policy(TrustedDevice())
        with self._lock:
            if self.stop_event.is_set():
                return {**result, "status": "CANCELLED"}
            self.client = client
        journal = None
        writer_done = threading.Event()
        try:
            callback({"type": "link", "message": "正在核对香橙派SSH身份…"})
            # 使用显式直连Socket，避免本机SSH代理规则误把局域网连接送到公网代理。
            config = paramiko.SSHConfig()
            config_path = Path.home()/'.ssh/config'
            if config_path.is_file():
                with config_path.open(encoding='utf-8-sig') as source:
                    config.parse(source)
            settings = config.lookup(host)
            for alias in ('orangepi-wifi','orangepi-wired'):
                if config.lookup(alias).get('hostname') == host:
                    settings = config.lookup(alias)
                    break
            address = settings.get('hostname',host)
            bind = settings.get('bindaddress')
            sock = socket.create_connection((address, 22), timeout=6, source_address=(bind,0) if bind else None)
            try:
                client.connect(settings.get('hostkeyalias',address), username="orangepi", key_filename=str(Path.home() / ".ssh/orangepi_voice_rsa"),
                               sock=sock, look_for_keys=False, allow_agent=False, timeout=6,
                               banner_timeout=8, auth_timeout=8)
            except Exception:
                sock.close()
                raise
            if self.stop_event.is_set():
                result["status"] = "CANCELLED"
                return result
            client.get_transport().set_keepalive(5)
            callback({"type": "link", "message": "SSH已认证，正在核对串口及FPGA…"})
            stdin, stdout, stderr = client.exec_command(command, timeout=10)
            if mapping:
                folder = Path(__file__).resolve().parents[3]/'build/pc_maze_station/sessions'
                folder.mkdir(parents=True,exist_ok=True)
                path = folder/(datetime.now().strftime('%Y%m%d_%H%M%S_%f')+'.jsonl')
                journal = path.open('x',encoding='utf-8',buffering=1)
                result['journal'] = str(path)
                callback({'type':'link','message':'本次地图与故障日志：'+path.name})
                def send_requests():
                    """独立发送按钮请求；无参数；返回无，停车不等下一帧地图，断线不重发。"""
                    while not writer_done.is_set() and not self.stop_event.is_set():
                        try:
                            item=requests.get(timeout=.1)
                        except queue.Empty:
                            continue
                        try:
                            stdin.write(json.dumps(item)+"\n");stdin.flush()
                            result['motion_commands']+=1
                        except Exception as exc:
                            callback({'type':'error','message':'请求传输中断，板端结果未确认：'+str(exc)})
                            client.close()
                            return
                if requests is not None:
                    threading.Thread(target=send_requests,daemon=True).start()
            else:stdin.close()
            stdout.channel.settimeout(12)
            for line in stdout:
                if self.stop_event.is_set():
                    break
                record = json.loads(line)
                record["pc_received_monotonic"] = time.monotonic()
                if journal:
                    journal.write(json.dumps(record,ensure_ascii=False)+'\n')
                result["records"].append(record)
                if mapping and len(result["records"])>5000:
                    del result["records"][:1000]
                    result["history_truncated"]=True
                callback(record)
            if self.stop_event.is_set():
                result["status"] = "CANCELLED"
            else:
                error = stderr.read(8192).decode("utf-8", "replace")
                code = stdout.channel.recv_exit_status()
                complete = any(r.get("type") == "complete" for r in result["records"])
                result["status"] = "PASS" if code == 0 and complete else "FAILED"
                if not mapping:
                    result["summary"] = summarize(result["records"])
                    if result["summary"]["samples"] != samples or not result["summary"]["wheels_before_zero"] or not result["summary"]["wheels_after_zero"]:
                        result["status"] = "FAILED"
                if error:
                    result["stderr"] = error
        except Exception as exc:
            result.update(status="CANCELLED" if self.stop_event.is_set() else "FAILED", error=str(exc))
            callback({"type": "error", "message": str(exc)})
        finally:
            writer_done.set()
            if journal:journal.close()
            client.close()
            with self._lock:
                self.client = None
            result["elapsed_s"] = round(time.monotonic() - started, 3)
            callback({"type": "finished", "status": result["status"]})
        return result


def summarize(records):
    """功能：汇总通信和样本推进证据；records为真实事件列表；返回统计，不推断端到端单向延迟。"""
    frames = [r for r in records if r.get("type") == "sample"]
    intervals = [(b["pc_received_monotonic"]-a["pc_received_monotonic"])*1000 for a,b in zip(frames,frames[1:])]
    acquisition = [r["acquisition_ms"] for r in frames]
    serial = [q["serial_rtt_ms"] for r in frames for q in r["records"]]
    progressing = {}
    for command, field in [("tof read " + str(i), "SEQ") for i in range(4)] + [("imu", "N")]:
        seqs = [q["fields"][field] for r in frames for q in r["records"] if q["command"] == command]
        progressing[command] = len(seqs) >= 2 and any(a != b for a,b in zip(seqs,seqs[1:]))
    return {"samples": len(frames), "acquisition_ms_max": max(acquisition, default=None),
            "serial_query_ms_max": max(serial, default=None), "pc_receive_interval_ms_max": max(intervals, default=None),
            "sequence_progress": progressing,
            "wheels_before_zero": any(r.get("type") == "wheels_before" and r.get("zero_output") is True for r in records),
            "wheels_after_zero": any(r.get("type") == "wheels_after" and r.get("zero_output") is True for r in records),
            "note": "串行采集非同时快照；接收间隔不等于网络延迟；通信通过不代表传感器或导航验收通过"}
