"""功能：由电脑通过SSH在内存中执行的串口只读采集器，不安装服务或修改板端文件。"""
import glob
import json
import re
import subprocess
import time

READ_COMMANDS = {
    "robot state": ("R1", {"MODE", "ACTIVE", "FAULT"}),
    "endpoint status": ("END1", {"V"}),
    "maze status": ("MZ", {"V", "STATE", "WHY"}),
    "maze io": ("MZIO", {"OWN", "TRIAL"}),
    "imu": ("I783", {"VALID", "CAL", "FAULT", "N"}),
    **{"tof read " + str(i): ("T4", {"CH", "MM", "VALID", "SEQ", "AGE"}) for i in range(4)},
    **{"enc live " + str(i): ("ELV", {"CH", "BUSY", "PWM", "TARGET", "ACTIVE"}) for i in range(4)},
}


def emit(record):
    """功能：输出一条JSON遥测；record为可序列化对象，不可为空；返回无。"""
    print(json.dumps(record, ensure_ascii=True), flush=True)


def parse_reply(command, line):
    """功能：严格核对命令回复；command为白名单项、line为文本；返回字段字典，错配抛异常。"""
    prefix, required = READ_COMMANDS[command]
    values = {k: int(v) for k, v in re.findall(r"([A-Z][A-Z0-9]*)=(-?\d+)", line)}
    if line.split(" ", 1)[0] != prefix or not required <= values.keys():
        raise ValueError("回复格式不匹配: " + command + " / " + line[:180])
    if command.startswith(("tof read ", "enc live ")) and values["CH"] != int(command.rsplit(" ", 1)[1]):
        raise ValueError("串口回复通道错配")
    return values


def query(port, command, allow_unsupported=False):
    """功能：执行只读查询；port为串口、command在白名单、allow_unsupported仅供迷宫能力探测；返回回复记录。"""
    if command not in READ_COMMANDS:
        raise ValueError("禁止非只读命令")
    started = time.monotonic()
    port.write((command + "\r\n").encode("ascii"))
    line = bytearray()
    while time.monotonic() - started < 4:
        part = port.read(1)
        if part:
            line.extend(part)
            if len(line) > 4096:
                raise ValueError("串口单行超长")
            if part == b"\n":
                text = line.decode("ascii", errors="strict").strip()
                if not text or text == command:
                    line.clear()
                    continue
                if allow_unsupported and command in ("maze status", "maze io") and text in ("B2 REJECTED", "MZ REJECT MOTOR=0"):
                    return {"command": command, "reply": text, "fields": {}, "unsupported": True,
                            "serial_rtt_ms": round((time.monotonic() - started) * 1000, 2)}
                return {"command": command, "reply": text, "fields": parse_reply(command, text),
                        "serial_rtt_ms": round((time.monotonic() - started) * 1000, 2)}
    raise TimeoutError("串口查询超时: " + command)


def wheels_stopped(records):
    """功能：检查真实四轮回复；records为4条记录；返回是否全部非BUSY且零输出。"""
    return len(records) == 4 and all(r["fields"].get("BUSY") == 0 and
        all(r["fields"].get(k) == 0 for k in ("PWM", "TARGET", "ACTIVE")) for r in records)


def run(options):
    """功能：有限采集；options含samples和period_s；返回无，故障输出错误后退出。"""
    import serial
    device = "/dev/serial/by-id/usb-1a86_USB_Single_Serial_5910014290-if00"
    if device not in glob.glob("/dev/serial/by-id/*"):
        raise RuntimeError("未发现预期FPGA串口；未自动选择其他设备")
    busy = subprocess.run(["fuser", device], capture_output=True)
    if busy.returncode != 1:
        raise RuntimeError("串口已占用或无法核对占用状态，不抢占现有程序")
    count = options.get("samples", 8)
    period = options.get("period_s", 2.0)
    if type(count) is not int or not 1 <= count <= 1000 or not isinstance(period, (int, float)) or not 1 <= period <= 30:
        raise ValueError("采样参数越界")
    with serial.Serial(port=None, baudrate=9600, timeout=.03, write_timeout=1, exclusive=True) as port:
        # 先关闭握手控制，避免打开串口时主动改变板卡复位或启动状态。
        port.dtr = False
        port.rts = False
        port.port = device
        port.open()
        time.sleep(.15)
        port.reset_input_buffer()
        robot = query(port, "robot state")
        emit({"type": "identity", "record": robot})
        if robot["fields"]["ACTIVE"] != 0:
            raise RuntimeError("车辆处于活动状态，本轮静态通信验证结束，未发送停车或启动命令")
        before = [query(port, "enc live " + str(i)) for i in range(4)]
        emit({"type": "wheels_before", "records": before, "zero_output": wheels_stopped(before)})
        if not wheels_stopped(before):
            raise RuntimeError("四轮未确认零输出，停止静态查询")
        emit({"type": "firmware", "record": query(port, "endpoint status")})
        maze_status = query(port, "maze status", allow_unsupported=True)
        maze_commands = []
        capabilities = {"type": "capabilities", "maze_status": False, "maze_io": False,
                        "map_nodes": False, "records": [maze_status]}
        if not maze_status.get("unsupported"):
            maze_commands.append("maze status")
            capabilities["maze_status"] = True
            maze_io = query(port, "maze io", allow_unsupported=True)
            capabilities["records"].append(maze_io)
            if not maze_io.get("unsupported"):
                maze_commands.append("maze io")
                capabilities["maze_io"] = True
        emit(capabilities)
        index = 0
        while index < count:
            started = time.monotonic()
            records = []
            for command in ["robot state"] + maze_commands + ["tof read 0", "tof read 1", "tof read 2", "tof read 3", "imu"]:
                record = query(port, command)
                records.append(record)
                if command == "robot state" and record["fields"]["ACTIVE"]:
                    raise RuntimeError("检测到外部启动，本轮静态查询结束")
            index += 1
            emit({"type": "sample", "index": index, "records": records,
                  "acquisition_ms": round((time.monotonic() - started) * 1000, 2)})
            if index < count:
                time.sleep(max(0, period - (time.monotonic() - started)))
        after = [query(port, "enc live " + str(i)) for i in range(4)]
        emit({"type": "wheels_after", "records": after, "zero_output": wheels_stopped(after)})
        if not wheels_stopped(after):
            raise RuntimeError("结束四轮未确认零输出")
        emit({"type": "complete", "samples": index, "motion_commands": 0, "firmware_writes": 0})


def entry(options):
    """功能：捕获错误并输出结构化诊断；options为运行配置字典；返回无。"""
    try:
        run(options)
    except Exception as exc:
        emit({"type": "error", "message": str(exc), "motion_commands": 0, "firmware_writes": 0})
        raise SystemExit(1)
