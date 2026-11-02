"""功能：验证拓扑约束、路径查询和串口只读边界，不访问真实硬件。"""
import io
import json
import sys
import types
import unittest
from unittest.mock import patch

from model import MapModel, demo_map
import readonly_agent as agent
from link import remote_command, summarize


class MapTests(unittest.TestCase):
    """验证无环图、直角通道、导入及路径的外部行为。"""

    def test_route_roundtrip(self):
        """功能：验证所有节点对路线及地图往返；无参数；返回无。"""
        g = demo_map()
        restored = MapModel.load(json.loads(json.dumps(g.export())))
        self.assertEqual(g.export(), restored.export())
        for a in g.nodes:
            for b in g.nodes:
                path = restored.route(a, b)
                self.assertEqual((path[0], path[-1]), (a, b))
                self.assertEqual(len(path), len(set(path)))
                self.assertTrue(all(tuple(sorted(e)) in g.edges for e in zip(path, path[1:])))

    def test_cycle_rejected(self):
        """功能：验证禁止环路且失败不改图；无参数；返回无。"""
        g = MapModel()
        for x, y in [(0, 0), (2, 0), (2, 2), (0, 2)]:
            g.add(x, y)
        for a, b in [(0, 1), (1, 2), (2, 3)]:
            g.connect(a, b)
        before = g.export()
        with self.assertRaises(ValueError):
            g.connect(3, 0)
        self.assertEqual(before, g.export())

    def test_geometry_rejections(self):
        """功能：验证斜线、重复方向和穿过节点被拒绝；无参数；返回无。"""
        g = MapModel()
        for x, y in [(0, 0), (1, 1), (3, 0), (4, 0)]:
            g.add(x, y)
        with self.assertRaises(ValueError):
            g.connect(0, 1)
        with self.assertRaises(ValueError):
            g.connect(0, 3)
        g.connect(0, 2)
        with self.assertRaises(ValueError):
            g.add(2, 0)

    def test_crossing_rejected(self):
        """功能：验证不登记节点的通道交叉被拒绝；无参数；返回无。"""
        g = MapModel()
        for x, y in [(-2, 0), (2, 0), (0, -2), (0, 2)]:
            g.add(x, y)
        g.connect(0, 1)
        with self.assertRaises(ValueError):
            g.connect(2, 3)

    def test_four_way_rejected(self):
        """功能：验证只支持最多三通；无参数；返回无。"""
        g = MapModel()
        for x, y in [(0,0), (2,0), (-2,0), (0,2), (0,-2)]:
            g.add(x, y)
        for b in (1,2,3):
            g.connect(0,b)
        with self.assertRaises(ValueError):
            g.connect(0,4)

    def test_limits_and_unknowns(self):
        """功能：验证容量、未知目标和来源隔离；无参数；返回无。"""
        g = MapModel()
        for i in range(32):
            g.add(i, 0)
        with self.assertRaises(ValueError):
            g.add(32, 0)
        self.assertEqual(g.route(0, 1), [])
        self.assertEqual(g.route(None, 1), [])
        with self.assertRaises(ValueError):
            MapModel("live")
        with self.assertRaises(ValueError):
            MapModel.load({"schema": "wrong"})


def fixture(command):
    """功能：生成隔离的协议测试回复；command为只读指令；返回文本，不代表当前板卡。"""
    fixed = {"robot state": "R1 MODE=0 ACTIVE=0 FAULT=0", "endpoint status": "END1 V=23 ALG=4 STATIC=0",
             "maze status": "MZ V=13 STATE=13 WHY=10", "maze io": "MZIO OWN=0 TRIAL=0",
             "imu": "I783 VALID=1 CAL=1 FAULT=0 N=100"}
    if command.startswith("tof read "):
        return f"T4 CH={command[-1]} MM=230 VALID=1 SEQ=8 AGE=1000 ERR=0 RANGE=0"
    if command.startswith("enc live "):
        return f"ELV CH={command[-1]} BUSY=0 PWM=0 TARGET=0 ACTIVE=0"
    return fixed[command]


class FakeSerial:
    """仅在测试内模拟逐字节串口，用于检查真实执行器发出的全部字符。"""
    def __init__(self):
        """功能：建立假串口；无参数；返回无。"""
        self.sent, self.buffer = [], bytearray()

    def __enter__(self):
        """功能：进入测试串口上下文；无参数；返回自身。"""
        return self

    def __exit__(self, *args):
        """功能：退出测试上下文；args为异常信息，可为空；返回无。"""

    def open(self):
        """功能：模拟打开；无参数；返回无。"""

    def reset_input_buffer(self):
        """功能：清空假接收缓存；无参数；返回无。"""
        self.buffer.clear()

    def write(self, data):
        """功能：记录精确写入并形成测试回复；data为字节串；返回写入长度。"""
        self.sent.append(data)
        self.buffer.extend((fixture(data.decode().strip()) + "\r\n").encode())
        return len(data)

    def read(self, count):
        """功能：模拟串口读；count为正整数；返回不超过该长度的字节串。"""
        data = bytes(self.buffer[:count])
        del self.buffer[:count]
        return data


class ProtocolTests(unittest.TestCase):
    """测试只读白名单、错帧拒绝、静态核对和报告。"""

    def test_reject_motion_and_configuration(self):
        """功能：验证所有非查询命令在写串口前被拒绝；无参数；返回无。"""
        port = FakeSerial()
        for command in ("maze trial 1", "maze start 0", "imu bias", "imu axis 2 1", "endpoint run 0", "stop", "~", "maze status\r\nmaze trial 1"):
            with self.assertRaises(ValueError):
                agent.query(port, command)
        self.assertEqual(port.sent, [])

    def test_mismatched_reply(self):
        """功能：验证旧前缀、缺字段及错通道不冒充新回复；无参数；返回无。"""
        for command, reply in [("maze status", "MZ REJECT MOTOR=0"), ("tof read 0", fixture("tof read 1")), ("imu", fixture("maze status"))]:
            with self.assertRaises(ValueError):
                agent.parse_reply(command, reply)

    def test_busy_is_not_zero(self):
        """功能：验证BUSY回复不能作为四轮停止证据；无参数；返回无。"""
        values = [{"fields": agent.parse_reply("enc live " + str(i), fixture("enc live " + str(i)))} for i in range(4)]
        self.assertTrue(agent.wheels_stopped(values))
        values[0]["fields"]["BUSY"] = 1
        self.assertFalse(agent.wheels_stopped(values))

    def test_v11_capability_fallback(self):
        """功能：验证旧版缺少迷宫接口时仍采集真实基础状态；无参数；返回无。"""
        port = FakeSerial()
        original = fixture
        def old_firmware(command):
            """功能：模拟旧版只拒绝迷宫命令；command为查询文本；返回测试回复。"""
            return "B2 REJECTED" if command == "maze status" else original(command)
        output = io.StringIO()
        with patch.dict(sys.modules, {"serial": types.SimpleNamespace(Serial=lambda **kw: port)}), \
             patch.object(agent.glob, "glob", return_value=["/dev/serial/by-id/usb-1a86_USB_Single_Serial_5910014290-if00"]), \
             patch.object(agent.subprocess, "run", return_value=types.SimpleNamespace(returncode=1)), \
             patch.object(agent.time, "sleep"), patch("sys.stdout", output), patch(__name__ + ".fixture", old_firmware):
            agent.run({"samples": 2, "period_s": 1})
        events = [json.loads(line) for line in output.getvalue().splitlines()]
        capability = next(r for r in events if r["type"] == "capabilities")
        self.assertFalse(capability["maze_status"])
        self.assertEqual(events[-1]["type"], "complete")
        self.assertNotIn(b"maze io\r\n", port.sent)
        frames = [r for r in events if r["type"] == "sample"]
        self.assertEqual(len(frames), 2)
        self.assertTrue(all(len(r["records"]) == 6 for r in frames))

    def test_complete_agent_reads_only(self):
        """功能：完整运行假串口采集并审计全部写入；无参数；返回无。"""
        port = FakeSerial()
        output = io.StringIO()
        device = "/dev/serial/by-id/usb-1a86_USB_Single_Serial_5910014290-if00"
        with patch.dict(sys.modules, {"serial": types.SimpleNamespace(Serial=lambda **kw: port)}), \
             patch.object(agent.glob, "glob", return_value=[device]), \
             patch.object(agent.subprocess, "run", return_value=types.SimpleNamespace(returncode=1)), \
             patch.object(agent.time, "sleep"), patch("sys.stdout", output):
            agent.run({"samples": 3, "period_s": 1})
        records = [json.loads(line) for line in output.getvalue().splitlines()]
        self.assertEqual(records[-1]["type"], "complete")
        self.assertEqual(len([r for r in records if r["type"] == "sample"]), 3)
        self.assertTrue(all(b.endswith(b"\r\n") and b.decode().strip() in agent.READ_COMMANDS for b in port.sent))
        self.assertEqual(len([b for b in port.sent if b.startswith(b"enc live ")]), 8)
        for i, r in enumerate(records):
            r["pc_received_monotonic"] = i
        summary = summarize(records)
        self.assertTrue(summary["wheels_before_zero"] and summary["wheels_after_zero"])
        self.assertFalse(any(summary["sequence_progress"].values()))

    def test_remote_command_bounded(self):
        """功能：验证采集命令只从固定源码生成且采样数受限；无参数；返回无。"""
        for bad in (-1, 0, 1001, "8; reboot", True):
            with self.assertRaises(ValueError):
                remote_command(bad)
        self.assertTrue(remote_command(8).startswith("python3 -u -c "))


if __name__ == "__main__":
    unittest.main(verbosity=2)
