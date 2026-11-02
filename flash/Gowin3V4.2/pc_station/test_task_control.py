"""使用真实中转源码、模拟串口与桌面按钮检查任务操作，全程离线。"""
import base64
import json
import queue
import re
import sys
import tkinter as tk
from types import SimpleNamespace
import threading
import unittest
from unittest.mock import patch

from link import map_remote_command
from metric_map import MetricMap
from station import Station
from test_mapping_stream import Port


class TaskPort(Port):
    """模拟 END26 的静止、启动、停车及不支持软重置的回包。"""
    def __init__(self):
        """建立已标定的空闲板；无参数；无返回。"""
        super().__init__()
        self.active=0
        self.state=0
        self.count=0
        self.cal=1
        self.moving_wheel=False

    def write(self,data):
        """处理一个串口事务；data 为命令字节串；返回写入长度。"""
        command=data.decode().strip()
        replies={'maze status':f'MZ V=15 STATE={self.state} WHY=0',
                 'maze io':f'MZIO OWN={self.active} TRIAL={self.active}',
                 'imu':f'I783 VALID=1 CAL={self.cal} FAULT=0 BUSY=0 N=2365',
                 'imu calstats':'ICSTAT N=1000 WHY=0','tof':'B2 SEQ=3',
                 'imu bias':'I783 CALIBRATING=1000','imu axis 2 1':'I783 AXIS_OK=1',
                 'maze reset':'MZ REJECT MOTOR=0','stop':'B2 STOP=1','maze trial 4':'MZ TRIAL=1'}
        if command.startswith('enc live '):
            ch=int(command.split()[-1])
            reply=f'ELV CH={ch} BUSY=0 PWM={int(self.moving_wheel and ch==2)} ACTIVE=0 TARGET=0'
        elif command in replies:
            reply=replies[command]
        else:
            return super().write(data)
        if command=='maze trial 4':
            self.active=1;self.state=2
        elif command=='stop':
            self.active=0;self.state=13
        self.sent.append(data)
        self.buffer.extend((reply+'\r\n').encode())
        return len(data)


class TaskTests(unittest.TestCase):
    """复用打包后的远端代码，核查命令字节、板端状态和按钮的真实队列。"""
    def setUp(self):
        """载入中转代码但跳过设备入口；无参数；无返回。"""
        payload=re.search(r"b64decode\('([^']+)'",map_remote_command())[1]
        source=base64.b64decode(payload).decode()
        self.ns={}
        exec(compile(source.removesuffix('task_entry()\n'),'task-test','exec'),self.ns)
        self.ns['emit']=lambda record:None
        self.port=TaskPort()

    def action(self,name,session=1):
        """调实际任务处理；name 为按钮标识，session 为地图会话；返回结果文本。"""
        return self.ns['perform_task'](name,session,lambda c:self.ns['task_query'](self.port,c),
            lambda:self.ns['parse_map_line'](self.ns['map_query'](self.port,'map pose')['reply']))

    def test_calibrate_start_stop(self):
        """验证标定、建航向、单次启动和即时停车确认；无参数；无返回。"""
        self.assertIn('1000',self.action('calibrate'))
        self.action('start')
        self.assertLess(self.port.sent.index(b'imu axis 2 1\r\n'),self.port.sent.index(b'maze trial 4\r\n'))
        with self.assertRaises(ValueError):self.action('start')
        self.assertEqual(self.port.sent.count(b'maze trial 4\r\n'),1)
        n=len(self.port.sent)
        self.assertIn('四轮零输出',self.action('stop'))
        self.assertEqual(self.port.sent[n],b'stop\r\n')
        self.port.moving_wheel=True
        with self.assertRaisesRegex(ValueError,'未确认停车'):self.action('stop')

    def test_invalid_start_and_reset(self):
        """旧会话、未标定、故障或旧固件拒绝均不能报成功；无参数；无返回。"""
        with self.assertRaisesRegex(ValueError,'会话'):self.action('start',2)
        self.port.cal=0
        with self.assertRaisesRegex(ValueError,'IMU'):self.action('start')
        self.port.cal=1;self.port.state=10;self.port.count=2
        with self.assertRaisesRegex(ValueError,'重置'):self.action('start')
        with self.assertRaisesRegex(ValueError,'不支持'):self.action('reset')
        self.assertEqual(self.port.sent.count(b'maze reset\r\n'),1)
        self.assertFalse(any(s.startswith(b'maze trial') for s in self.port.sent))
        self.assertEqual((self.port.count,self.port.state,self.port.cal),(2,10,1))

    def test_ui_clicks_and_disconnect(self):
        """验证离线禁用、重复点击和在途停车、断线保留地图；无参数；无返回。"""
        root=tk.Tk();root.withdraw()
        app=Station(root)
        try:
            app.task_buttons['start'].invoke()
            self.assertTrue(app.map_requests.empty())
            app.map_connected=True;app.graph=MetricMap();app.graph.session=1
            app.link=SimpleNamespace(stop_event=threading.Event(),close=lambda:None)
            app.handle(dict(type='task_status',state=0,why=0,cal=1,valid=1,fault=0,busy=0))
            for name in ('calibrate','start','reset'):
                app.task_buttons[name].invoke()
                app.task_buttons[name].invoke()
                self.assertEqual(app.map_requests.get_nowait(),{'action':name,'session':1})
                self.assertTrue(app.map_requests.empty())
                self.assertEqual(app.task_buttons['stop']['state'],'normal')
                app.handle(dict(type='task_result',action=name,ok=False,message='测试应答'))
            app.task_buttons['start'].invoke()
            app.task_buttons['stop'].invoke()
            app.handle(dict(type='task_result',action='start',ok=False,message='已中断'))
            self.assertEqual(app.pending_action,'stop')
            app.handle(dict(type='report',result={'status':'FAILED'}))
            self.assertIn('停止结果未确认',app.task_text.get())
            self.assertEqual(app.graph.session,1)
            self.assertTrue(all(b['state']=='disabled' for b in app.task_buttons.values()))
        finally:
            app.close()

    def test_stop_during_start(self):
        """在建航向的串口事务中注入停车；无参数；确认先停车且不再下发启动命令。"""
        incoming=queue.Queue()
        stop_received=threading.Event()
        records=[]
        def lines():
            """模拟 SSH 输入；无参数；逐行返回请求，空值代表断开。"""
            while True:
                item=incoming.get()
                if item is None:return
                yield json.dumps(item)
                if item.get('action')=='stop':stop_received.set()
        class InterleavedPort(TaskPort):
            """模拟仅一个读写持有者的串口。"""
            def write(inner,data):
                """在航向回复前让停车消息进入队列；data 为串口字节；返回长度。"""
                if data==b'imu axis 2 1\r\n':
                    incoming.put({'action':'stop','session':1})
                    if not stop_received.wait(2):raise AssertionError('停车消息未接收')
                return super().write(data)
            def __enter__(inner):return inner
            def __exit__(inner,*exc):pass
            def open(inner):pass
            def reset_input_buffer(inner):pass
        port=InterleavedPort()
        def emit(record):
            """模拟工作台发起单次启动，收到停车核实后断开；record 为事件；无返回。"""
            records.append(record)
            if record['type']=='task_status' and not any(r.get('action')=='stop' for r in records):
                incoming.put({'action':'start','session':1})
            elif record.get('action')=='stop':
                incoming.put(None)
        self.ns['emit']=emit
        device='/dev/serial/by-id/usb-1a86_USB_Single_Serial_5910014290-if00'
        with patch.dict(sys.modules,{'serial':SimpleNamespace(Serial=lambda **kw:port)}), \
                patch.object(sys,'stdin',lines()),patch.object(self.ns['glob'],'glob',return_value=[device]), \
                patch.object(self.ns['subprocess'],'run',return_value=SimpleNamespace(returncode=1)):
            self.ns['task_stream']()
        self.assertNotIn(b'maze trial 4\r\n',port.sent)
        self.assertEqual(port.sent.count(b'stop\r\n'),1)
        self.assertTrue(any(r.get('action')=='stop' and r['ok'] for r in records))
