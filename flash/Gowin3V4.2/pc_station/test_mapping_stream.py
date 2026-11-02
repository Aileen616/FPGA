"""功能：在字节级假串口中执行实际地图中转源码，验证增量、补传和目标请求边界。"""
import json
from pathlib import Path
import queue
import unittest
from metric_map import MetricMap


class Port:
    """按固定协议回复的隔离串口；sent记录所有实际写入。"""
    def __init__(self):
        """功能：初始化测试串口；无参数；返回无。"""
        self.sent=[];self.buffer=bytearray();self.revision=2;self.count=2;self.event_kind=0
    def write(self,data):
        """功能：生成测试协议应答；data为完整查询字节串；返回写入长度。"""
        self.sent.append(data);command=data.decode().strip()
        if command=='map pose':reply=f'MP S=1 R={self.revision} N={self.count} I=0 P=0 C=0 G=-1 T=-1 X=10 Y=20 V=1 MS=100'
        elif command.startswith('map node '):
            ident=int(command.split()[-1]);parent=ident-1
            reply=f'MN S=1 R={self.revision} ID={ident} PA={parent} EN=2 EX=1 TR=1 CO=0 X=0 Y={ident*600} L=600 ER=0 V=1 G=0'
        elif command.startswith('map event '):
            after=int(command.split()[-1]);next_revision=(after+1)&0xffffffff if self.event_kind==1 else after
            reply=f'ME S=1 K={self.event_kind} R={next_revision} ID={self.count-1 if self.event_kind==1 else -1}'
        elif command.startswith('map goto '):reply='MG OK=1'
        else:raise AssertionError(command)
        self.buffer.extend((reply+'\r\n').encode())
        return len(data)
    def read(self,count):
        """功能：读测试串口字节；count为正整数；返回字节串。"""
        data=bytes(self.buffer[:count]);del self.buffer[:count];return data


class StreamTests(unittest.TestCase):
    """真实中转循环单步测试，不打开SSH或USB。"""
    def setUp(self):
        """功能：加载与SSH执行完全相同的源码；无参数；返回无。"""
        folder=Path(__file__).parent;self.ns={};self.received=[]
        for name in ('readonly_agent.py','map_protocol.py','mapping_stream_agent.py'):
            exec(compile((folder/name).read_text(encoding='utf-8'),name,'exec'),self.ns)
        self.ns['emit']=self.received.append
    def test_incremental_and_resync(self):
        """功能：验证首次全图、增量通知和日志溢出后补全；无参数；返回无。"""
        port=Port();state={'session':None,'acknowledged':0};requests=queue.Queue()
        self.ns['map_poll'](port,state,requests)
        self.assertEqual(state,{'session':1,'acknowledged':2})
        port.count=3;port.revision=3;port.event_kind=1
        self.ns['map_poll'](port,state,requests)
        self.assertEqual(state['acknowledged'],3)
        port.revision=30;port.event_kind=2
        self.ns['map_poll'](port,state,requests)
        self.assertEqual(state['acknowledged'],30)
        graph=MetricMap()
        for item in self.received:graph.apply(item)
        self.assertEqual(len(graph.nodes),3);self.assertEqual(graph.route(0,2),[0,1,2])
        self.assertFalse(any(b'map goto' in data for data in port.sent))
    def test_explicit_target_only(self):
        """功能：旧会话请求不发送，当前会话明确请求才下发；无参数；返回无。"""
        port=Port();state={'session':None,'acknowledged':0};requests=queue.Queue()
        requests.put({'session':0,'target':1});self.ns['map_poll'](port,state,requests)
        self.assertNotIn(b'map goto 1\r\n',port.sent)
        requests.put({'session':1,'target':1});self.ns['map_poll'](port,state,requests)
        self.assertEqual(port.sent.count(b'map goto 1\r\n'),1)
        self.assertTrue(any(r.get('type')=='map_ack' and r['accepted'] for r in self.received))
        for illegal in ('stop','maze trial 1','map goto 1\r\nstop'):
            with self.assertRaises(ValueError):self.ns['map_query'](port,illegal)


if __name__=='__main__':unittest.main(verbosity=2)
