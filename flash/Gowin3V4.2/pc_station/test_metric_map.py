"""功能：验证真实C协议输出到毫米地图的离线接入；不建立网络连接。"""
import json
from pathlib import Path
import unittest

from metric_map import MetricMap
from map_protocol import parse_map_line,target_request

ROOT=Path(__file__).resolve().parents[3]


def node(ident,parent,x,y,revision=1,session=1):
    """功能：创建协议测试节点；参数为编号、父节点、毫米坐标和版本会话；返回测试事件字典。"""
    return {'type':'map_node','session':session,'revision':revision,'id':ident,'parent':parent,'entry':2,
            'exits':3,'tried':1,'completed':0,'x_mm':x,'y_mm':y,'length_mm':abs(x)+abs(y),'goal':0,'valid':1}


class MetricTests(unittest.TestCase):
    """毫米图依据节点身份连接，允许计程漂移，不套用草图整数网格约束。"""
    def test_drift_not_identity(self):
        """功能：验证坐标漂移不重复建点或凭邻近合并；无参数；返回无。"""
        g=MetricMap();g.apply(node(0,-1,0,0));g.apply(node(1,0,1201,20));g.apply(node(2,1,1202,21))
        self.assertEqual(g.route(0,2),[0,1,2]);self.assertEqual(len(g.nodes),3)
        g.apply(node(1,0,1201,20,revision=2));self.assertEqual(len(g.nodes),3)
        self.assertEqual(g.nodes[1].y,-20)

    def test_out_of_order_and_cycle(self):
        """功能：验证旧版本不覆盖新值，错误父链不成环；无参数；返回无。"""
        g=MetricMap();g.apply(node(0,-1,0,0));g.apply(node(1,0,100,0,revision=3))
        self.assertFalse(g.apply(node(1,0,999,0,revision=2)));self.assertEqual(g.nodes[1].x,100)
        h=MetricMap();h.apply(node(1,2,1,0))
        with self.assertRaises(ValueError):h.apply(node(2,1,2,0))
        self.assertNotIn(2,h.nodes)

    def test_wire_from_compiled_board(self):
        """功能：解析真实板端C在MMIO模型生成的协议；无参数；返回无，不当作实物通信证据。"""
        path=ROOT/'build/soc/task4_encoder_map/host/controls/O2_run.log'
        if not path.exists():self.skipTest('请先运行板端离线回归')
        packets=[parse_map_line(line[len('MAP_WIRE '):]) for line in path.read_text(errors='replace').splitlines() if line.startswith('MAP_WIRE ')]
        self.assertTrue(any(p['type']=='map_pose' for p in packets))
        self.assertTrue(any(p['type']=='map_node' for p in packets))
        self.assertTrue(any(p['type']=='map_ack' and p['accepted'] for p in packets))

    def test_complete_geometry_replay(self):
        """功能：回放真实C几何仿真全部节点和里程事件；无参数；返回无。"""
        path=ROOT/'build/soc/task4_encoder_map/host/mapping/map_replay.jsonl'
        if not path.exists():self.skipTest('请先运行几何闭环仿真')
        g=MetricMap('replay')
        for line in path.read_text().splitlines():g.apply(json.loads(line))
        self.assertEqual(len(g.nodes),8);self.assertEqual(len(g.edges),7);self.assertTrue(g.pose['complete'])
        for a in g.nodes:
            for b in g.nodes:
                path=g.route(a,b);self.assertEqual(path[0],a);self.assertEqual(path[-1],b)
        saved=MetricMap.load(g.export())
        self.assertEqual(saved.edges,g.edges);self.assertEqual(saved.source,'recorded')

    def test_bad_wire_and_requests(self):
        """功能：验证缺字段、重复字段和命令注入被拒绝；无参数；返回无。"""
        for line in ('B2 REJECTED','MP S=1','MG OK=1 OK=0','MG OK=1;reboot'):
            with self.assertRaises(ValueError):parse_map_line(line)
        for ident in (-1,32,'1\r\nstop',True):
            with self.assertRaises(ValueError):target_request(1,ident)


if __name__=='__main__':unittest.main(verbosity=2)
