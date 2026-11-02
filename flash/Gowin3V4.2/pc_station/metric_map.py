"""功能：从板端节点和里程记录维护真实毫米地图；不靠坐标近似合并节点。"""
from model import MapModel,Node


class MetricMap(MapModel):
    """父子连接来自板端拓扑，绘图y取反；raw保存原始毫米坐标和里程有效标记。"""

    def __init__(self, source="live"):
        """功能：创建板端或回放地图；source为live/replay/recorded；返回无。"""
        super().__init__()
        if source not in ("live","replay","recorded"):
            raise ValueError("毫米地图来源无效")
        self.source=source
        self.session=None
        self.raw={}
        self.pose=None
        self.revision=0

    def apply(self,event):
        """功能：应用真实协议或回放记录；event为字典；返回是否更新，旧会话节点返回false。"""
        if event.get("type") not in ("map_pose","map_node"):
            return False
        session=event["session"]
        if type(session) is not int or session<0:
            raise ValueError("地图会话错误")
        if self.session is None or (event["type"]=="map_pose" and session!=self.session):
            self.nodes.clear();self.edges.clear();self.raw.clear();self.pose=None;self.current=None;self.revision=0
            self.session=session
        if session!=self.session:
            return False
        if event["type"]=="map_pose":
            if not 0<=event["count"]<=32 or not -1<=event["current"]<32:
                raise ValueError("地图节点数量或当前位置编号错误")
            if self.pose and ((event["stamp_ms"]-self.pose["stamp_ms"])&0xffffffff)>0x7fffffff:
                return False
            self.pose=dict(event);self.current=event["current"] if event["current"]>=0 else None
            self.revision=event["revision"]
            return True
        ident=event["id"];parent=event["parent"]
        if type(ident) is not int or not 0<=ident<32 or type(parent) is not int or not -1<=parent<32 or ident==parent:
            raise ValueError("地图节点或父节点编号错误")
        if not 0<=event["entry"]<4 or not 0<=event["exits"]<16 or event["length_mm"]<0:
            raise ValueError("地图方向或长度错误")
        old=self.raw.get(ident)
        if old and ((event["revision"]-old["revision"])&0xffffffff)>0x7fffffff:
            return False
        # 板端父节点是身份依据，里程漂移不会导致合并；同时拒绝协议中的错误环路。
        cursor=parent;seen={ident}
        while cursor>=0:
            if cursor in seen:raise ValueError("板端拓扑出现环路")
            if cursor not in self.raw:break
            seen.add(cursor);cursor=self.raw[cursor]["parent"]
        exits=event["exits"]
        kind="start" if parent<0 else "goal" if event["goal"] else "deadend" if exits.bit_count()==1 else "junction" if exits.bit_count()>=3 else "corner"
        self.raw[ident]=dict(event)
        self.nodes[ident]=Node(ident,event["x_mm"],-event["y_mm"],kind)
        self.edges={(p["parent"],i) if p["parent"]<i else (i,p["parent"]) for i,p in self.raw.items() if p["parent"] in self.nodes}
        return True

    def export(self):
        """功能：导出可回放毫米地图；无参数；返回字典，不将其标为当前在线地图。"""
        return {"schema":"bx72.metric-map.v1","coordinate_unit":"mm","source":self.source,
                "pose":self.pose,"nodes":list(self.raw.values())}

    @classmethod
    def load(cls,data):
        """功能：导入已保存毫米地图；data为字典；返回历史地图，非法内容抛异常。"""
        if data.get("schema")!="bx72.metric-map.v1" or data.get("coordinate_unit")!="mm":
            raise ValueError("毫米地图格式错误")
        graph=cls("recorded")
        if data.get("pose"):graph.apply(data["pose"])
        for event in data["nodes"]:graph.apply(event)
        return graph
