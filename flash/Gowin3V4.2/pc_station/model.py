"""功能：管理电脑侧无环拓扑图；坐标仅表示绘图网格，不代表实测毫米。"""
from collections import deque
from dataclasses import dataclass, asdict

KINDS = {"start": "起点", "corner": "拐角", "junction": "路口", "deadend": "死路", "goal": "终点"}


@dataclass(frozen=True)
class Node:
    """节点编号、网格坐标和类型；编号非负，类型取KINDS中的键。"""
    id: int
    x: int
    y: int
    kind: str = "junction"


class MapModel:
    """电脑侧草图或演示地图；不从测距数据猜测真实节点。"""

    def __init__(self, source="draft"):
        """功能：创建空图；source为draft/demo，不可为空；返回无。"""
        if source not in ("draft", "demo"):
            raise ValueError("本版仅接收电脑草图和演示地图")
        self.source = source
        self.nodes = {}
        self.edges = set()
        self.current = None

    def add(self, x, y, kind="junction", node_id=None):
        """功能：新增绘图节点；坐标为整数，类型有效，编号可为空自动分配；返回编号。"""
        if kind not in KINDS or type(x) is not int or type(y) is not int or max(abs(x), abs(y)) > 100:
            raise ValueError("节点类型或网格坐标无效，坐标范围为-100至100")
        if len(self.nodes) >= 32:
            raise ValueError("第一版最多支持32个节点")
        if any((n.x, n.y) == (x, y) for n in self.nodes.values()):
            raise ValueError("该位置已有节点")
        for a, b in self.edges:
            u, v = self.nodes[a], self.nodes[b]
            if (u.x == v.x == x and min(u.y, v.y) < y < max(u.y, v.y)) or (u.y == v.y == y and min(u.x, v.x) < x < max(u.x, v.x)):
                raise ValueError("请先分开已有通道，再在通道内部添加节点")
        ident = max(self.nodes, default=-1) + 1 if node_id is None else node_id
        if type(ident) is not int or ident < 0 or ident in self.nodes:
            raise ValueError("节点编号无效或重复")
        if kind in ("start", "goal") and any(n.kind == kind for n in self.nodes.values()):
            raise ValueError("起点和终点各只能有一个")
        self.nodes[ident] = Node(ident, x, y, kind)
        if self.current is None:
            self.current = ident
        return ident

    def neighbors(self, ident):
        """功能：查询相邻节点；ident为已存在编号；返回排序后的编号列表。"""
        return sorted(b if a == ident else a for a, b in self.edges if ident in (a, b))

    def route(self, start, goal):
        """功能：搜索节点路线；起终点可为空；返回路径列表，未知或不连通返回空列表。"""
        if start not in self.nodes or goal not in self.nodes:
            return []
        parent = {start: None}
        queue = deque([start])
        while queue:
            u = queue.popleft()
            if u == goal:
                path = [u]
                while parent[path[-1]] is not None:
                    path.append(parent[path[-1]])
                return path[::-1]
            for v in self.neighbors(u):
                if v not in parent:
                    parent[v] = u
                    queue.append(v)
        return []

    def connect(self, a, b):
        """功能：连接无环轴向通道；a/b为不同的已有编号；返回无，非法连接抛异常。"""
        if type(a) is not int or type(b) is not int or a not in self.nodes or b not in self.nodes or a == b:
            raise ValueError("请选择两个不同的已有节点")
        edge = tuple(sorted((a, b)))
        if edge in self.edges:
            return
        u, v = self.nodes[a], self.nodes[b]
        if (u.x == v.x) == (u.y == v.y):
            raise ValueError("通道只能水平或竖直，转弯处请添加节点")
        if self.route(a, b):
            raise ValueError("本版禁止环路")
        for ident, other in ((a, b), (b, a)):
            if len(self.neighbors(ident)) >= 3:
                raise ValueError("只支持直角和T形节点，不支持十字路口")
            p, q = self.nodes[ident], self.nodes[other]
            direction = ((q.x > p.x) - (q.x < p.x), (q.y > p.y) - (q.y < p.y))
            for neighbor in self.neighbors(ident):
                n = self.nodes[neighbor]
                if direction == ((n.x > p.x) - (n.x < p.x), (n.y > p.y) - (n.y < p.y)):
                    raise ValueError("同一节点的同一方向只能连接一条通道")
        for n in self.nodes.values():
            if n.id not in (a, b) and ((u.x == v.x == n.x and min(u.y, v.y) < n.y < max(u.y, v.y)) or
                                      (u.y == v.y == n.y and min(u.x, v.x) < n.x < max(u.x, v.x))):
                raise ValueError("通道经过已有节点，请分段连接")
        for c, d in self.edges:
            if {a, b} & {c, d}:
                continue
            p, q = self.nodes[c], self.nodes[d]
            # 通道交叉会在图中产生未登记路口，因此也要拒绝。
            if u.x == v.x and p.y == q.y:
                crossing = min(p.x, q.x) <= u.x <= max(p.x, q.x) and min(u.y, v.y) <= p.y <= max(u.y, v.y)
            elif u.y == v.y and p.x == q.x:
                crossing = min(u.x, v.x) <= p.x <= max(u.x, v.x) and min(p.y, q.y) <= u.y <= max(p.y, q.y)
            else:
                crossing = (u.x == v.x == p.x == q.x and max(min(u.y,v.y), min(p.y,q.y)) < min(max(u.y,v.y),max(p.y,q.y))) or (u.y == v.y == p.y == q.y and max(min(u.x,v.x),min(p.x,q.x)) < min(max(u.x,v.x),max(p.x,q.x)))
            if crossing:
                raise ValueError("通道不能交叉或重叠，请在交点登记节点")
        self.edges.add(edge)

    def export(self):
        """功能：导出地图；无参数；返回可JSON序列化字典。"""
        return {"schema": "bx72.node-map.v1", "source": self.source, "coordinate_unit": "schematic_grid",
                "nodes": [asdict(n) for n in self.nodes.values()], "edges": [list(e) for e in sorted(self.edges)],
                "current": self.current}

    @classmethod
    def load(cls, data):
        """功能：校验并导入电脑地图；data为非空字典；返回新图，非法文件抛异常。"""
        if data.get("schema") != "bx72.node-map.v1" or data.get("coordinate_unit") != "schematic_grid":
            raise ValueError("不支持的地图格式或坐标单位")
        graph = cls(data.get("source", "draft"))
        for n in data["nodes"]:
            graph.add(n["x"], n["y"], n["kind"], n["id"])
        for a, b in data["edges"]:
            graph.connect(a, b)
        if data.get("current") is not None and data["current"] not in graph.nodes:
            raise ValueError("当前位置编号不存在")
        graph.current = data.get("current")
        return graph


def demo_map():
    """功能：创建明确标记的无环演示图；无参数；返回地图对象。"""
    graph = MapModel("demo")
    for x, y, kind in [(-4, 3, "start"), (-4, 0, "junction"), (-4, -3, "deadend"),
                       (0, 0, "junction"), (0, 3, "deadend"), (4, 0, "corner"), (4, -3, "goal")]:
        graph.add(x, y, kind)
    for a, b in [(0, 1), (1, 2), (1, 3), (3, 4), (3, 5), (5, 6)]:
        graph.connect(a, b)
    return graph
