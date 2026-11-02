"""功能：启动任务4电脑上位机，提供地图草图、演示和SSH只读通信验证。"""
import argparse
from datetime import datetime
import json
from pathlib import Path
import queue
import sys
import threading
import time
import tkinter as tk
from tkinter import ttk, filedialog, messagebox

from model import MapModel, KINDS, demo_map
from link import ReadOnlyLink
from metric_map import MetricMap
from map_protocol import target_request

BG = "#edf2f7"
INK = "#172b45"
BLUE = "#1672d4"
COLORS = {"start": "#159970", "goal": "#e45360", "deadend": "#8b98aa", "corner": "#508bdd", "junction": "#508bdd"}
STATES = ["空闲", "起步检查", "通道行走", "路口观察", "对齐", "路径决策", "通过路口", "获取参考", "终点观察", "等待", "故障", "任务完成", "探索耗尽", "人工停止"]
DEFAULT_HOST = "orangepi-wifi"
TASK_LABELS = {'calibrate':'标定','start':'开始探索','stop':'停止','reset':'重置任务'}


def enable_dpi():
    """功能：使Windows窗口坐标与截图像素一致；无参数；返回无，其他平台不操作。"""
    if sys.platform == "win32":
        import ctypes
        ctypes.windll.user32.SetProcessDPIAware()


class Station:
    """所有控件仅在主线程更新，通信线程通过队列传递真实记录。"""

    def __init__(self, root, host=DEFAULT_HOST):
        """功能：构建窗口；root为Tk窗口、host为可编辑连接地址；返回无。"""
        self.root = root
        root.title("BX72 · 任务4地图与节点导航上位机")
        root.geometry(f"{min(1240, root.winfo_screenwidth()-60)}x{min(820, root.winfo_screenheight()-90)}+25+25")
        root.minsize(1020, 640)
        root.configure(bg=BG)
        self.graph = MapModel()
        self.selected = None
        self.preview = []
        self.step = 40
        self.pan = [0, 0]
        self.events = queue.Queue()
        self.link = None
        self.worker = None
        self.connected_at = None
        self.latest_sample = None
        self.last_result = None
        self.records = []
        self.animation_id = None
        self.animation_delay_ms = 550  # 电脑演示每步间隔，毫秒；不参与实车控制。
        self.closed = False
        self.map_requests=queue.Queue()
        self.map_connected=False
        self.control_ready=False
        self.pending_action=None
        self.task_text=tk.StringVar(value="连接车辆后可操作 · 标定时请保持静止")
        self.board_text=tk.StringVar(value="等待板端状态")
        self.host = tk.StringVar(value=host)
        self.tool = tk.StringVar(value="选择节点")
        self.kind = tk.StringVar(value="路口")
        self.source_text = tk.StringVar(value="电脑草图 · 非实测地图")
        self.link_text = tk.StringVar(value="未连接")
        self.route_text = tk.StringVar(value="点击节点，预览从电脑参考节点出发的路线")
        self.position_text = tk.StringVar(value="实车位置：尚无节点遥测")
        self.metrics = tk.StringVar(value="暂无实机数据")
        self.sample_age = tk.StringVar(value="数据年龄：—")
        self._build()
        self._tick_id = root.after(100, self.drain)
        root.protocol("WM_DELETE_WINDOW", self.close)
        self.log("上位机已就绪。点击「连接车辆与地图」进入任务控制。")

    def _build(self):
        """功能：创建界面控件；无参数；返回无。"""
        style = ttk.Style()
        style.theme_use("clam")
        style.configure("TButton", font=("Microsoft YaHei UI", 10), padding=(10, 7),
                        background="#eef3f8",foreground=INK,bordercolor="#d3dfeb",relief="flat")
        style.map("TButton",background=[("active","#d9e7f4"),("disabled","#f3f5f7")],
                  foreground=[("disabled","#8290a0")])
        style.configure("TNotebook",background=BG,borderwidth=0)
        style.configure("TNotebook.Tab",padding=(12,7),font=("Microsoft YaHei UI",10))
        style.map("TNotebook.Tab",background=[("selected","white")],foreground=[("selected",INK)])
        style.configure("TCombobox", padding=5)
        header = tk.Frame(self.root, bg=INK, height=76)
        header.pack(fill="x")
        tk.Label(header, text="BX72  /  迷宫导航", fg="white", bg=INK,
                 font=("Microsoft YaHei UI", 20, "bold")).pack(side="left", padx=22, pady=18)
        controls = tk.Frame(header,bg=INK)
        controls.pack(side="right",padx=18)
        self.task_buttons={}
        for action,color in [('calibrate','#30577c'),('start','#086954'),('stop','#a93243'),('reset','#47566c')]:
            button=tk.Button(controls,text=TASK_LABELS[action],command=lambda a=action:self.send_task(a),
                font=("Microsoft YaHei UI",11,"bold"),relief="flat",borderwidth=0,
                bg=color,fg="white",activebackground=color,activeforeground="white",
                disabledforeground="#c5ceda",padx=17,pady=8,state="disabled")
            button.pack(side="left",padx=4)
            self.task_buttons[action]=button
        strip=tk.Frame(self.root,bg="#e2ecf4")
        strip.pack(fill="x")
        tk.Label(strip,textvariable=self.task_text,bg="#e2ecf4",fg=INK,anchor="w",
                 font=("Microsoft YaHei UI",10)).pack(side="left",padx=20,pady=5)
        tk.Label(strip,textvariable=self.board_text,bg="#e2ecf4",fg="#30577c").pack(side="right",padx=20)
        bar = tk.Frame(self.root, bg="white")
        bar.pack(fill="x", padx=14, pady=(12, 8))
        tk.Label(bar, text="香橙派地址", bg="white").pack(side="left", padx=10)
        ttk.Entry(bar, textvariable=self.host, width=19).pack(side="left", pady=10)
        self.connect_button = ttk.Button(bar, text="验证通信（8组）", command=self.connect)
        self.connect_button.pack(side="left", padx=8)
        self.map_button=ttk.Button(bar,text="连接车辆与地图",command=self.connect_map)
        self.map_button.pack(side="left",padx=3)
        self.disconnect_button = ttk.Button(bar, text="断开", command=self.disconnect, state="disabled")
        self.disconnect_button.pack(side="left")
        tk.Label(bar, textvariable=self.link_text, fg=BLUE, bg="white", anchor="w").pack(side="left", padx=14)
        body = tk.Frame(self.root, bg=BG)
        body.pack(fill="both", expand=True, padx=14)
        left = tk.Frame(body, bg="white")
        left.pack(side="left", fill="both", expand=True)
        tools = tk.Frame(left, bg="white")
        tools.pack(fill="x", padx=10, pady=8)
        ttk.Combobox(tools, textvariable=self.tool, values=["选择节点", "添加节点", "连接节点"], state="readonly", width=10).pack(side="left")
        ttk.Combobox(tools, textvariable=self.kind, values=list(KINDS.values()), state="readonly", width=6).pack(side="left", padx=5)
        ttk.Button(tools, text="新建", width=4, command=self.new_map).pack(side="left", padx=2)
        ttk.Button(tools, text="导入", width=4, command=self.import_map).pack(side="left", padx=2)
        ttk.Button(tools, text="导出", width=4, command=self.export_map).pack(side="left", padx=2)
        ttk.Button(tools, text="回放", width=4, command=self.load_replay).pack(side="left", padx=2)
        ttk.Button(tools, text="居中", width=4, command=self.fit).pack(side="right")
        tk.Label(left, textvariable=self.source_text, bg="#e9f0fa", fg="#345b89", anchor="w", padx=12, pady=8).pack(fill="x")
        self.canvas = tk.Canvas(left, bg="#f9fbfe", highlightthickness=0, width=700, height=430)
        self.canvas.pack(fill="both", expand=True)
        self.canvas.bind("<Configure>", self.draw)
        self.canvas.bind("<Button-1>", self.click)
        self.canvas.bind("<MouseWheel>", self.zoom)
        self.canvas.bind("<ButtonPress-2>", self.pan_start)
        self.canvas.bind("<B2-Motion>", self.pan_move)
        tk.Label(left, text="滚轮缩放 · 中键拖动 · 蓝线为已录入通道 · 橙线为选点路线", bg="white", fg="#61738a", pady=6).pack(fill="x")
        right = tk.Frame(body, bg="white", width=302)
        right.pack(side="right", fill="y", padx=(10, 0))
        right.pack_propagate(False)
        self.sidebar = ttk.Notebook(right)
        self.sidebar.pack(fill="both", expand=True)
        self.route_panel = tk.Frame(self.sidebar, bg="white")
        self.telemetry_panel = tk.Frame(self.sidebar, bg="white")
        self.sidebar.add(self.route_panel, text="节点与路线")
        self.sidebar.add(self.telemetry_panel, text="实机遥测")
        right = self.route_panel
        tk.Label(right, text="节点与路线", bg="white", fg=INK, font=("Microsoft YaHei UI", 13, "bold"), anchor="w").pack(fill="x", padx=14, pady=(14, 8))
        self.node_list = tk.Listbox(right, height=7, font=("Microsoft YaHei UI", 10), relief="flat", bg="#f3f6fa", exportselection=False)
        self.node_list.pack(fill="x", padx=14)
        self.node_list.bind("<<ListboxSelect>>", self.list_select)
        tk.Label(right, textvariable=self.route_text, bg="white", fg=INK, wraplength=270, anchor="w", justify="left").pack(fill="x", padx=14, pady=10)
        ttk.Button(right, text="设为电脑参考节点", command=self.set_origin).pack(fill="x", padx=14, pady=3)
        ttk.Button(right, text="演示完整探索与返回终点", command=self.start_demo).pack(fill="x", padx=14, pady=3)
        ttk.Button(right, text="演示沿预览路线移动", command=self.animate_route).pack(fill="x", padx=14, pady=3)
        self.goto_button=ttk.Button(right,text="前往选中节点",command=self.send_target,state="disabled")
        self.goto_button.pack(fill="x", padx=14, pady=3)
        right = self.telemetry_panel
        tk.Label(right, text="实机遥测", bg="white", fg=INK, font=("Microsoft YaHei UI", 13, "bold"), anchor="w").pack(fill="x", padx=14, pady=(14, 6))
        tk.Label(right, textvariable=self.position_text, bg="white", fg="#65758a", wraplength=270, justify="left", anchor="w").pack(fill="x", padx=14, pady=6)
        tk.Label(right, textvariable=self.metrics, bg="white", fg=INK, wraplength=270, justify="left", anchor="w").pack(fill="x", padx=14, pady=5)
        tk.Label(right, textvariable=self.sample_age, bg="white", fg="#65758a", anchor="w").pack(fill="x", padx=14, pady=3)
        ttk.Button(right, text="保存通信记录", command=self.save_report).pack(fill="x", padx=14, pady=8)
        tk.Label(right, text="标定时保持静止。开始探索前，将车摆在本轮起点并对准前方。", bg="white", fg="#526980", wraplength=270, justify="left").pack(fill="x", padx=14, pady=5)
        footer = tk.Frame(self.root, bg=BG)
        footer.pack(side="bottom",before=body,fill="x", padx=14, pady=8)
        self.logbox = tk.Text(footer, height=3, bg=INK, fg="#cee0f4", font=("Microsoft YaHei UI", 9), relief="flat", padx=10, pady=4, state="disabled")
        self.logbox.pack(fill="x")

    def log(self, text):
        """功能：追加有界日志；text为非空文本；返回无。"""
        self.logbox.configure(state="normal")
        self.logbox.insert("end", datetime.now().strftime("%H:%M:%S") + "  " + text + "\n")
        if int(self.logbox.index("end-1c").split(".")[0]) > 400:
            self.logbox.delete("1.0", "100.0")
        self.logbox.see("end")
        self.logbox.configure(state="disabled")

    def screen(self, x, y):
        """功能：网格转画布坐标；x/y为网格数值；返回像素坐标二元组。"""
        return (self.canvas.winfo_width()/2 + self.pan[0] + x*self.step,
                self.canvas.winfo_height()/2 + self.pan[1] + y*self.step)

    def draw(self, event=None):
        """功能：绘制当前地图；event可为空或Tk事件；返回无。"""
        c = self.canvas
        c.delete("all")
        w, h = c.winfo_width(), c.winfo_height()
        ox, oy = self.screen(0, 0)
        spacing=max(8,round(self.step*(100 if isinstance(self.graph,MetricMap) else 1)))
        for x in range(int(ox % spacing), w, spacing):
            c.create_line(x, 0, x, h, fill="#e6edf6")
        for y in range(int(oy % spacing), h, spacing):
            c.create_line(0, y, w, y, fill="#e6edf6")
        c.create_text(16, 17, text="↑ 初始前向    → 初始右向", fill="#8493a7", anchor="w")
        for a, b in self.graph.edges:
            u, v = self.graph.nodes[a], self.graph.nodes[b]
            c.create_line(*self.screen(u.x, u.y), *self.screen(v.x, v.y), fill="#a8c9e9", width=10, capstyle="round")
        for a, b in zip(self.preview, self.preview[1:]):
            u, v = self.graph.nodes[a], self.graph.nodes[b]
            c.create_line(*self.screen(u.x, u.y), *self.screen(v.x, v.y), fill="#f0a335", width=5, dash=(7, 4))
        for n in self.graph.nodes.values():
            x, y = self.screen(n.x, n.y)
            if n.id == self.graph.current:
                c.create_oval(x-21, y-21, x+21, y+21, outline="#23a39a", width=3)
            c.create_oval(x-12, y-12, x+12, y+12, fill=COLORS[n.kind], outline="#f0a335" if n.id == self.selected else "white", width=3)
            c.create_text(x, y, text=str(n.id), fill="white", font=("Consolas", 10, "bold"))
            c.create_text(x, y+29, text=KINDS[n.kind], fill=INK, font=("Microsoft YaHei UI", 10))
        if isinstance(self.graph,MetricMap) and self.graph.pose:
            pose=self.graph.pose
            px,py=self.screen(pose['x_mm'],-pose['y_mm'])
            if pose['valid']:
                c.create_rectangle(px-6,py-6,px+6,py+6,fill=INK,outline="white",width=2)
        if not self.graph.nodes:
            hint="等待板端节点数据" if isinstance(self.graph,MetricMap) else "选择「添加节点」开始绘图\n或点击右侧按钮查看探索演示"
            c.create_text(w/2, h/2, text=hint, fill="#8192a8", font=("Microsoft YaHei UI", 14), justify="center")

    def refresh(self):
        """功能：刷新地图来源、节点列表和路线；无参数；返回无。"""
        if isinstance(self.graph,MetricMap):
            names={'live':'板端编码器地图','replay':'离线仿真回放 · 非实车','recorded':'已保存的历史毫米地图'}
            self.source_text.set(names[self.graph.source]+" · 单位mm · 位移为里程估计")
        else:self.source_text.set("演示地图 · 模拟数据，未控制实车" if self.graph.source == "demo" else "电脑草图 · 非实测地图，坐标不代表毫米")
        self.node_ids = sorted(self.graph.nodes)
        self.node_list.delete(0, "end")
        for ident in self.node_ids:
            n = self.graph.nodes[ident]
            y=-n.y if isinstance(self.graph,MetricMap) else n.y
            self.node_list.insert("end", f"{ident:02d}   {KINDS[n.kind]}   ({n.x}, {y})")
            if ident == self.selected:
                self.node_list.selection_set(len(self.node_list.get(0, "end"))-1)
        self.preview = self.graph.route(self.graph.current, self.selected)
        self.route_text.set("电脑路线：" + " → ".join(map(str, self.preview)) if self.preview else "未选中目标，或目标尚未连通")
        self.draw()

    def click(self, event):
        """功能：处理地图编辑或选点；event为鼠标事件；返回无，错误显示提示。"""
        if self.animation_id is not None:
            return
        chosen = next((n.id for n in self.graph.nodes.values() if sum((a-b)**2 for a,b in zip(self.screen(n.x,n.y),(event.x,event.y))) < 22**2), None)
        try:
            if isinstance(self.graph,MetricMap):
                self.selected=chosen;self.refresh();return
            if self.tool.get() == "添加节点":
                if chosen is not None:
                    self.selected = chosen
                else:
                    ox, oy = self.screen(0, 0)
                    kind = next(k for k,v in KINDS.items() if v == self.kind.get())
                    self.selected = self.graph.add(round((event.x-ox)/self.step), round((event.y-oy)/self.step), kind)
            elif self.tool.get() == "连接节点" and chosen is not None:
                if self.selected is not None and self.selected != chosen:
                    self.graph.connect(self.selected, chosen)
                self.selected = chosen
            else:
                self.selected = chosen
            self.refresh()
        except ValueError as exc:
            messagebox.showinfo("地图校验", str(exc))

    def list_select(self, event=None):
        """功能：同步列表选择；event可为空；返回无。"""
        selected = self.node_list.curselection()
        if selected:
            self.selected = self.node_ids[selected[0]]
            self.preview = self.graph.route(self.graph.current, self.selected)
            self.route_text.set("电脑路线：" + " → ".join(map(str, self.preview)) if self.preview else "目标尚未连通")
            self.draw()

    def set_origin(self):
        """功能：设定电脑预览参考点；无参数；返回无，不改变实车位置。"""
        if isinstance(self.graph,MetricMap):
            self.log("毫米地图参考节点由板端记录更新");return
        if self.selected in self.graph.nodes:
            self.cancel_animation()
            self.graph.current = self.selected
            self.log("仅设置电脑参考节点：" + str(self.selected))
            self.refresh()

    def fit(self):
        """功能：缩放居中显示地图；无参数；返回无。"""
        if self.graph.nodes:
            xs = [n.x for n in self.graph.nodes.values()]
            ys = [n.y for n in self.graph.nodes.values()]
            scale=min((self.canvas.winfo_width()-120)/max(1,max(xs)-min(xs)),(self.canvas.winfo_height()-130)/max(1,max(ys)-min(ys)))
            self.step=max(.000001,min(.3,scale)) if isinstance(self.graph,MetricMap) else max(8,min(60,int(scale)))
            self.pan = [-(min(xs)+max(xs))*self.step/2, -(min(ys)+max(ys))*self.step/2]
        else:
            self.step, self.pan = 40, [0, 0]
        self.draw()

    def zoom(self, event):
        """功能：鼠标滚轮调整网格大小；event为滚轮事件；返回无。"""
        if isinstance(self.graph,MetricMap):self.step=max(.000001,min(10,self.step*(1.2 if event.delta>0 else 1/1.2)))
        else:self.step = max(8, min(80, self.step + (4 if event.delta > 0 else -4)))
        self.draw()

    def pan_start(self, event):
        """功能：记录拖动起点；event为鼠标事件；返回无。"""
        self.drag_at = (event.x, event.y)

    def pan_move(self, event):
        """功能：平移画布；event为鼠标拖动事件；返回无。"""
        if hasattr(self, "drag_at"):
            self.pan[0] += event.x-self.drag_at[0]
            self.pan[1] += event.y-self.drag_at[1]
            self.drag_at = (event.x, event.y)
            self.draw()

    def cancel_animation(self):
        """功能：取消电脑演示回调；无参数；返回无。"""
        if self.animation_id is not None:
            self.root.after_cancel(self.animation_id)
            self.animation_id = None

    def new_map(self):
        """功能：清空电脑地图；无参数；返回无，已有图需确认。"""
        if self.map_connected:
            self.log("请先断开地图接收，再编辑电脑草图");return
        if self.graph.nodes and not messagebox.askyesno("新建地图", "清空当前电脑地图？未导出的修改将丢失。"):
            return
        self.cancel_animation()
        self.graph, self.selected, self.preview = MapModel(), None, []
        self.step,self.pan=40,[0,0]
        self.refresh()

    def import_map(self):
        """功能：从用户选择文件导入并校验地图；无参数；返回无。"""
        if self.map_connected:
            self.log("请先断开地图接收，再导入历史地图");return
        path = filedialog.askopenfilename(filetypes=[("节点地图", "*.json")])
        if path:
            try:
                data=json.loads(Path(path).read_text(encoding="utf-8"))
                graph=MetricMap.load(data) if data.get('schema')=='bx72.metric-map.v1' else MapModel.load(data)
                self.cancel_animation()
                self.graph, self.selected = graph, None
                self.refresh()
                self.fit()
            except (ValueError, KeyError, TypeError, OSError) as exc:
                messagebox.showerror("导入失败", str(exc))

    def export_map(self):
        """功能：导出电脑地图；无参数；返回无。"""
        path = filedialog.asksaveasfilename(defaultextension=".json", filetypes=[("节点地图", "*.json")])
        if path:
            try:
                Path(path).write_text(json.dumps(self.graph.export(), ensure_ascii=False, indent=2), encoding="utf-8")
                self.log("地图已导出")
            except OSError as exc:
                messagebox.showerror("保存失败", str(exc))

    def start_demo(self):
        """功能：演示完整探索、记录终点后继续、最后返回终点；无参数；返回无。"""
        if self.map_connected:
            self.log("请先断开地图接收，再运行电脑演示");return
        if self.graph.nodes and self.graph.source == "draft" and not messagebox.askyesno("载入演示", "将替换当前电脑草图，是否继续？"):
            return
        self.cancel_animation()
        self.sidebar.select(self.route_panel)
        full = demo_map()
        walk = [0]
        def visit(u, parent):
            """功能：生成演示深度遍历；u为当前点、parent可为空；返回无。"""
            for v in reversed(full.neighbors(u)):
                if v != parent:
                    walk.append(v)
                    visit(v, u)
                    walk.append(u)
        visit(0, None)
        walk += full.route(0, 6)[1:]
        self.graph = MapModel("demo")
        self.step,self.pan=40,[0,0]
        self.selected = None
        self.log("开始模拟探索；首次发现终点后继续遍历，完整探索后再返回终点。")
        self._animate(walk, full, discover=True)

    def _animate(self, walk, full, discover=False):
        """功能：逐点播放电脑演示；walk为编号序列、full为参考图、discover为增量绘图开关；返回无。"""
        self.cancel_animation()
        previous = None
        index = 0
        def advance():
            """功能：推进一个电脑演示步骤；无参数；返回无。"""
            nonlocal index, previous
            self.animation_id = None
            if index >= len(walk):
                self.log("电脑演示完成，当前参考节点：" + str(self.graph.current))
                return
            ident = walk[index]
            if discover and ident not in self.graph.nodes:
                n = full.nodes[ident]
                self.graph.add(n.x, n.y, n.kind, n.id)
                self.log("模拟发现节点 " + str(ident) + " / " + KINDS[n.kind])
                if n.kind == "goal":
                    self.log("模拟终点已记录，继续探索剩余支路")
            if discover and previous is not None:
                self.graph.connect(previous, ident)
            self.graph.current = ident
            previous = ident
            index += 1
            self.refresh()
            self.animation_id = self.root.after(self.animation_delay_ms, advance)
        advance()

    def animate_route(self):
        """功能：播放电脑路线预览；无参数；返回无，不向实车下发任何内容。"""
        if isinstance(self.graph,MetricMap):
            self.log("毫米地图请用轨迹回放，或连接后明确下发目标节点");return
        if len(self.preview) > 1:
            self.graph.source = "demo"
            self.log("仅播放电脑路线演示，未控制实车")
            self._animate(list(self.preview), self.graph)

    def connect(self, mapping=False):
        """功能：后台启动8组实机只读验证；无参数；返回无。"""
        if self.worker and self.worker.is_alive():
            return
        self.link = ReadOnlyLink()
        self.records, self.last_result = [], None
        self.latest_sample = None
        self.sidebar.select(self.telemetry_panel)
        self.metrics.set("正在连接，尚无本次实机数据")
        self.position_text.set("实车位置：等待本次节点记录" if mapping else "实车位置：诊断接口不提供节点位置")
        self.connect_button.configure(state="disabled")
        self.map_button.configure(state="disabled")
        self.disconnect_button.configure(state="normal")
        host = self.host.get()
        self.map_connected=mapping
        self.map_requests=queue.Queue()
        if mapping:
            self.cancel_animation();self.graph=MetricMap();self.selected=None;self.refresh()
        def work():
            """功能：在线程中采集并投递完成报告；无参数；返回无。"""
            try:
                result = self.link.run(host, self.events.put, samples=8,mapping=mapping,requests=self.map_requests)
            except Exception as exc:
                result = {"status": "FAILED", "error": str(exc), "motion_commands": 0, "firmware_writes": 0}
            self.events.put({"type": "report", "result": result})
        self.worker = threading.Thread(target=work, daemon=True)
        self.worker.start()

    def connect_map(self):
        """功能：接收配套候选的持续节点地图；无参数；返回无，不自动发车或重连。"""
        self.connect(mapping=True)

    def task_availability(self):
        """根据实时任务链路和待确认操作更新按钮；无参数；无返回。"""
        for action,button in self.task_buttons.items():
            enabled=self.control_ready and (not self.pending_action or
                (action=='stop' and self.pending_action!='stop'))
            button.configure(state='normal' if enabled else 'disabled')

    def send_task(self,action):
        """发送一次任务操作；action 为顶部按钮的固定标识；无返回，回包前不重复排队。"""
        if (action not in TASK_LABELS or not self.map_connected or not self.control_ready or
                not isinstance(self.graph,MetricMap) or self.graph.source!='live' or
                self.task_buttons[action]['state']=='disabled'):
            return
        self.pending_action=action
        self.task_availability()
        message={'calibrate':'正在标定，请保持小车静止','start':'正在建立起点与航向参考',
                 'stop':'已请求停车，等待四轮回读','reset':'正在核对停车状态和任务重置接口'}[action]
        self.task_text.set(message)
        self.log(message)
        self.map_requests.put({'action':action,'session':self.graph.session})

    def send_target(self):
        """功能：发送明确选中的已知节点；无参数；返回无，离线回放不会下发。"""
        if self.map_connected and not self.pending_action and isinstance(self.graph,MetricMap) and self.graph.source=='live' and self.selected in self.graph.nodes:
            self.map_requests.put(target_request(self.graph.session,self.selected))
            self.log("已请求目标节点 "+str(self.selected)+"，等待板端应答")

    def load_replay(self):
        """功能：选择离线JSONL地图轨迹；无参数；返回无，不建立任何网络连接。"""
        if self.worker and self.worker.is_alive():
            self.log("请先断开通信，再载入离线回放");return
        path=filedialog.askopenfilename(filetypes=[("地图轨迹","*.jsonl")],
            initialdir=str(Path(__file__).resolve().parents[3]/'build/soc/task4_encoder_map/host/mapping'),initialfile='map_replay.jsonl')
        if path:
            try:
                records=[json.loads(line) for line in Path(path).read_text(encoding='utf-8').splitlines() if line.strip()]
                self.start_replay(records)
            except (ValueError,KeyError,OSError) as exc:messagebox.showerror("回放失败",str(exc))

    def start_replay(self,records):
        """功能：逐组回放离线轨迹；records为地图事件列表；返回无，按钮不启用实车下发。"""
        self.cancel_animation();self.graph=MetricMap('replay');self.selected=None
        pending=iter(records)
        def advance():
            """功能：推进一组地图回放；无参数；返回无。"""
            self.animation_id=None
            try:
                for _ in range(100):self.handle(next(pending))
            except StopIteration:
                self.log("离线里程与完整任务回放结束");return
            except (ValueError,KeyError,TypeError) as exc:
                self.log("回放数据错误："+str(exc));return
            self.animation_id=self.root.after(30,advance)
        advance()

    def disconnect(self):
        """功能：关闭本次SSH只读会话；无参数；返回无，不发送停车指令。"""
        if self.link:
            self.control_ready=False
            self.map_connected=False
            self.task_availability()
            self.goto_button.configure(state='disabled')
            self.link.close()
        self.link_text.set("正在断开通信…")

    def handle(self, record):
        """功能：在主线程处理一条真实通信记录；record为事件字典；返回无。"""
        kind = record.get("type")
        self.records.append(record)
        if len(self.records)>5000:del self.records[:1000]
        if kind=='task_status':
            state=record['state']
            name=STATES[state] if 0<=state<len(STATES) else str(state)
            im='标定中' if record['busy'] else '标定有效' if record['cal'] and record['valid'] and not record['fault'] else '未就绪'
            self.board_text.set(f"导航 {name}  ·  WHY {record['why']}     |     IMU {im}")
            if self.map_connected and not self.control_ready and not self.link.stop_event.is_set():
                self.control_ready=True
                self.task_text.set("任务控制已连接 · 标定时请保持静止")
                self.task_availability()
            return
        if kind=='task_result':
            self.log(record['message'])
            if self.pending_action==record['action'] or not self.pending_action:
                self.pending_action=None
                self.task_text.set(record['message'])
            self.task_availability()
            return
        if kind=='task_uart':return
        if kind in ('map_pose','map_node'):
            if not isinstance(self.graph,MetricMap):self.graph=MetricMap('live')
            if kind=='map_pose' and record['session']!=self.graph.session:
                self.selected=None
            old_count=len(self.graph.nodes)
            if self.graph.apply(record):
                if kind=='map_pose':
                    self.latest_sample=record.get('pc_received_monotonic',time.monotonic())
                    phases=['探索建图','返回终点','到点待命','节点导航','未找到终点']
                    phase=record['phase']
                    text=phases[phase] if 0<=phase<len(phases) else str(phase)
                    if phase==0 and record['count']==0 and record['current']<0:text='待启动／起步检查（尚无节点）'
                    prefix='回放' if self.graph.source=='replay' else '板端'
                    if self.graph.source=='live':self.link_text.set('节点地图已连接 · 持续接收')
                    self.position_text.set(f"{prefix}：({record['x_mm']}, {record['y_mm']}) mm\n最后确认节点：{record['current']}")
                    self.metrics.set(f"阶段：{text}\n完整探索：{bool(record['complete'])}\n终点节点：{record['goal']}\n里程有效：{bool(record['valid'])}")
                    self.goto_button.configure(state='normal' if self.map_connected and record['complete'] else 'disabled')
                self.refresh()
                if len(self.graph.nodes)!=old_count:self.fit()
            return
        if kind=='map_ack':
            self.log("板端"+("已接受" if record['accepted'] else "未接受")+"目标节点请求");return
        if kind == "link":
            self.link_text.set(record["message"])
            self.log(record["message"])
        elif kind == "error":
            self.link_text.set("通信失败")
            self.log("通信错误：" + record["message"])
        elif kind == "sample":
            self.latest_sample = record.get("pc_received_monotonic", time.monotonic())
            data = {r["command"]: r["fields"] for r in record["records"]}
            m, imu = data.get("maze status"), data["imu"]
            if m:
                state = STATES[m["STATE"]] if 0 <= m["STATE"] < len(STATES) else str(m["STATE"])
                navigation = f"导航：{state}  WHY={m['WHY']}"
            else:
                navigation = "当前固件未提供迷宫状态接口"
            ranges = []
            for i, label in enumerate(("前", "右", "左", "后")):
                tof = data["tof read " + str(i)]
                valid = tof.get("VALID") == 1 and tof.get("ERR") == 0 and tof.get("RANGE") == 0 and tof.get("AGE", 9999999) <= 450000
                ranges.append(f"{label}：{tof['MM']} mm" + ("" if valid else "（无效/过期）"))
            self.metrics.set(navigation + "\n" + "\n".join(ranges) +
                             f"\nIMU：VALID={imu['VALID']} CAL={imu['CAL']}\nFAULT={imu['FAULT']}\n本组采集：{record['acquisition_ms']:.0f} ms")
            self.link_text.set(f"收到第 {record['index']} / 8 组真实遥测")
            self.log(f"真实遥测 #{record['index']}，采集 {record['acquisition_ms']:.0f} ms；地图接口尚未接入")
        elif kind in ("wheels_before", "wheels_after"):
            self.log(("开始" if kind == "wheels_before" else "结束") + "四轮零输出：" + str(record["zero_output"]))
        elif kind == "firmware":
            self.log("真实板端身份：" + record["record"]["reply"][:150])
        elif kind == "capabilities":
            self.log("迷宫状态接口：" + ("支持" if record["maze_status"] else "当前固件不支持") + "；节点地图接口：未接入")
        elif kind == "report":
            self.last_result = record["result"]
            self.connect_button.configure(state="normal")
            self.map_button.configure(state="normal")
            self.map_connected=False;self.goto_button.configure(state='disabled')
            self.control_ready=False
            if self.pending_action:
                self.task_text.set(TASK_LABELS[self.pending_action]+"结果未确认，请核对车辆")
            else:self.task_text.set("任务控制已断开")
            self.pending_action=None
            self.board_text.set("离线 · 保留末次记录")
            self.task_availability()
            self.disconnect_button.configure(state="disabled")
            self.link_text.set("验证通过 · 会话已关闭" if self.last_result["status"] == "PASS" else "会话结束：" + self.last_result["status"])
            self.log("通信结果：" + self.last_result["status"])
            if self.last_result.get("error"):
                self.log(self.last_result["error"])

    def drain(self):
        """功能：处理队列并显示数据年龄；无参数；返回无，每100毫秒调用。"""
        if self.closed:
            return
        try:
            while True:
                record=self.events.get_nowait()
                try:self.handle(record)
                except (ValueError,KeyError,TypeError) as exc:self.log('遥测数据错误：'+str(exc))
        except queue.Empty:
            pass
        if self.latest_sample is not None:
            age = time.monotonic()-self.latest_sample
            self.sample_age.set(f"末组接收距今：{age:.1f} s" + (" · 历史数据" if age > 5 else ""))
        self._tick_id = self.root.after(100, self.drain)

    def save_report(self):
        """功能：保存本次通信报告；无参数；返回无，文件由用户选择。"""
        if not self.last_result:
            messagebox.showinfo("通信记录", "请先完成或断开一次通信验证")
            return
        path = filedialog.asksaveasfilename(defaultextension=".json", initialfile="communication_report.json")
        if path:
            try:
                Path(path).write_text(json.dumps(self.last_result, ensure_ascii=False, indent=2), encoding="utf-8")
                self.log("通信报告已保存")
            except OSError as exc:
                messagebox.showerror("保存失败", str(exc))

    def close(self):
        """功能：关闭界面及本会话；无参数；返回无，不修改车辆状态。"""
        self.closed = True
        self.cancel_animation()
        self.root.after_cancel(self._tick_id)
        if self.link:
            self.link.close()
        self.root.destroy()


def main():
    """功能：运行桌面程序或命令行通信验证；参数来自命令行；返回进程退出码。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default=DEFAULT_HOST)
    parser.add_argument("--probe", action="store_true")
    parser.add_argument("--samples", type=int, default=8)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    if args.probe:
        def show(record):
            """功能：输出简要通信进度；record为事件字典；返回无。"""
            print(json.dumps({k:v for k,v in record.items() if k not in ("records", "record")}, ensure_ascii=False), flush=True)
        result = ReadOnlyLink().run(args.host, show, args.samples)
        if args.output:
            args.output.parent.mkdir(parents=True, exist_ok=True)
            args.output.write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8")
        return 0 if result["status"] == "PASS" else 1
    enable_dpi()
    root = tk.Tk()
    Station(root, args.host)
    root.mainloop()
    return 0


if __name__ == "__main__":
    sys.exit(main())
