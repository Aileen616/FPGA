"""功能：在本机Tk窗口中验证编辑、选择、演示和缩放，保存界面截图；不连接硬件。"""
import json
from pathlib import Path
import tkinter as tk
import time
from PIL import ImageGrab

from station import Station, enable_dpi
from model import demo_map


def main():
    """功能：运行桌面界面检查；无参数；返回无，失败抛异常。"""
    out = Path(__file__).resolve().parents[3] / "build/pc_maze_station"
    out.mkdir(parents=True, exist_ok=True)
    enable_dpi()
    root = tk.Tk()
    app = Station(root)
    errors = []
    def callback_error(kind, value, trace):
        """功能：保存Tk回调异常；参数为Tk异常信息；返回无。"""
        errors.append(str(value))
    root.report_callback_exception = callback_error
    try:
        root.update()
        app.tool.set("添加节点")
        for x, y, kind in [(0, 0, "起点"), (3, 0, "拐角"), (3, -2, "终点")]:
            app.kind.set(kind)
            sx, sy = app.screen(x, y)
            app.canvas.event_generate("<Button-1>", x=int(sx), y=int(sy))
            root.update()
        assert len(app.graph.nodes) == 3
        app.tool.set("连接节点")
        app.selected = None
        for ident in (0, 1, 2):
            n = app.graph.nodes[ident]
            sx, sy = app.screen(n.x, n.y)
            app.canvas.event_generate("<Button-1>", x=int(sx), y=int(sy))
            root.update()
        assert app.graph.route(0, 2) == [0, 1, 2]
        app.tool.set("选择节点")
        app.selected = 2
        app.refresh()
        root.update()
        assert app.preview == [0, 1, 2]
        app.graph = demo_map()
        app.graph.current = 6
        app.selected = 2
        app.refresh()
        app.fit()
        root.update()
        assert app.preview == [6, 5, 3, 1, 2]
        app.animation_delay_ms = 10
        app.animate_route()
        deadline = time.monotonic()+5
        while app.animation_id is not None and time.monotonic() < deadline:
            root.update()
            time.sleep(.01)
        assert app.graph.current == 2 and app.animation_id is None
        app.start_demo()
        deadline = time.monotonic()+5
        while app.animation_id is not None and time.monotonic() < deadline:
            root.update()
            time.sleep(.01)
        assert len(app.graph.nodes) == 7 and len(app.graph.edges) == 6
        assert app.graph.current == 6 and app.animation_id is None
        app.selected = 2
        app.refresh()
        app.fit()
        root.attributes("-topmost", True)
        root.lift()
        root.update()
        time.sleep(.2)
        x, y = root.winfo_rootx(), root.winfo_rooty()
        ImageGrab.grab(bbox=(x, y, x+root.winfo_width(), y+root.winfo_height())).save(out / "station_preview.png")
        assert not errors, errors
        (out / "ui_smoke.json").write_text(json.dumps({"status": "PASS", "checks": ["画布添加节点", "画布连接节点", "路线选择", "完整地图适配窗口", "演示路线回放", "完整探索后返回终点", "截图"], "hardware_access": False}, ensure_ascii=False, indent=2), encoding="utf-8")
        print("UI_SMOKE_PASS")
    finally:
        app.close()


if __name__ == "__main__":
    main()
