"""功能：将真实C几何闭环记录回放到桌面地图，验证毫米显示与离线选点，不连接硬件。"""
import json
from pathlib import Path
import time
import tkinter as tk
from PIL import ImageGrab
from station import Station,enable_dpi
from metric_map import MetricMap


def main():
    """功能：执行毫米地图桌面检查并截图；无参数；返回无，失败抛异常。"""
    rootdir=Path(__file__).resolve().parents[3]
    folder=rootdir/'build/soc/task4_encoder_map/host/ui';folder.mkdir(parents=True,exist_ok=True)
    records=[json.loads(line) for line in (rootdir/'build/soc/task4_encoder_map/host/mapping/map_replay.jsonl').read_text().splitlines()]
    enable_dpi();root=tk.Tk();app=Station(root);errors=[]
    def on_error(kind,value,trace):
        """功能：记录Tk异常；参数为Tk异常上下文；返回无。"""
        errors.append(str(value))
    root.report_callback_exception=on_error
    try:
        root.update();app.start_replay(records)
        deadline=time.monotonic()+40
        while app.animation_id is not None and time.monotonic()<deadline:
            root.update();time.sleep(.01)
        assert app.animation_id is None,'回放未完成'
        assert isinstance(app.graph,MetricMap) and len(app.graph.nodes)==8 and len(app.graph.edges)==7
        assert app.graph.pose['complete']==1
        app.selected=0;app.refresh();app.fit();root.update()
        assert app.preview[0]==app.graph.current and app.preview[-1]==0
        app.send_target();assert app.map_requests.empty(),'离线回放不得发送目标'
        assert str(app.goto_button['state'])=='disabled'
        saved=MetricMap.load(app.graph.export());assert saved.edges==app.graph.edges
        app.sidebar.select(app.telemetry_panel)
        root.attributes('-topmost',True);root.lift();root.update();time.sleep(.25)
        x,y=root.winfo_rootx(),root.winfo_rooty()
        ImageGrab.grab(bbox=(x,y,x+root.winfo_width(),y+root.winfo_height())).save(folder/'metric_map_preview.png')
        assert not errors,errors
        (folder/'result.json').write_text(json.dumps({'status':'PASS','nodes':8,'edges':7,'records':len(records),
            'checks':['真实C记录回放','毫米坐标与原始里程保留','所有节点连通','点击路线预览','导出导入','离线禁止下发'],
            'hardware_access':False},ensure_ascii=False,indent=2),encoding='utf-8')
        print('METRIC_UI_PASS',len(records),'records')
    finally:app.close()


if __name__=='__main__':main()
