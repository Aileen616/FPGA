"""功能：经SSH在内存中收发地图；复用前置readonly_agent定义的串口query和emit，无常驻安装。"""
import queue
import sys
import threading


def map_query(port, command):
    """功能：执行固定格式地图命令；port为串口，command为本模块生成文本；返回严格校验的回复。"""
    if command=="map pose":
        rule=("MP",{"S","R","N","I","P","C","G","T","X","Y","V","MS"})
    elif re.fullmatch(r"map node (?:[0-9]|[12][0-9]|3[01])",command):
        rule=("MN",{"S","R","ID","PA","EN","EX","TR","CO","X","Y","L","ER","V","G"})
    elif re.fullmatch(r"map event [0-9]{1,10}",command):
        rule=("ME",{"S","K","R","ID"})
    elif re.fullmatch(r"map goto (?:[0-9]|[12][0-9]|3[01])",command):
        rule=("MG",{"OK"})
    else:raise ValueError("禁止任意串口指令")
    READ_COMMANDS[command]=rule
    try:
        result=query(port,command)
        if command.startswith('map node ') and result['fields']['ID']!=int(command.rsplit(' ',1)[1]):
            raise ValueError('地图节点回复编号错配')
        return result
    finally:del READ_COMMANDS[command]


def map_poll(port,state,requests,checkpoint=None):
    """功能：完成一轮位置和节点同步；port为串口、state含会话/游标、requests为目标队列；返回当前位置信息。"""
    def read_map(command):
        """在每条串口事务前检查优先停车；command为固定地图命令，返回实际回复。"""
        if checkpoint:checkpoint()
        return map_query(port,command)
    record=read_map('map pose');pose=parse_map_line(record['reply']);emit(pose)
    if state['session']!=pose['session']:
        state['session']=pose['session'];state['acknowledged']=0
        for ident in range(pose['count']):emit(parse_map_line(read_map('map node '+str(ident))['reply']))
        state['acknowledged']=pose['revision']
    event=parse_map_line(read_map('map event '+str(state['acknowledged']))['reply'])
    if event['session']==state['session']:
        if event['kind']==2:
            for ident in range(pose['count']):emit(parse_map_line(read_map('map node '+str(ident))['reply']))
            state['acknowledged']=pose['revision']
        elif event['kind']==1:
            if event['revision']!=((state['acknowledged']+1)&0xffffffff):
                raise ValueError('地图增量通知序号错配')
            emit(parse_map_line(read_map('map node '+str(event['id']))['reply']))
            state['acknowledged']=event['revision']
    if requests is None:return pose
    try:
        request=requests.get_nowait()
        if request['session']==state['session']:
            ack=parse_map_line(map_query(port,'map goto '+str(request['target']))['reply']);ack['target']=request['target'];emit(ack)
        else:emit({'type':'error','message':'目标属于旧地图，未下发'})
    except queue.Empty:pass
    return pose


def map_stream():
    """功能：接收节点和里程，显式点击才提交目标；无参数；返回无，断线不改变板端自主任务。"""
    import serial
    device="/dev/serial/by-id/usb-1a86_USB_Single_Serial_5910014290-if00"
    if device not in glob.glob('/dev/serial/by-id/*') or subprocess.run(['fuser',device],capture_output=True).returncode!=1:
        raise RuntimeError("预期串口不存在或被占用，未抢占")
    requests=queue.Queue();finished=threading.Event()
    def receive_requests():
        """功能：读取电脑请求；无参数；返回无，仅传递会话和目标编号。"""
        try:
            for line in sys.stdin:
                try:
                    item=json.loads(line)
                    if type(item.get('target')) is int and 0<=item['target']<32 and type(item.get('session')) is int:
                        requests.put(item)
                    else:emit({'type':'error','message':'目标节点请求格式错误'})
                except (ValueError,TypeError):emit({'type':'error','message':'目标节点请求无法解析'})
        finally:finished.set()
    threading.Thread(target=receive_requests,daemon=True).start()
    state={'acknowledged':0,'session':None}
    with serial.Serial(port=None,baudrate=9600,timeout=.03,write_timeout=1,exclusive=True) as port:
        port.dtr=False;port.rts=False;port.port=device;port.open();time.sleep(.15);port.reset_input_buffer()
        while not finished.is_set():
            started=time.monotonic();map_poll(port,state,requests)
            time.sleep(max(0,.2-(time.monotonic()-started)))


def map_entry():
    """功能：输出地图通信错误；无参数；返回无，失败退出本会话，不发停车或重启命令。"""
    try:map_stream()
    except Exception as exc:
        emit({'type':'error','message':str(exc)})
        raise SystemExit(1)
