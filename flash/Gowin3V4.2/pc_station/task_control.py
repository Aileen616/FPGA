"""任务工作台的内存中转程序，与地图采集共用一个串口。前置模块提供 query、map_poll 和 emit。"""

TASK_RULES = {
    'stop': ('B2', {'STOP'}), 'tof': ('B2', {'SEQ'}),
    'imu bias': ('I783', set()), 'imu axis 2 1': ('I783', set()),
    'imu calstats': ('ICSTAT', {'N', 'WHY'}), 'maze reset': ('MZ', set()),
}


def task_query(port, command):
    """按固定协议发送一条命令并记录回复；port 为串口，command 为本模块生成文本；返回应答。"""
    rule = TASK_RULES.get(command)
    if re.fullmatch(r'maze trial [1-9][0-9]{0,9}', command):
        rule = ('MZ', {'TRIAL'})
    if rule:
        READ_COMMANDS[command] = rule
    try:
        record = query(port, command)
        emit({'type': 'task_uart', 'record': record})
        return record
    finally:
        if rule:
            del READ_COMMANDS[command]


def perform_task(action, session, read, pose):
    """执行一次按钮动作；action、session 为已验证请求，read 和 pose 提供串口应答；返回结果说明。"""
    def stopped():
        """核对任务所有权和四轮输出；无参数；未能确认停稳时抛出异常。"""
        io = read('maze io')['fields']
        if io['OWN'] or io['TRIAL'] or not wheels_stopped([read('enc live '+str(i)) for i in range(4)]):
            raise ValueError('未确认停车，请检查任务所有权及四轮输出')

    if action == 'stop':
        if read('stop')['fields']['STOP'] != 1:
            raise ValueError('停车命令未获确认')
        stopped()
        state = read('maze status')['fields']
        return f"已确认停车 · 四轮零输出 · STATE={state['STATE']} WHY={state['WHY']}"
    before = pose()
    if before['session'] != session:
        raise ValueError('地图会话已改变，请按当前地图重新操作')
    stopped()
    if action == 'calibrate':
        if read('imu bias')['reply'] != 'I783 CALIBRATING=1000':
            raise ValueError('板端拒绝静止标定，请检查 IMU 并保持小车静止')
        while True:
            imu = read('imu')['fields']
            if not imu['BUSY']:
                break
        stats = read('imu calstats')['fields']
        if (imu['CAL'], imu['VALID'], imu['FAULT'], stats['N'], stats['WHY']) != (1,1,0,1000,0):
            raise ValueError(f"标定未通过 · N={stats['N']} WHY={stats['WHY']}")
        return '标定完成 · 1000 帧 · 可将车放到起点'
    if action == 'start':
        state = read('maze status')['fields']
        if state['STATE'] != 0 or before['count']:
            raise ValueError(f"板端未准备好新任务 · STATE={state['STATE']} WHY={state['WHY']}，请先重置任务")
        imu = read('imu')['fields']
        if (imu['CAL'], imu['VALID'], imu['FAULT'], imu['BUSY']) != (1,1,0,0):
            raise ValueError('IMU 尚未就绪，请停车标定')
        if read('imu axis 2 1')['reply'] != 'I783 AXIS_OK=1':
            raise ValueError('当前航向参考未获确认')
        seq = read('tof')['fields']['SEQ'] + 1
        if not 1 <= seq <= 0xffffffff or read('maze trial '+str(seq))['fields']['TRIAL'] != 1:
            raise ValueError('板端未接受本次启动')
        return '探索已启动 · 本次摆放位置为起点'
    if action == 'reset':
        imu = read('imu')['fields']
        reply = read('maze reset')['reply']
        if reply == 'MZ REJECT MOTOR=0':
            raise ValueError('当前固件不支持任务软重置，本轮地图和标定保留')
        if not re.search(r'\bRESET=1\b', reply):
            raise ValueError('板端未确认任务重置')
        after = pose()
        state = read('maze status')['fields']
        if (after['session']==session or after['count'] or after['current']!=-1 or
                after['complete'] or after['goal']!=-1 or state['STATE'] or state['WHY']):
            raise ValueError('重置应答与板端地图或故障状态不一致')
        if imu['CAL'] and read('imu')['fields']['CAL'] != 1:
            raise ValueError('板端已清图，但 IMU 标定失效，请重新标定')
        return '任务已重置 · 已确认板端空图及空闲状态'
    raise ValueError('未知任务操作')


def task_stream():
    """运行单串口地图与任务会话；无参数；SSH 输入结束后退出，不重试运动命令。"""
    import serial
    device = '/dev/serial/by-id/usb-1a86_USB_Single_Serial_5910014290-if00'
    if device not in glob.glob('/dev/serial/by-id/*') or subprocess.run(['fuser',device],capture_output=True).returncode != 1:
        raise RuntimeError('预期串口不存在或被占用')
    pending = queue.Queue()
    stop = threading.Event()
    finished = threading.Event()

    def receive():
        """接收工作台请求；无参数；停车置优先标志并撤销此前排队请求。"""
        try:
            for line in sys.stdin:
                item = json.loads(line)
                if item.get('action') == 'stop':
                    while not pending.empty():
                        try: pending.get_nowait()
                        except queue.Empty: break
                    stop.set()
                elif item.get('action') in ('calibrate','start','reset') and type(item.get('session')) is int:
                    pending.put(item)
                elif 'target' in item:
                    pending.put(target_request(item.get('session'),item['target']))
                else:
                    raise ValueError('任务请求格式错误')
        finally:
            finished.set()
    threading.Thread(target=receive,daemon=True).start()
    with serial.Serial(port=None,baudrate=9600,timeout=.03,write_timeout=1,exclusive=True) as port:
        port.dtr=False;port.rts=False;port.port=device;port.open()
        time.sleep(.15);port.reset_input_buffer()
        def read(command):
            """在事务边界让停车优先；command 为固定命令；返回实际应答，不自动重发。"""
            if stop.is_set() or finished.is_set():
                raise InterruptedError('操作已中断')
            return task_query(port,command)
        def pose():
            """读取板端地图会话；无参数；返回解析后的位姿记录。"""
            return parse_map_line(map_query(port,'map pose')['reply'])
        def handle_stop():
            """执行优先停车并核实四轮；无参数；返回无，失败也保留真实结果。"""
            if stop.is_set():
                stop.clear()
                try:
                    message = perform_task('stop',None,lambda cmd:task_query(port,cmd),pose)
                    ok = True
                except (ValueError,TimeoutError) as exc:
                    message = '停车结果未确认：'+str(exc)
                    ok = False
                emit({'type':'task_result','action':'stop','ok':ok,'message':message})
        state = {'session':None,'acknowledged':0}
        next_status = 0
        try:
            while not finished.is_set():
                handle_stop()
                try:
                    item = pending.get_nowait()
                except queue.Empty:
                    item = None
                if item is not None:
                    action = item.get('action','goto')
                    try:
                        if action == 'goto':
                            if item['session'] != pose()['session'] or stop.is_set():
                                raise ValueError('目标请求已失效')
                            ack = parse_map_line(map_query(port,'map goto '+str(item['target']))['reply'])
                            emit(dict(ack,target=item['target']))
                        else:
                            message = perform_task(action,item['session'],read,pose)
                            emit({'type':'task_result','action':action,'ok':True,'message':message})
                    except (ValueError,InterruptedError) as exc:
                        emit({'type':'task_result','action':action,'ok':False,'message':str(exc)})
                    handle_stop()
                map_poll(port,state,None,handle_stop)
                if time.monotonic() >= next_status:
                    nav = task_query(port,'maze status')['fields']
                    handle_stop()
                    imu = task_query(port,'imu')['fields']
                    emit({'type':'task_status','state':nav['STATE'],'why':nav['WHY'],'cal':imu['CAL'],
                          'valid':imu['VALID'],'fault':imu['FAULT'],'busy':imu['BUSY']})
                    next_status = time.monotonic()+1
                stop.wait(.05)
        finally:
            handle_stop()


def task_entry():
    """报告会话中断；无参数；失败时非零退出，动作结果以已收到的板端回包为准。"""
    try:
        task_stream()
    except Exception as exc:
        emit({'type':'error','message':str(exc)})
        raise SystemExit(1)
