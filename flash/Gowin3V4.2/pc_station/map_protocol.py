"""功能：解析独立建图候选的紧凑ASCII地图协议，不连接设备。"""
import re

FIELDS={
    "MP": {"S","R","N","I","P","C","G","T","X","Y","V","MS"},
    "MN": {"S","R","ID","PA","EN","EX","TR","CO","X","Y","L","ER","V","G"},
    "ME": {"S","K","R","ID"}, "MG": {"OK"},
}


def parse_map_line(line):
    """功能：解析单行地图协议；line为文本，不可为空；返回事件字典，缺字段或错误帧抛异常。"""
    parts=line.strip().split()
    if not parts or parts[0] not in FIELDS:
        raise ValueError("当前固件未提供建图协议，或收到不匹配应答")
    values={}
    for token in parts[1:]:
        match=re.fullmatch(r"([A-Z]+)=(-?\d+)",token)
        if not match or match[1] in values:
            raise ValueError("地图字段格式错误或重复")
        values[match[1]]=int(match[2])
    if not FIELDS[parts[0]]<=values.keys():
        raise ValueError("地图字段不完整: "+line)
    prefix=parts[0]
    if prefix=="MP":
        return {"type":"map_pose","session":values["S"],"revision":values["R"],"count":values["N"],
                "current":values["I"],"phase":values["P"],"complete":values["C"],"goal":values["G"],
                "target":values["T"],"x_mm":values["X"],"y_mm":values["Y"],"valid":values["V"],"stamp_ms":values["MS"]}
    if prefix=="MN":
        return {"type":"map_node","session":values["S"],"revision":values["R"],"id":values["ID"],
                "parent":values["PA"],"entry":values["EN"],"exits":values["EX"],"tried":values["TR"],
                "completed":values["CO"],"x_mm":values["X"],"y_mm":values["Y"],"length_mm":values["L"],
                "return_error_mm":values["ER"],"valid":values["V"],"goal":values["G"]}
    if prefix=="ME":
        return {"type":"map_event","session":values["S"],"kind":values["K"],"revision":values["R"],"id":values["ID"]}
    return {"type":"map_ack","accepted":values["OK"]==1}


def target_request(session, ident):
    """功能：生成目标节点请求对象；session/ident为非负整数；返回仅含固定字段的字典。"""
    if type(session) is not int or session<0 or type(ident) is not int or not 0<=ident<32:
        raise ValueError("目标节点编号或地图会话无效")
    return {"session":session,"target":ident}
