# END26/ALG5/MZ15：有线SRAM与红色终点静止验证

2026-11-02，用户授权基于当前小车版本完成布局布线、新码流SRAM下载及有线静止识别；明确不写Flash、不启动车辆，需要移放红块时通知。已完成一次SRAM加载和有红/无红/放回三状态检验。

## 结果

当前SRAM实读END26/ALG5/MZ15，候选`build/soc/task4_red_presence`。基线为最近实读END25/ALG4/MZ15的`task4_map_goal_seen`快照；仅替换红色检测器，原导航、电机、图像采集和引脚不变。用户摆位约600mm、方块35mm，距离没有独立尺量，不要求转角。

| 现场状态 | 两张有效新帧 | COUNT | 结论 |
| --- | --- | --- | --- |
| 棕板上放红块 | 60、61 | 1、1 | 有红通过 |
| 用户移走红块、棕板保留 | 126、127 | 0、0 | 无红通过 |
| 用户原处放回红块 | 199、200 | 1、1 | 放回通过 |

每次用户确认摆位后开始对应阶段，跳过第一张新发布图像，排除前一场景在途帧，再读取两张不同SEQ。所有验收帧VALID=1、ERROR=0，三组末尾WERR=0，FAILS保持1未增长。首组框为(51,21,8,8)/AREA51、(50,21,9,8)/AREA53，位于先前红块区域。

三组前后实际四轮BUSY/ACTIVE/PWM/TARGET全0，最终OWN/TRIAL=0、COMMITS=0，后台RUN=1。这里的零输出是数字接口证据；未进行车辆行驶、自动接近或实际停车距离验收。未调用maze goal、未消费导航终点token，因此不把此次纯视觉通过称为MT_DONE或全迷宫通过。

本轮合计：**SRAM下载1、Flash写入0、车辆启动0、IMU标定0**。没有断电、重启香橙派、修复SD或更换网络配置。旧基线源码和FS未覆盖，旧硬件当前状态已标记被本候选取代并保存原记录。

## 码流与布局布线

- 完整FS：`build/soc/task4_red_presence/program_files/task4_red_presence_full.fs`，41852326字节。
- FS SHA256：`4ba10a6456b9a017e48a68d15bde1521314802c9010a36c5ec600dc4245c2e02`。
- 固件67668字节，SHA256：`9cfed2f7434f081e2f00ac267aad0cc844afe22c43352bd62990986b32b46788`。
- setup +0.024ns、hold +0.175ns、recovery +5.225ns、removal +0.264ns、最小脉宽 +2.482ns，setup/hold违规端点0，原引脚逐项一致。
- 使用原place1/route1、128KiB ITCM/64KiB DTCM；冻结输入在构建和下载后摘要均不变。
- JTAG IDCODE=0011081B，USERCODE=73EB与旧END25相同，不能单靠短码识别新版本；依据完整FS摘要及UART END26/ALG5确认。
- 未做完整SRAM内容读回，外部DVP板级时序没有新增签核；本次时序结论仅限已有工程约束下的内部审阅。

## 下载和检验过程中的保留记录

程序包经有线连接上传，校验既有SSH主机密钥；支持文件和下载器摘要逐项核对。远端全部新增文件位于/dev/shm；沿用原同inode的JTAG锁，以rb打开，不修改曾只读的SD挂载。实际只调用一次openFPGALoader SRAM模式，下载日志、JTAG及UART核对通过，没有Flash模式调用。

启动早期UART记录ERROR10、SEQ0，随后自动恢复正常识别；此早期采图失败保留，FAILS=1未清除。三组验收窗口内未新增失败。不能据此声称冷启动全流程始终无异常或任意场景识别率通过。

第一次阳性检验在已读到阳性新帧后，因主机工具白名单遗漏只读`endpoint box 0`而中止；固件未改、未重复下载。修正主机白名单后用新文件`positive_1_verified.json`完成正式阳性阶段，原`positive_1.json`保留。首个本地PNR启动检查另遇UTF-8读取配置问题，在运行Gowin前停止；修正文本读取后实际综合/PNR执行一次。

## 证据和复现入口

- 总结：`build/soc/task4_red_presence/hardware/run_001/summary.json`。
- 当前驻留：`build/soc/task4_red_presence/current_hardware.json`。
- 离线/PNR：候选`review.json`、`pnr_review.json`、`build.log`。
- 下载：`hardware/run_001/program.json`、`upload/manifest.json`及远端下载日志副本。
- 三态：`positive_1_verified.json`、`negative.json`、`positive_2.json`。
- 两次独立有线回收：`remote_verified_001/`、`remote_verified_002/`，9个证据文件分别进行远端独立SHA256计算及本地核对。
- 工具：`tools/pnr_task4_red_presence.py`、`review_task4_red_presence_pnr.py`、`hardware_task4_red_presence.py`。

下载阶段具备本轮目录防重复检查。复现时不得直接重跑已完成program阶段；需要新下载或发车时按新任务范围处理。已完成的三态静止验证不需要继续转角度或批量采图。

剩余范围：移动过程中识别、其他红色干扰物、距离边界、自动接近和终点停车均未验收。本次达到的是用户指定的最简静止可行性验证。
