# 最近任务4 END26重新加载SRAM：版本与四轮停机核对通过

用户要求“加载最近的任务4到sram”。沿用最近完成布局布线及静止红块验证的END26/ALG5/MZ15红色终点候选，没有修改导航、识别参数、输出或时限，没有重新综合。

## 实际结果

- 下载前实际UART为V11/ALG3，JTAG26AD；四轮真实BUSY/ACTIVE/PWM/TARGET均0。不能把此前END26记录当作本次下载前身份，也没有唯一认定何时或为何回到V11。
- 本轮仅调用一次下载器SRAM模式，日志Load SRAM / DONE，随后JTAG读到0011081B / 73EB / 00026020。
- UART实读END26/ALG5，迷宫MZ15。短码73EB与旧版重复，因此结合完整码流摘要、UART和下载日志核对。
- 最后OWN/TRIAL/COMMITS=0，四轮逐条BUSY/ACTIVE/PWM/TARGET=0，未发车。
- 后台视觉RUN1，读到SEQ18、19，末次25，均VALID1/ERROR0、COUNT0；三组末尾WERR0、FAILS保持1未增长。启动早期ERROR10保留，不称上电全过程零异常。
- 当前场景没有进行红块移放，本轮不重复宣称红色1/0/1实测通过；既往同一码流三态结果保留于run_001。
- IMU原始采样VALID1、FAULT0、RACE0，但CAL0。本轮未重新标定或设置航向。地图N0/I-1、里程V0，不能将未标定状态称为可直接发车或有效建图。

本轮计数：**SRAM1、Flash0、车辆启动0、IMU标定0**；END26候选累计SRAM2。没有完整SRAM内容读回，没有重新读取Flash，没有进行动态识别、停车距离或迷宫验收。

## 码流与必要工具恢复

完整FS：`build/soc/task4_red_presence/program_files/task4_red_presence_full.fs`，41852326字节。

SHA256：`4ba10a6456b9a017e48a68d15bde1521314802c9010a36c5ec600dc4245c2e02`。

文件、冻结fw/src摘要和此前PNR审阅一致。沿用此前内部setup +0.024ns、hold +0.175ns、违规端点0的审阅，本轮没有重新跑布局布线，也未新增板级外部时序签核。

当前香橙派新系统已枚举FT232H与原USB串口，但缺少工具。本轮安装python3-serial、openocd、libftdi1-2及其必要依赖，9个新包、未升级其他软件。从本地`build/spare_sd_20261102/loader-verified.tar.gz`只提取下载器文件，确认SHA256为`b6a06e720fe4058bb2e548565d8bde5f20be909df2556d14dd7b69004b5cb306`后恢复原路径，未使用不同版本替代。

本次通过有线管理上传到独立/dev/shm目录；固定JTAG锁路径在新系统中首次建立，之后同inode互斥。下载前后均检查实际串口停止状态，使用原配置，不操作物理复位脚。命令仅为`-m --file-type fs`，无Flash写入入口。无线仍保持“密码8个8”连接，PC10.119.174.43、香橙派10.119.174.53。

## 证据

当前驻留：`build/soc/task4_red_presence/current_hardware.json`。

本轮目录：`build/soc/task4_red_presence/hardware/reload_002/`。

- environment_before.json、tool_inventory.json、tools_restore.json、setup.json：设备和必要工具恢复。
- reload_manifest.json、upload/manifest.json：固定码流、下载器及支持文件绑定。
- before.json：V11及四轮零输出、实读JTAG。
- program.json：单次SRAM、实际加载后身份。
- after.json：三组视觉、实际四轮、IMU、地图与ToF只读结果。
- remote_copy/：下载器日志、JTAG日志及原始远端JSON；9个文件分别核对远端独立SHA256和本地回收摘要。
- remote_copy_comparison.json：完整回收校验；JSON另外按内容比较本地主流程记录，避免将Windows换行差异误报为证据不同。
- summary.json：最终结果；previous_current_hardware.json保留此前最后验证的END26记录，不将其冒充本次下载前状态。

本次授权的SRAM加载已完成，后续车辆启动需新的明确指令。
