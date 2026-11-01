# END23 SRAM 迷宫试跑版本

本目录是 2026-11-01 最近五次 SRAM 试跑所用 **END23／ALG4／MZ13／FIX2** 的原样源码与码流归档。240 个 RTL／固件源码文件以及最终 ROM、固件和 FS 均保持部署时字节不变；没有把刚发现的终点问题修复混入此版本。

**这是待修复的实车调试版本，完整迷宫和视觉终点结束尚未通过。** 用户确认最近车辆实际停在终点黑色挡板附近，但程序没有完成视觉终点确认。

## 版本与文件

| 项目 | 本版内容 |
|---|---|
| FPGA／处理器 | BX72，GW5AT-LV138PG484AC1/I0，PicoRV32 裸机软核 |
| UART | `END1 V=23 ALG=4 STATIC=0`、`MZ V=13`、`MZREF FIX=2` |
| JTAG 短标识 | `BA36`，存在版本碰撞，不能单独用于判断版本 |
| 完整 FS | [task4_maze_corner_fix_full.fs](program_files/task4_maze_corner_fix_full.fs) |
| FS SHA256 | `535394d3c9d3de5abc64faa5f8df71383207914951cee2b35870bd255e8e2db5` |
| 固件 | 65144 字节，[firmware.bin](firmware/firmware.bin)、[ram32.hex](firmware/ram32.hex) |
| 导航源码 | [maze_tof.c](fw/maze_tof.c)、[板级接入](fw/maze_board_static.inc) |
| 视觉源码 | [endpoint_black.c](fw/endpoint_black.c)、[识别服务](fw/endpoint_service.inc) |
| 构建记录 | [build.tcl](build.tcl)、[firmware_commands.json](firmware_commands.json)、[冻结输入](provenance/build_inputs.json) |
| 全文件校验 | [MANIFEST.json](MANIFEST.json) |

这里的“当前”指本次上传前最后经板端确认的 SRAM 版本，不是断电后的 Flash 启动版本。本次上传未访问硬件、未重新加载 SRAM、未写 Flash、未发车。

## 车辆和传感器配置

车长 260 mm、车宽 250 mm，通道净宽 600 mm。左右 ToF 从车身外缘向内缩进 60 mm，前后 ToF 位于边缘；四个探头均安装在对应侧面中点。小车通过麦克纳姆轮平移，保持初始车头方向。迷宫运动控制不使用编码器里程；原编码器和其他控制功能仍保留，轮状态查询用于核对实际输出。

| 物理方向 | ToF 标识 | 原始查询通道 |
|---|---|---|
| 前 | X2 | `tof read 0` |
| 右 | X3 | `tof read 1` |
| 后 | X1 | `tof read 3` |
| 左 | X4 | `tof read 2` |

`maze read` 按前、右、后、左的导航方向编号，与上表原始通道编号不同。终点摆放规则仍是初始车头前向尽头，使用挡板和黑色标记。

## 已有验证与已知问题

原首次右拐的参考代号卡点和短连续直角 CROSS 移交已修改；离线回归、最终 ROM 和完整码流审阅记录在 [review.json](provenance/review.json)。该历史报告的 `OFFLINE_PASS_HW_PENDING` 是构建时状态，后续硬件结果以本目录 evidence 和文档为准。

- 多次遥测观察到首次向右平移后再向前行驶。不能据此宣称完整迷宫通过。
- 早期 LEFT 对齐目标出现 17／46／44 mm，不在现有执行可达范围，导致请求为零而停留在 ALIGN；尚未修复。
- 最近 retest_004 为 STATE10／WHY5，retest_005 为 STATE10／WHY2。最终均核对四轮 PWM/TARGET/ACTIVE 为零；物理停车位置在终点挡板处是用户现场反馈。
- 终点触发被限制在“只剩来路”的死路拓扑，侧面仍有开口时不会发起终点确认；最近两次请求 TOKEN 始终为零。
- 运动终点接收使用的 `coverage_verified` 仍为零。静态 `maze goal 1` 的通过不能替代运动终点接入验收。
- SEQ90 回收画面几乎全暗，主要暗区域触边，板端与 O0／O2 C、Python 同帧均为零候选。前 ToF 约 151 mm，过近、遮挡、镜头朝向及曝光的具体原因尚未确定。
- 4 倍速度比例的离线模型终点停车未通过；动态比例尚未实车标定。外部 DVP／板级时序未签核。

详见 [首次直角及实车记录](docs/TASK4_MAZE_CORNER_FIX_20261101.md)、[终点排查](docs/TASK4_ENDPOINT_STOP_DIAGNOSIS_20261101.md)、[最近两次硬件摘要](evidence/hardware_summary.json)。所有问题均保留原状，本版没有增加运行总时限或输出比例限制。

## 校验与复建

在任意工作目录运行：

```powershell
python sram/end23-maze/tools/verify_snapshot.py
```

脚本仅核对文件、源码、ROM 与码流，不访问串口、JTAG 或电机。

冻结构建脚本保留原路径，便于与已部署版本逐字核对。按原环境复建时，将本目录复制到 `F:/FPGA/build/soc/task4_maze_corner_fix`；Gowin V1.9.12.03 的 `gw_sh.exe` 执行该目录 `build.tcl`，使用所附 ROM 完成综合、布局布线。C 固件的 26 条原始编译／链接／导出命令位于 `firmware_commands.json`，工具链来自 Gowin GMD_2025.01 的 `riscv-none-elf-*`。它们使用原绝对路径，不是已验证的任意路径独立构建入口；本次归档没有重新跑完整构建。

## SRAM 加载和运行接口

本目录所附 FS 是同一份已试跑码流。既有 FT232H 下载链使用：

```text
openFPGALoader -c bx72_ft232h --freq 2500000 -m --file-type fs -v task4_maze_corner_fix_full.fs
```

`-m` 配置 SRAM，断电丢失；本目录没有提供 Flash 烧写流程。加载本身不会自动发车。

板上现有接口包括 `endpoint status`、`maze status`、`maze ref`、`maze io`、`tof read 0..3`、`imu` 和 `enc live 0..3`。加载后需要静止 IMU 零偏采样及初始航向确认。试跑入口为 `maze trial <非零递增序号>`；`maze stop` 和原急停入口会停车。故障／结束状态不能用同一条试跑命令直接重启，目前通过重载同一码流重新初始化。

试跑模式使用原混合器，`TLIMIT=0`；生产动态许可与终点接收缺口详见上文。它不代表车辆已经具备完整终点导航能力。

## 归档范围

包含当前冻结源码、最终 ROM／固件、完整 FS、构建命令、校验清单、实车摘要和相关问题记录。未复制历史候选、厂商工具安装目录、私钥、SSH／Wi-Fi 配置、设备备份或自动发车脚本。图片和原始 RGB565 仅保留本轮 SEQ90 终点诊断帧。
