# Gowin3V4.2

BX72 任务 4 已固化的 Flash 版本。板端身份为 **END27 / ALG5 / MZ16**，Flash 重载启动码为 **0x3876**。

- [完整 Flash 位流](program_files/task4_corner30_full.fs)的 SHA-256 为 `62cd983de2b5606dbb83bf00b1fe934d7efa4a305f45ef4becb52f870f738429`。
- [裸机固件源码](fw)、[RTL 与引脚约束](src)、[最终固件与 ROM](firmware)、[地图上位机源码](pc_station)。
- [Flash 验证记录](evidence/flash.json)显示，写入后对 5,222,400 字节配置载荷独立读回并核对，通过 Flash 重载和 UART 身份检查。整车断电冷启动未测试。
- 同一位流在 SRAM 实车试跑中曾完成 7 节点探索、返回终点 5，并接受指定节点 3 导航。其他试跑也出现过测距参考不可用或父节点匹配失败停车。Flash 重载验证为静态检查。

## 使用

上位机需要带 Tkinter 和 Paramiko 的 Python 环境，以及已配置的香橙派 SSH 连接。在版本目录运行 `python pc_station/station.py`。连接车辆后保持小车静止，点「标定」，再点「开始探索」。

在新检出的版本目录运行 `python tools/verify_snapshot.py` 校验发布清单。重新编译固件可运行 `python tools/rebuild_firmware.py --toolchain <Gowin RV32 工具链 bin 目录> --output <新的独立目录>`。

Gowin 综合和布局布线使用归档的最终 ROM。原始 `build.tcl` 和 `src/pico_define.vh` 固定指向 `F:/FPGA/build/soc/task4_corner30`。在此路径的副本运行 Gowin Standard 1.9.12.03 的 `gw_sh build.tcl`，若迁移目录，需同步调整这两份文件中的工程和 ROM 路径。
