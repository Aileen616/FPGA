# BX72 FPGA 当前 Flash 版本

本次只保存已固化到 BX72 Flash 的 **V11**，目录：[flash/v11](flash/v11)。

- Flash 重载标识：`26AD`；UART 身份：`END1 V=11 ALG=3 STATIC=0`。
- 完整码流：[endpoint_integrated_budget_full.fs](flash/v11/program_files/endpoint_integrated_budget_full.fs)。
- SHA256：`02a6856ac1fc7579f897b4b451198adf5739b91eb6b6acfac34e3304a8532812`。
- 源码：`flash/v11/src`（RTL）、`flash/v11/fw`（裸机软核固件）；ROM：`flash/v11/firmware/ram32.hex`。
- 完整文件校验：[MANIFEST.json](flash/v11/MANIFEST.json)。

Flash 已完成写入、独立读回和重载检查；约一分钟静态相机、IMU、四路ToF检查通过。用户确认手机重新配对后连接成功。实物电机、物理手柄、完整实车任务与断电冷启动不在本次已验收范围内。

本版保留原电机控制通路与原屏显，新增终点视觉。终点识别约8～9.4秒/帧，不代表实时终点停车验收通过。

## 构建位置

冻结构建脚本保留原路径。复建时将 `flash/v11` 内容复制到 `F:/FPGA/build/soc/endpoint_integrated_budget`，使用 Gowin V1.9.12.03 的 `gw_sh.exe` 执行该目录 `build.tcl`，会生成综合和布局布线输出。构建使用所附最终ROM；重新编译C固件另需Gowin RV32工具链及编译配置。本次上传未重新执行完整构建。

仅上传此Flash版本，不包含未部署修复候选、工具安装包或本地连接凭据。原有 `tools/Test-GitEnvironment.cmd` 继续保留。
