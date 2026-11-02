# Gowin3 V4 — END26任务4 SRAM快照

本版原样归档2026-11-03发布前经板端查询确认的 **END26／ALG5／MZ15**：ToF迷宫导航、四轮编码器里程建图、完整探索与节点导航接口、红色终点检测。`Gowin3 V4`是GitHub发布名称，固件内部身份保持END26，没有为改版本号重新编译。

**完整迷宫和移动红色终点停车尚未验收通过。** 本次保存现状，不包含路口阈值修正，也没有修改程序、重烧板卡或再次发车。

## 版本绑定

| 内容 | 版本或文件 |
| --- | --- |
| 平台 | BX72 / GW5AT-LV138PG484AC1/I0 C，PicoRV32 RV32IMC裸机 |
| 存储 | ITCM128KiB、DTCM64KiB，保留4KiB栈 |
| UART | `END1 V=26 ALG=5 STATIC=0`，迷宫MZ15 |
| JTAG短标识 | 73EB，与旧END25重复，不能单独判版本 |
| 完整FS | [task4_red_presence_full.fs](program_files/task4_red_presence_full.fs) |
| FS SHA256 | `4ba10a6456b9a017e48a68d15bde1521314802c9010a36c5ec600dc4245c2e02` |
| 固件 | [firmware.bin](firmware/firmware.bin)，67668字节 |
| 固件SHA256 | `9cfed2f7434f081e2f00ac267aad0cc844afe22c43352bd62990986b32b46788` |
| ROM | [ram32.hex](firmware/ram32.hex)，128KiB镜像 |
| 源码 | [fw](fw)裸机C及接入、[src](src)RTL和引脚约束，共244个冻结文件 |

[发布前查询](evidence/publication_identity_check.json)读到END26/ALG5、RUN1/VALID1/ERROR0，迷宫OWN/TRIAL0。该查询没有读取完整SRAM；所附FS依据原下载记录、摘要及冻结源码绑定，不是运行时RAM/地图内存转储，也不保证每张图像检出目标。

## 功能与已知问题

车辆通过四向平移保持初始车头方向。通道模型600mm、车体260×250mm，左右ToF内缩60mm；物理方向前X2、右X3、后X1、左X4。四轮编码器记录节点坐标与边长，当前到达和路口几何判断仍主要依据ToF，不将里程记录等同于外部尺量。

- 完整探索模式先登记终点，再补齐分支并返回终点；已经完成地图时支持已知节点导航接口。不能把接口存在当成全迷宫实车通过。
- 红色算法检测100×60 RGB565图中面积至少16像素的红色连通区域，不验证方块语义。此前静止“有红／移走／放回”两帧结果各为1／0／1，见[静止证据](evidence/red_static_1_0_1.json)。移动识别、自动接近和终点停车尚未验收。
- **左直角漏报出口仍未修复。** 当前左右ToF原始≤385mm算墙、386～434mm为未知、≥435mm才算开口；实车左直角曾发布EX4，正确左拐应为EX12。登记瞬间完整缓存未冻结，物理原因尚未唯一认定。见[诊断报告](docs/TASK4_LEFT_CORNER_DIAGNOSIS_20261103.md)。
- 返回节点匹配、必要测距条件或几何等待可能停车；保留原30秒信息等待条件，不将其解释成新增试跑总时限。
- 开口定位可能产生低于执行范围的目标，例如EDGE352−300得到TARGET52mm，而原运动预算35+10+50=95mm，使ALIGN持续无推进。见[目标分析](docs/TASK4_MAP_RETURN_ANALYSIS_20261102.md)。
- 四向全通mask15按当前代码报不支持；当前采用无环树形节点模型、最多32节点。
- 核心请求速度和误差预算仍不能代替实车动态标定。前次PNR内部setup +0.024ns、hold +0.175ns，setup/hold违规端点0；不代表外部DVP/板级签核或整车功能通过。

实车记录与构建时报告分开保存于[evidence](evidence)、[provenance](provenance)。构建报告中的HW_PENDING是当时离线阶段状态，不抹去后续上板证据；实测失败也没有改写成通过。

## 校验和复建

在仓库根目录运行，只有文件核对，不连接硬件：

```powershell
python sram/gowin3-v4/tools/verify_snapshot.py
```

已经提供可直接使用的完整FS；不需要为了归档重新综合。原[build.tcl](build.tcl)、[firmware_commands.json](firmware_commands.json)保持冻结字节，含开发机绝对路径。新增辅助入口只在当前包或指定输出目录工作，不覆盖原开发目录：

```powershell
# 仅重新编译C到独立目录；需要Gowin RV32 GCC14.2工具链。
python sram/gowin3-v4/tools/rebuild_firmware.py --toolchain <riscv工具bin目录> --output <独立输出目录>

# 使用归档的最终ROM重新综合/布局布线；需要Gowin Standard 1.9.12.03。
gw_sh sram/gowin3-v4/tools/rebuild.tcl
```

C编译与RTL复建是独立操作：rebuild.tcl默认使用本包的归档ROM。修改C后必须把新ROM绑定到另一个独立候选再生成对应FS，不能将旧FS当作新程序。本次已在独立目录重新编译C，生成67668字节固件，SHA256与归档固件完全一致；没有重新执行完整FPGA综合/布局布线。

## SRAM加载与启动边界

使用已适配本板的FT232H下载器，手动命令为：

```text
openFPGALoader -c bx72_ft232h --freq 2500000 -m --file-type fs -v task4_red_presence_full.fs
```

`-m`为SRAM，断电或重新配置后丢失；本包没有Flash烧写或自动发车脚本。下载后核对UART END26/ALG5/MZ15及四轮实际输出，不能只看73EB。

KEY2/V17接逻辑复位，按当前RTL设计可重启软核、保留配置；复位清除本轮状态和IMU标定，重新任务仍需静止标定与显式启动。它不同于重新配置键。任务4当前试跑入口为`maze trial <非零递增序号>`；并没有可用的`maze reset`命令，不能直接将FAULT/STOPPED重新变成待机。现有手柄START绑定任务3，不是任务4开始键。

诊断接口包括endpoint status、maze status、maze io、maze ref、tof read、imu、enc live及map pose/node/event；用户明确请求的运动属于另外一次操作，不由本包验证工具触发。

## 归档范围

包含该SRAM版本的完整冻结fw/src、最终固件/ROM/FS、构建命令、必要验证摘要、已知问题说明及校验工具。电脑上位机正在独立维护，本快照没有把未冻结的电脑工作台变更混入板端版本。

未复制历史候选、工具安装包、香橙派系统备份、SSH/Wi-Fi配置、登录密码、私钥、原始通信日志或自动试车脚本。历史诊断文档保留原路径引用；当前包的入口和校验清单使用相对路径。Flash V11与END23目录保持原样。
