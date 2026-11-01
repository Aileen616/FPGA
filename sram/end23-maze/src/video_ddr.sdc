# 保持80A2控制、网络及HDMI时钟预算；DDR厂家PLL为50/200MHz。
create_clock -name osc_clk -period 47.619 [get_pins {internal_osc/OSCOUT}]
create_clock -name net_clk -period 8.000 [get_ports {rgmii_rx_clk_i}]
create_clock -name c0_pclk -period 16.000 [get_ports {c0_pclk}]
create_clock -name c1_pclk -period 16.000 [get_ports {c1_pclk}]
create_clock -name board_clk -period 20.000 [get_ports {Clk}]
# 厂家DDR时钟为200MHz，PHY的CLKDIV÷4产生50MHz用户时钟，必须显式约束。
create_generated_clock -name ddr_memory_clk -source [get_ports {Clk}] -multiply_by 4 [get_pins {memory/clock_generator/PLL_inst/CLKOUT2}]
create_generated_clock -name ddr_user_clk -source [get_pins {memory/clock_generator/PLL_inst/CLKOUT2}] -divide_by 4 [get_pins {memory/controller/gw3_top/u_ddr_phy_top/fclkdiv/CLKOUT}]
# HDMI像素时钟由串行时钟专用÷5产生，保留OSER10同源时序关系。
create_generated_clock -name hdmi_serial_clk -source [get_ports {rgmii_rx_clk_i}] -multiply_by 201 -divide_by 100 [get_pins {display/clock_generator/PLL_inst/CLKOUT0}]
create_generated_clock -name hdmi_pixel_clk -source [get_pins {display/clock_generator/PLL_inst/CLKOUT0}] -divide_by 5 [get_pins {display/pixel_divider/CLKOUT}]
# 仅豁免单比特握手首级，不切整个域。
set_false_path -to [get_pins {*req_meta*/D *ack_meta*/D *flush_meta*/D *done_meta*/D *pub_meta*/D heartbeat_meta*/D display/status_meta*/D memory_health_meta*/D}]
# Gray指针限制位间到达时间；正常二级同步仍按本域时钟分析。
set_max_delay 7.500 -to [get_pins {*rg_meta*/D *wg_meta*/D stats_meta*/D}]
set_false_path -hold -to [get_pins {*rg_meta*/D *wg_meta*/D stats_meta*/D}]
set_max_delay 12.000 -to [get_pins {*time_meta*/D}]
set_false_path -hold -to [get_pins {*time_meta*/D}]
# 多位握手负载保持至确认；两级同步后额外两拍等待再使用。
# 综合可能将bridge/held_addr合并至调用者，按准确目标限定，避免源层级筛选失效。
# dst_address仅在请求二级同步且额外等待两拍后锁存已保持的地址，保持20ns原预算。
set_max_delay 20.000 -to [get_pins {*bridge/dst_address*/D}]
set_false_path -hold -to [get_pins {*bridge/dst_address*/D}]
set_max_delay 20.000 -to [get_pins {*bridge/result*/D}]
set_false_path -hold -to [get_pins {*bridge/result*/D}]
set_max_delay 20.000 -from [get_pins {*writer/timestamp*/Q *writer/frame_valid*/Q}] -to [get_pins {buffer0/timestamp_us*/D buffer1/timestamp_us*/D buffer0/valid*/D buffer1/valid*/D}]
set_false_path -hold -to [get_pins {buffer0/timestamp_us*/D buffer1/timestamp_us*/D buffer0/valid*/D buffer1/valid*/D}]
set_max_delay 25.000 -to [get_pins {display/buffer*/read_meta*/D display/reader*/read_bank*/D}]
set_false_path -hold -to [get_pins {display/buffer*/read_meta*/D display/reader*/read_bank*/D}]
set_false_path -to [get_pins {display/done_meta*/D}]
# 复位由reset_sync异步断言、同步释放；只豁免同步器异步端。
set_false_path -to [get_pins {*reset/release_ff*/CLEAR display/fb_reset*/release_ff*/CLEAR display/lock_monitor/lock_release*/CLEAR}]
set_false_path -from [get_pins {display/video_startup/reset_n*/Q}] -to [get_pins {display/dvi/rgb2dvi_inst/u_OSER10*/RESET display/dvi/rgb2dvi_inst/TMDS8b10b_inst_*/*/CLEAR display/dvi/rgb2dvi_inst/TMDS8b10b_inst_*/*/PRESET}]
# 厂家PHY初始化路径：仅约束以下明确端点，正常DDR数据通路继续分析。
# DLL代码在LOCK后、初始化信号两级采样产生的单次CE窗口锁存，并非每个200MHz边沿更新。
set_max_delay 15.000 -to [get_pins {memory/controller/gw3_top/u_ddr_phy_top/dll_step_base*_s0/D}]
set_false_path -hold -to [get_pins {memory/controller/gw3_top/u_ddr_phy_top/dll_step_base*_s0/D}]
# CALIB是至少一个用户周期的调整脉冲，IDES8_MEM内部三级FCLK采样后检测上升沿。
# 首级不作相关时钟保持检查，但限制传输不超过一个FCLK周期，防止不受约束的长路径。
set_max_delay 5.000 -from [get_pins {memory/controller/gw3_top/u_ddr_phy_top/*/ides_calib_d_s0/Q}] -to [get_pins {memory/controller/gw3_top/u_ddr_phy_top/*u_ides8_mem/CALIB}]
set_false_path -hold -from [get_pins {memory/controller/gw3_top/u_ddr_phy_top/*/ides_calib_d_s0/Q}] -to [get_pins {memory/controller/gw3_top/u_ddr_phy_top/*u_ides8_mem/CALIB}]
# 重校准请求由厂家双触发器跨入连续50MHz域；仅首级D使用跨域例外。
set_max_delay 20.000 -to [get_pins {memory/controller/gw3_top/u_ddr_phy_top/ddr_sync/recalib_s_0_s0/D}]
set_false_path -hold -to [get_pins {memory/controller/gw3_top/u_ddr_phy_top/ddr_sync/recalib_s_0_s0/D}]
# 初始化在输出停钟或PLL尚未锁定时释放IO复位；这些RESET均由厂家ddr_rst驱动。
# 只选IO原语，避开同层FIFO的常数复位；以经过RESET引脚的路径约束异步检查。
set_false_path -through [get_pins {memory/controller/gw3_top/u_ddr_phy_top/u_ddr_phy_wds/*u_ides8_mem/RESET memory/controller/gw3_top/u_ddr_phy_top/u_ddr_phy_wds/*u_oser8_mem/RESET memory/controller/gw3_top/u_ddr_phy_top/u_ddr_phy_wds/*/u_dqs_gen/RESET memory/controller/gw3_top/u_ddr_phy_top/u_ddr_phy_wds/*/u_dqs/RESET memory/controller/gw3_top/u_ddr_phy_top/u_ddr_phy_wds/*u_cmd_gen/RESET memory/controller/gw3_top/u_ddr_phy_top/u_ddr_phy_wds/*u_ck_gen/RESET}]
# DVP/RGMII外部延迟仍需实测，不视为板级签核。

# UART和编码器仅同步单比特输入；从外部输入切至首级，内部同步链及保护正常计时。
set_false_path -from [get_ports {Rst_n bt_rx uart_rx encoder_a[*] encoder_b[*]}]


# 性能快照：仅首级toggle同步入口豁免，多位负载保持到下一请求。
set_false_path -to [get_pins {motor/perf/req_meta*/D motor/perf/ack_meta*/D}]
set_max_delay 20.000 -from [get_pins {motor/perf/net_hold*/Q}] -to [get_pins {motor/perf/net_snapshot*/D}]
set_false_path -hold -to [get_pins {motor/perf/net_snapshot*/D}]
# 新增监测器的网络复位异步置位、经同步链释放。
set_false_path -to [get_pins {motor/perf/net_reset/release_ff*/CLEAR}]

# 初始化跨时钟路径增加1ns最小延迟要求，促使布线修复保持时间；不豁免该路径。
# SUG940-2.0.4E第70-71页：set_min_delay用于hold/removal，单位ns。
set_min_delay 1.000 -from [get_pins {memory/controller/gw3_top/u_ddr_phy_top/ddr_init_internal_r_s0/Q}] -to [get_pins {memory/controller/gw3_top/u_ddr_phy_top/wloadn_3_s0/D}]
# 保留全局最差路径，并单独输出初始化路径，避免修复后落在默认25条之外。
report_timing -hold -max_paths 25
report_timing -hold -from [get_regs {memory/controller/gw3_top/u_ddr_phy_top/ddr_init_internal_r_s0}] -to [get_regs {memory/controller/gw3_top/u_ddr_phy_top/wloadn_3_s0}] -max_paths 1
