# 仅50MHz板载输入，PLL输出由工具根据实际IP参数建立生成时钟。
create_clock -name board_clk -period 20.000 [get_ports {Clk}]
# 外部复位仅进入两拍同步器，异步断言、同步释放。
set_false_path -from [get_ports {Rst_n}]
set_false_path -to [get_pins {*reset/release_ff*/CLEAR}]
# 保留3E66停钟释放DVI复位的既有例外；不切断正常RGB或串行时钟域。
set_false_path -from [get_pins {video_startup/reset_n*/Q}] -to [get_pins {dvi/rgb2dvi_inst/u_OSER10*/RESET dvi/rgb2dvi_inst/TMDS8b10b_inst_*/*/CLEAR dvi/rgb2dvi_inst/TMDS8b10b_inst_*/*/PRESET}]

# 相机PCLK沿用现有96MHz预算；外部延迟尚未实测。
create_clock -name camera_pclk -period 10.416 [get_ports {camera_pclk}]
# 仅跨域握手及初始化标志的首级同步器；不切整个时钟域。
set_false_path -to [get_pins {local_display/buffer/req_meta*/D local_display/buffer/ack_meta*/D done_sync_0*/D}]

# 原裸机CPU与姿态模块的21MHz内部时钟。
create_clock -name osc_clk -period 47.619 [get_pins {internal_osc/OSCOUT}]
# UART/编码器/IMU/ToF在模块内同步；保留原外部异步输入约束。
set_false_path -from [get_ports {bt_rx uart_rx encoder_a[*] encoder_b[*] imu_miso imu_int1 imu_int3 tof_scl tof_sda}]
# 只豁免字符翻页和健康状态的首级同步器，后级仍正常计时。
set_false_path -to [get_pins {overlay/req_meta*/D motor/osd/ack_meta*/D health_meta*/D}]

# 视觉专用握手CDC：多位载荷在握手确认前保持。
set_false_path -to [get_pins {motor/vision/time_meta*/D motor/vision/ack_meta*/D motor/vision/req_meta*/D}]
set_max_delay -from [get_pins {motor/vision/payload*/Q}] -to [get_pins {motor/vision/latest*/D motor/vision/source_stamp*/D}] 80
