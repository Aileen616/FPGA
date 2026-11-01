if {[catch {
create_project -name competition_mono -dir {F:/FPGA/build/soc/task4_maze_corner_fix/gowin_route2} -pn GW5AT-LV138PG484AC1/I0 -device_version C -force
set_option -verilog_std sysv2017
set_option -top_module competition_mono_top
set_option -use_cpu_as_gpio 1
set_option -use_sspi_as_gpio 1
set_option -place_option 1
set_option -route_option 1
set_option -correct_hold_violation 1
set_option -num_critical_paths 100
set_option -include_path {F:/FPGA/build/soc/task4_maze_corner_fix/src}
add_file {F:/FPGA/build/soc/task4_maze_corner_fix/src/picorv32.v}
add_file {F:/FPGA/build/soc/task4_maze_corner_fix/src/picosoc.v}
add_file {F:/FPGA/build/soc/task4_maze_corner_fix/src/picosoc_dtcm.v}
add_file {F:/FPGA/build/soc/task4_maze_corner_fix/src/picosoc_itcm.v}
add_file {F:/FPGA/build/soc/task4_maze_corner_fix/src/wb_brancher.v}
add_file {F:/FPGA/build/soc/task4_maze_corner_fix/src/wb_bus.v}
add_file {F:/FPGA/build/soc/task4_maze_corner_fix/src/wbuart.v}
add_file {F:/FPGA/build/soc/task4_maze_corner_fix/src/dvi_tx.v}
add_file {F:/FPGA/build/soc/task4_maze_corner_fix/src/pll_init.v}
add_file {F:/FPGA/build/soc/task4_maze_corner_fix/src/i2c_master_bit_ctrl.v}
add_file {F:/FPGA/build/soc/task4_maze_corner_fix/src/i2c_master_byte_ctrl.v}
add_file {F:/FPGA/build/soc/task4_maze_corner_fix/src/hdmi_startup.sv}
add_file {F:/FPGA/build/soc/task4_maze_corner_fix/src/reset_sync.sv}
add_file {F:/FPGA/build/soc/task4_maze_corner_fix/src/esp_diag_uart.sv}
add_file {F:/FPGA/build/soc/task4_maze_corner_fix/src/esp_diag_wb.sv}
add_file {F:/FPGA/build/soc/task4_maze_corner_fix/src/encoder_ab.sv}
add_file {F:/FPGA/build/soc/task4_maze_corner_fix/src/motor_forward_wb.sv}
add_file {F:/FPGA/build/soc/task4_maze_corner_fix/src/pwm_open_loop.sv}
add_file {F:/FPGA/build/soc/task4_maze_corner_fix/src/tb6612_adapter.sv}
add_file {F:/FPGA/build/soc/task4_maze_corner_fix/src/runtime_perf_wb.sv}
add_file {F:/FPGA/build/soc/task4_maze_corner_fix/src/speed_sample.sv}
add_file {F:/FPGA/build/soc/task4_maze_corner_fix/src/pi_speed_unipolar.sv}
add_file {F:/FPGA/build/soc/task4_maze_corner_fix/src/motor_speed_loop.sv}
add_file {F:/FPGA/build/soc/task4_maze_corner_fix/src/competition_soc.sv}
add_file {F:/FPGA/build/soc/task4_maze_corner_fix/src/competition_mono_top.sv}
add_file {F:/FPGA/build/soc/task4_maze_corner_fix/src/mono_scanout.sv}
add_file {F:/FPGA/build/soc/task4_maze_corner_fix/src/tof4_wb.sv}
add_file {F:/FPGA/build/soc/task4_maze_corner_fix/src/bmi088_spi_master.sv}
add_file {F:/FPGA/build/soc/task4_maze_corner_fix/src/motor_imu_wb.sv}
add_file {F:/FPGA/build/soc/task4_maze_corner_fix/src/ov5640_config.sv}
add_file {F:/FPGA/build/soc/task4_maze_corner_fix/src/imu_text_overlay.sv}
add_file {F:/FPGA/build/soc/task4_maze_corner_fix/src/mipi_osd_wb.sv}
add_file {F:/FPGA/build/soc/task4_maze_corner_fix/src/p3_hdmi_pll_mod.v}
add_file {F:/FPGA/build/soc/task4_maze_corner_fix/src/camera_pll.v}
add_file {F:/FPGA/build/soc/task4_maze_corner_fix/src/camera_pll_mod.v}
add_file {F:/FPGA/build/soc/task4_maze_corner_fix/src/i2c_control.v}
add_file {F:/FPGA/build/soc/task4_maze_corner_fix/src/i2c_bit_shift.v}
add_file {F:/FPGA/build/soc/task4_maze_corner_fix/src/ov5640_init_table_rgb.v}
add_file {F:/FPGA/build/soc/task4_maze_corner_fix/src/competition_mono.cst}
add_file {F:/FPGA/build/soc/task4_maze_corner_fix/src/competition_mono.sdc}
add_file {F:/FPGA/build/soc/task4_maze_corner_fix/src/dvp_hdmi_double_buffer.sv}
add_file {F:/FPGA/build/soc/task4_maze_corner_fix/src/hdmi_bram_scanout.sv}
add_file {F:/FPGA/build/soc/task4_maze_corner_fix/src/tof_register_fsm.sv}
add_file {F:/FPGA/build/soc/task4_maze_corner_fix/src/ps2_poll_wb.sv}
add_file {F:/FPGA/build/soc/task4_maze_corner_fix/src/fpga_blue_vision_wb.sv}
add_file {F:/FPGA/build/soc/task4_maze_corner_fix/src/endpoint_image_wb.sv}
run syn
run pnr
} message]} {puts stderr $message;exit 1}
exit 0

