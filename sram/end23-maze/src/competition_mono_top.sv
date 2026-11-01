// 功能：恢复相机片内双缓冲横向画面，恢复原CPU/IMU/OSD，DDR及网口停用，电机使用原控制器及停机保护。
// 参数：无；Clk为板载50MHz，Rst_n低有效；电机默认停机，USB及蓝牙控制沿用原协议和租约。
module competition_mono_top(input wire Clk,Rst_n,bt_rx,uart_rx,
    input wire ps2_dat,output wire ps2_cmd,ps2_cs_n,ps2_clk,
 input wire [3:0] encoder_a,encoder_b,
 inout wire tof_scl,tof_sda,inout wire [3:0] tof_xshut,
 input wire imu_miso,imu_int1,imu_int3,output wire imu_sck,imu_mosi,imu_cs_acc_n,imu_cs_gyro_n,
 input wire camera_pclk,camera_vsync,camera_href,input wire [7:0] camera_data,
 inout wire camera_sclk,camera_sdat,
 output wire [3:0] motor_in1,motor_in2,motor_pwm,
 output wire eth_rstn,camera_xclk,O_ddr_reset_n,O_ddr_cke,uart_tx,bt_tx,
 output wire hdmi_clk_p,hdmi_clk_n,output wire [2:0] hdmi_dat_p,hdmi_dat_n,output wire hdmi_oe);
 wire ref24;
 // 保持PHY与DDR复位，沿用649B已稳定的相机时钟和显示通路。
 assign eth_rstn=1'b0;assign camera_xclk=ref24;assign O_ddr_reset_n=1'b0;assign O_ddr_cke=1'b0;
 assign hdmi_oe=1'b1;
 reg [15:0] startup=0;always @(posedge Clk)if(!(&startup))startup<=startup+1'b1;
 wire sys_rst_n,cam_clock_locked,hdmi_reference_rst_n;
 reset_sync sys_reset(Clk,(&startup)&&Rst_n,sys_rst_n);
 // 保持稳定0D8D的24MHz内部参考，同时恢复原相机外部时钟。
 camera_pll cam_clock(.clkin(Clk),.init_clk(Clk),.reset(!sys_rst_n),.clkout0(ref24),.lock(cam_clock_locked));
 reset_sync hdmi_reference_reset(Clk,sys_rst_n&&cam_clock_locked,hdmi_reference_rst_n);
 wire serial_clk,pixel_clk,raw_lock,pll_reset,clock_enable,dvi_reset_n,video_locked;
 wire [5:0] icpsel;wire [2:0] lpfres;
 p3_hdmi_pll_MOD hdmi_clock(.clkin(ref24),.clkout0(serial_clk),.clkout1(pixel_clk),.lock(raw_lock),
 .reset(pll_reset),.clock_enable(clock_enable),.icpsel(icpsel),.lpfres(lpfres),.lpfcap(2'b00));
 PLL_INIT #(.CLK_PERIOD(20),.MULTI_FAC(42)) hdmi_init(.CLKIN(Clk),.I_RST(!hdmi_reference_rst_n),
 .O_RST(pll_reset),.PLLLOCK(raw_lock),.O_LOCK(video_locked),.ICPSEL(icpsel),.LPFRES(lpfres));
 hdmi_startup video_startup(Clk,hdmi_reference_rst_n,video_locked,dvi_reset_n,clock_enable);
 // 相机配置/双缓冲/旋转扫描均沿用3E66，显示扫描不依赖相机配置完成。
 wire config_rst_n,init_done,init_fault,capture_rst_n,display_rst_n,timing_rst_n;
 wire [15:0] chip_id,local_frames,local_bad_frames;wire [7:0] config_index;wire [3:0] config_fault_code;
 reset_sync config_reset(Clk,sys_rst_n&&cam_clock_locked,config_rst_n);
 ov5640_config camera_config(.clk(Clk),.rst_n(config_rst_n),.done(init_done),.fault(init_fault),
 .chip_id(chip_id),.index(config_index),.scl(camera_sclk),.sda(camera_sdat),.fault_code(config_fault_code));
 wire buffer_reset_n=sys_rst_n&&init_done&&video_locked;
 reset_sync capture_reset(camera_pclk,buffer_reset_n,capture_rst_n);
 reset_sync display_reset(pixel_clk,buffer_reset_n,display_rst_n);
 reset_sync timing_reset(pixel_clk,sys_rst_n&&video_locked,timing_rst_n);
 (* ASYNC_REG="TRUE" *) reg [1:0] done_sync;
 always @(posedge pixel_clk or negedge timing_rst_n)
 if(!timing_rst_n)done_sync<=0;else done_sync<={done_sync[0],init_done};
 wire hs,vs,de,vblank,image_valid,published;wire [7:0] red,green,blue;
 wire [10:0] scan_x;wire [9:0] scan_y;
 hdmi_bram_scanout local_display(.pclk(camera_pclk),.cam_rst_n(capture_rst_n),
 .vsync(camera_vsync),.href(camera_href),.data(camera_data),.pixel_clk(pixel_clk),
 .rd_rst_n(display_rst_n),.timing_rst_n(timing_rst_n),.done(done_sync[1]),
 .hs(hs),.vs(vs),.de(de),.red(red),.green(green),.blue(blue),.scan_x(scan_x),.scan_y(scan_y),
 .vblank(vblank),.valid(image_valid),.published(published),.frames(local_frames),.bad_frames(local_bad_frames));
 // 恢复原21MHz裸机子系统和字符叠加；将原控制器受保护的电机输出恢复到物理端口。
 wire control_clk,motor_rst_n;
 OSC #(.FREQ_DIV(10)) internal_osc(.OSCOUT(control_clk));
 reset_sync motor_reset(control_clk,sys_rst_n&&Rst_n,motor_rst_n);
 wire text_we,text_bank,publish,displayed;wire [6:0] text_addr;wire [31:0] text_data;
 wire ohs,ovs,ode,osd_rst_n;wire [7:0] ored,ogreen,oblue;
 reset_sync osd_reset(pixel_clk,motor_rst_n&&video_locked,osd_rst_n);
 imu_text_overlay overlay(.sys_clk(control_clk),.sys_rst_n(motor_rst_n),.text_we(text_we),.text_bank(text_bank),
 .text_addr(text_addr),.text_data(text_data),.publish(publish),.displayed(displayed),
 .pixel_clk(pixel_clk),.pixel_rst_n(osd_rst_n),.vblank(vblank),.x(scan_x),.y(scan_y),
 .hs_in(hs),.vs_in(vs),.de_in(de),.red_in(red),.green_in(green),.blue_in(blue),
 .hs(ohs),.vs(ovs),.de(ode),.red(ored),.green(ogreen),.blue(oblue));
 // 状态位保留原布局；DDR就绪位明确为0，不伪造网络或DDR健康状态。
 reg local_bad_seen;
 always @(posedge camera_pclk or negedge capture_rst_n)
 if(!capture_rst_n)local_bad_seen<=0;else if(local_bad_frames!=0)local_bad_seen<=1;
 (* ASYNC_REG="TRUE" *) reg [7:0] health_meta,health_sync;
 always @(posedge control_clk or negedge motor_rst_n)
 if(!motor_rst_n)begin health_meta<=0;health_sync<=0;end
 else begin health_meta<={3'b0,local_bad_seen,2'b0,init_fault,init_done};health_sync<=health_meta;end
 wire [3:0] aligned_encoder_a={encoder_a[3],encoder_a[0],encoder_a[1],encoder_a[2]};
 wire [3:0] aligned_encoder_b={encoder_b[3],encoder_b[0],encoder_b[1],encoder_b[2]};
 wire vision_cam_rst_n;
 reset_sync vision_camera_reset(camera_pclk,motor_rst_n&&init_done,vision_cam_rst_n);
 competition_soc motor(.vision_pclk(camera_pclk),.vision_cam_rst_n(vision_cam_rst_n),
 .vision_ready(init_done),.vision_vsync(camera_vsync),.vision_href(camera_href),.vision_data(camera_data),.ps2_dat(ps2_dat),.ps2_cmd(ps2_cmd),.ps2_cs_n(ps2_cs_n),.ps2_clk(ps2_clk),.clk(control_clk),.rst_n(motor_rst_n),.bt_rx(bt_rx),.bt_tx(bt_tx),.usb_rx(uart_rx),.usb_tx(uart_tx),
 .tof_scl(tof_scl),.tof_sda(tof_sda),.tof_xshut(tof_xshut),.imu_miso(imu_miso),.imu_int1(imu_int1),.imu_int3(imu_int3),
 .imu_sck(imu_sck),.imu_mosi(imu_mosi),.imu_cs_n({imu_cs_gyro_n,imu_cs_acc_n}),
 .encoder_a(aligned_encoder_a),.encoder_b(aligned_encoder_b),.motor_in1(motor_in1),.motor_in2(motor_in2),.motor_pwm(motor_pwm),
 .motor_active(),.motor_fault(),.text_we(text_we),.text_bank(text_bank),.text_addr(text_addr),.text_data(text_data),
 .publish(publish),.displayed(displayed),.camera_status({24'b0,health_sync}),.display_status({24'b0,health_sync}),
 .video_diag(512'b0),.perf_net_clk(Clk),.perf_net_rst_n(1'b0),.perf_request(1'b0),.perf_complete(2'b0),.perf_valid(2'b0),
 .perf_state(5'b0),.perf_sent(32'b0),.perf_dropped(32'b0),.perf_packets(32'b0));
 DVI_TX_Top dvi(.I_rst_n(dvi_reset_n),.I_serial_clk(serial_clk),.I_rgb_clk(pixel_clk),
 .I_rgb_hs(ohs),.I_rgb_vs(ovs),.I_rgb_de(ode),.I_rgb_r(ored),.I_rgb_g(ogreen),.I_rgb_b(oblue),
 .O_tmds_clk_p(hdmi_clk_p),.O_tmds_clk_n(hdmi_clk_n),.O_tmds_data_p(hdmi_dat_p),.O_tmds_data_n(hdmi_dat_n));
endmodule
