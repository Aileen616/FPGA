// 功能：JDY串口与四轮限幅试转的独立裸机子系统，不依赖网络/视频时钟。
// 参数：CLK_HZ为系统时钟Hz，默认片内OSC标称21MHz；UART9600，双重300ms超时。
// 输入：rst_n低有效同步释放复位；bt_rx及编码器AB在下层同步，须为兼容3.3V电平。
// 输出：按FL/FR/RL/RR排列的TB6612控制；默认PWM=0，无自动启动或Flash操作。
module competition_soc #(parameter integer CLK_HZ=21000000,UART_BAUD=9600)(
    input wire vision_pclk,vision_cam_rst_n,vision_vsync,vision_href,vision_ready,
    input wire [7:0] vision_data,
    input wire ps2_dat,output wire ps2_cmd,ps2_cs_n,ps2_clk,
    inout wire tof_scl,tof_sda,inout wire [3:0] tof_xshut,
    input wire clk,rst_n,bt_rx,usb_rx,imu_miso,imu_int1,imu_int3,
    output wire usb_tx,text_we,text_bank,publish,output wire [6:0] text_addr,output wire [31:0] text_data,
    input wire displayed,input wire [31:0] camera_status,display_status,input wire [511:0] video_diag,
    input wire perf_net_clk,perf_net_rst_n,perf_request,input wire [1:0] perf_complete,perf_valid,
    input wire [4:0] perf_state,input wire [31:0] perf_sent,perf_dropped,perf_packets,
    output wire imu_sck,imu_mosi,output wire [1:0] imu_cs_n,
    input wire [3:0] encoder_a,encoder_b,
    output wire bt_tx,
    output wire [3:0] motor_in1,motor_in2,motor_pwm,
    output wire motor_active,motor_fault
);
    wire cyc,stb,we,ack,comm_ack,motion_ack,comm_active,comm_fault;
    wire [31:0] addr,wdata,rdata,comm_data,motion_data;
    wire [3:0] sel;
    wire vision_select=addr[31:12]==20'h2000a;
    wire vision_ack,vision_rst_n;wire [31:0] vision_rdata;
    reset_sync vision_reset(clk,rst_n&&vision_ready,vision_rst_n);
    fpga_blue_vision_wb #(.CLK_HZ(CLK_HZ)) vision(.clk(clk),.rst_n(vision_rst_n),
        .pclk(vision_pclk),.cam_rst_n(vision_cam_rst_n),.vsync(vision_vsync),.href(vision_href),.data(vision_data),
        .cyc(cyc&&vision_select),.stb(stb&&vision_select),.we(we),.addr(addr),.wdata(wdata),.sel(sel),
        .ack(vision_ack),.rdata(vision_rdata));
    wire imu_select=addr[31:12]==20'h20004;
    wire imu_ack,imu_allowed;wire [31:0] imu_data;
    wire motion_select=addr[31:12]==20'h20003;
    wire tof_select=addr[31:12]==20'h20005;
    wire tof_ack;wire [31:0] tof_data;
    wire osd_select=addr[31:12]==20'h20007,perf_select=addr[31:12]==20'h20006,aux_select=addr[31:12]==20'h20008;
    wire osd_ack,perf_ack,aux_ack;wire [31:0] osd_data,perf_data,aux_data;
    wire ps2_select=addr[31:12]==20'h20009;
    wire ps2_ack;wire [31:0] ps2_data;
    ps2_poll_wb #(.CLK_HZ(CLK_HZ)) ps2(.clk(clk),.rst_n(rst_n),.ps2_dat(ps2_dat),
        .cyc(cyc&&ps2_select),.stb(stb&&ps2_select),.addr(addr),.ack(ps2_ack),.rdata(ps2_data),
        .ps2_cmd(ps2_cmd),.ps2_cs_n(ps2_cs_n),.ps2_clk(ps2_clk));
    assign ack=vision_select?vision_ack:ps2_select?ps2_ack:aux_select?aux_ack:osd_select?osd_ack:perf_select?perf_ack:tof_select?tof_ack:(imu_select?imu_ack:(motion_select?motion_ack:comm_ack));
    assign rdata=vision_select?vision_rdata:ps2_select?ps2_data:aux_select?aux_data:osd_select?osd_data:perf_select?perf_data:tof_select?tof_data:(imu_select?imu_data:(motion_select?motion_data:comm_data));
    Gowin_PicoRV32_Top cpu(.clk_in(clk),.resetn_in(rst_n),.wbuart_rx(1'b1),.wbuart_tx(),
        .slv_ext_cyc_o(cyc),.slv_ext_stb_o(stb),.slv_ext_we_o(we),.slv_ext_ack_i(ack),
        .slv_ext_adr_o(addr),.slv_ext_wdata_o(wdata),.slv_ext_rdata_i(rdata),
        .slv_ext_sel_o(sel),.irq_in(12'b0));
    esp_diag_wb #(.CLK_HZ(CLK_HZ),.UART_BAUD(UART_BAUD),.TIMEOUT_CYCLES(CLK_HZ*3/10)) communication(
        .clk(clk),.rst_n(rst_n),.wb_cyc(cyc&&!motion_select&&!imu_select&&!tof_select&&!osd_select&&!perf_select&&!aux_select&&!ps2_select&&!vision_select),.wb_stb(stb&&!motion_select&&!imu_select&&!tof_select&&!osd_select&&!perf_select&&!aux_select&&!ps2_select&&!vision_select),
        .wb_we(we),.wb_addr(addr),.wb_wdata(wdata),.wb_sel(sel),.wb_ack(comm_ack),.wb_rdata(comm_data),
        .esp_rx(usb_rx),.esp_tx(usb_tx),.active(comm_active),.fault(comm_fault),.uptime_ms());
    motor_forward_wb #(.CLK_HZ(CLK_HZ),.EFFORT_MAX(19660)) motion(
        .clk(clk),.rst_n(rst_n),.wb_cyc(cyc&&motion_select),.wb_stb(stb&&motion_select),
        .wb_we(we),.wb_addr(addr),.wb_wdata(wdata),.wb_sel(sel),.wb_ack(motion_ack),.wb_rdata(motion_data),
        .safety_allowed(comm_active&&!comm_fault&&imu_allowed),.encoder_a(encoder_a),.encoder_b(encoder_b),
        .motor_in1(motor_in1),.motor_in2(motor_in2),.motor_pwm(motor_pwm),
        .motor_active(motor_active),.motor_fault(motor_fault));
    motor_imu_wb #(.CLK_HZ(CLK_HZ)) imu(
        .clk(clk),.rst_n(rst_n),.wb_cyc(cyc&&imu_select),.wb_stb(stb&&imu_select),
        .wb_we(we),.wb_addr(addr),.wb_wdata(wdata),.wb_sel(sel),.wb_ack(imu_ack),.wb_rdata(imu_data),
        .miso(imu_miso),.int1(imu_int1),.int3(imu_int3),.motor_active(motor_active),.sck(imu_sck),.mosi(imu_mosi),.cs_n(imu_cs_n),.allowed(imu_allowed));
    tof4_wb #(.CLK_HZ(CLK_HZ)) tof(
        .clk(clk),.rst_n(rst_n),.wb_cyc(cyc&&tof_select),.wb_stb(stb&&tof_select),
        .wb_we(we),.wb_addr(addr),.wb_wdata(wdata),.wb_sel(sel),.wb_ack(tof_ack),.wb_rdata(tof_data),
        .scl(tof_scl),.sda(tof_sda),.xshut(tof_xshut));

    esp_diag_wb #(.CLK_HZ(CLK_HZ),.UART_BAUD(UART_BAUD)) auxiliary(
        .clk(clk),.rst_n(rst_n),.wb_cyc(cyc&&aux_select),.wb_stb(stb&&aux_select),.wb_we(we),
        .wb_addr({20'h20002,addr[11:0]}),.wb_wdata(wdata),.wb_sel(sel),.wb_ack(aux_ack),.wb_rdata(aux_data),
        .esp_rx(bt_rx),.esp_tx(bt_tx),.active(),.fault(),.uptime_ms());
    mipi_osd_wb osd(.clk(clk),.rst_n(rst_n),.cyc(cyc&&osd_select),.stb(stb&&osd_select),
        .we(we),.addr(addr),.wdata(wdata),.sel(sel),.ack(osd_ack),.rdata(osd_data),
        .text_we(text_we),.text_bank(text_bank),.text_addr(text_addr),.text_data(text_data),
        .publish(publish),.displayed(displayed),.camera_status(camera_status),.display_status(display_status),.video_diag(video_diag));
    runtime_perf_wb #(.CLK_HZ(CLK_HZ),.PASSIVE_LOOP(0)) perf(.clk(clk),.rst_n(rst_n),.wb_cyc(cyc&&perf_select),.wb_stb(stb&&perf_select),
        .wb_we(we),.wb_addr(addr),.wb_wdata(wdata),.wb_sel(sel),.wb_ack(perf_ack),.wb_rdata(perf_data),
        .observe_cyc(cyc&&!perf_select),.observe_stb(stb&&!perf_select),.observe_ack(ack),.observe_we(we),.observe_addr(addr),
        .net_clk(perf_net_clk),.net_rst_n(perf_net_rst_n),.request(perf_request),.complete(perf_complete),.valid(perf_valid),
        .send_state(perf_state),.sent_pairs(perf_sent),.dropped_pairs(perf_dropped),.packet_count(perf_packets));
endmodule
