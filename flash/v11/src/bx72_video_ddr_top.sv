// 功能：36EC控制与性能统计加DDR网络/显示帧缓存，独立离线候选。
`include "stereo_network_config.vh"
module bx72_video_ddr_top(
    input wire Clk,input wire Rst_n,bt_rx,uart_rx,input wire [3:0] encoder_a,encoder_b,
    output wire bt_tx,output wire [3:0] motor_in1,motor_in2,motor_pwm,
    input wire c0_pclk,c0_vsync,c0_href,input wire [7:0] c0_data,
    inout wire c0_reset_n,c0_pwdn,c0_scl,c0_sda,
    input wire c1_pclk,c1_vsync,c1_href,input wire [7:0] c1_data,
    inout wire c1_reset_n,c1_pwdn,c1_scl,c1_sda,
    input wire rgmii_rx_clk_i,
    output wire rgmii_tx_clk,rgmii_txen,output wire [3:0] rgmii_txd,
    output wire eth_rstn,eth_mdc,inout wire eth_mdio,
    output wire [1:0] Led,output wire uart_tx,
    output wire hdmi_clk_p,hdmi_clk_n,output wire [2:0] hdmi_dat_p,hdmi_dat_n,output wire hdmi_oe,
    output wire [14:0] O_ddr_addr,output wire [2:0] O_ddr_ba,
    output wire O_ddr_cs_n,O_ddr_ras_n,O_ddr_cas_n,O_ddr_we_n,O_ddr_clk,O_ddr_clk_n,
    output wire O_ddr_cke,O_ddr_odt,O_ddr_reset_n,output wire [3:0] O_ddr_dqm,
    inout wire [31:0] IO_ddr_dq,inout wire [3:0] IO_ddr_dqs,IO_ddr_dqs_n);
    wire sys_clk;
    wire [9:0] hdmi_status;
    OSC #(.FREQ_DIV(10)) internal_osc(.OSCOUT(sys_clk));
    reg [15:0] startup=0;
    always @(posedge sys_clk) if(!(&startup)) startup<=startup+1'b1;
    wire sys_rst_n,net_rst_n,cam0_rst_n,cam1_rst_n;
    wire [1:0] configured,config_error;
    reset_sync sys_reset(sys_clk,&startup,sys_rst_n);
    wire net_clk=rgmii_rx_clk_i;
    wire ddr_calibrated,ddr_reset_active,display_lock,mem_clk,mem_rst_n;
    wire ready=sys_rst_n && (&configured) && ddr_calibrated && !ddr_reset_active && display_lock;
    reset_sync memory_reset(mem_clk,ready,mem_rst_n);
    wire [7:0] ddr_req_valid,ddr_req_write,ddr_req_ready,ddr_rsp_valid;
    wire [231:0] ddr_req_addr;wire [2047:0] ddr_req_data;wire [255:0] ddr_rsp_data;
    wire [1:0] network_overflow,display_overflow,display_underflow;
    reset_sync net_reset(net_clk,ready,net_rst_n);
    reset_sync cam0_reset(c0_pclk,ready,cam0_rst_n);
    reset_sync cam1_reset(c1_pclk,ready,cam1_rst_n);
    stereo_hdmi_config config_camera(.clk(sys_clk),.rst_n(sys_rst_n),
        .scl({c1_scl,c0_scl}),.sda({c1_sda,c0_sda}),
        .reset_n({c1_reset_n,c0_reset_n}),.pwdn({c1_pwdn,c0_pwdn}),
        .done(configured),.error(config_error),.index0(),.index1());
    // 复用厂家常高复位策略；管理数据线释放，防止与PHY读响应冲突。
    assign eth_rstn=1'b1;
    assign eth_mdc=1'b0;
    assign eth_mdio=1'bz;
    // 预计算分段进位，保持每125拍的二进制/Gray时间戳，缩短125MHz组合路径。
    wire [63:0] microseconds,time_gray;
    microsecond_gray_counter timebase(.clk(net_clk),.rst_n(net_rst_n),
        .microseconds(microseconds),.time_gray(time_gray));
    wire request;wire [1:0] complete,valid;
    wire [63:0] timestamp0,timestamp1;
    wire [16:0] rd_addr;wire [15:0] pixel0,pixel1;
    wire [1:0] pixel_ready;
    assign ddr_req_write[5:4]=0;assign ddr_req_data[1535:1024]=0;
    ddr_snapshot #(.BASE(29'd0)) buffer0(
        .pclk(c0_pclk),.cam_rst_n(cam0_rst_n),.vsync(c0_vsync),.href(c0_href),.data(c0_data),.time_gray(time_gray),
        .net_clk(net_clk),.net_rst_n(net_rst_n),.request(request),.rd_addr(rd_addr),.pixel(pixel0),.pixel_ready(pixel_ready[0]),
        .complete(complete[0]),.valid(valid[0]),.timestamp_us(timestamp0),.overflow(network_overflow[0]),
        .mem_clk(mem_clk),.mem_rst_n(mem_rst_n),.wr_valid(ddr_req_valid[0]),.wr_write(ddr_req_write[0]),
        .wr_ready(ddr_req_ready[0]),.wr_addr(ddr_req_addr[0+:29]),.wr_data(ddr_req_data[0+:256]),.wr_response(ddr_rsp_valid[0]),
        .rd_valid(ddr_req_valid[4]),.rd_ready(ddr_req_ready[4]),.rd_address(ddr_req_addr[116+:29]),.rd_response(ddr_rsp_valid[4]),.rd_data(ddr_rsp_data));
    ddr_snapshot #(.BASE(29'd65536)) buffer1(
        .pclk(c1_pclk),.cam_rst_n(cam1_rst_n),.vsync(c1_vsync),.href(c1_href),.data(c1_data),.time_gray(time_gray),
        .net_clk(net_clk),.net_rst_n(net_rst_n),.request(request),.rd_addr(rd_addr),.pixel(pixel1),.pixel_ready(pixel_ready[1]),
        .complete(complete[1]),.valid(valid[1]),.timestamp_us(timestamp1),.overflow(network_overflow[1]),
        .mem_clk(mem_clk),.mem_rst_n(mem_rst_n),.wr_valid(ddr_req_valid[1]),.wr_write(ddr_req_write[1]),
        .wr_ready(ddr_req_ready[1]),.wr_addr(ddr_req_addr[29+:29]),.wr_data(ddr_req_data[256+:256]),.wr_response(ddr_rsp_valid[1]),
        .rd_valid(ddr_req_valid[5]),.rd_ready(ddr_req_ready[5]),.rd_address(ddr_req_addr[145+:29]),.rd_response(ddr_rsp_valid[5]),.rd_data(ddr_rsp_data));
    wire tx_start,tx_done,tx_req;wire [15:0] tx_length;wire [7:0] tx_data;
    wire [31:0] sent_pairs,dropped_pairs,packet_count,pair_id;wire [4:0] send_state;
    ddr_packet_sender #(.STREAM_ID(`BXV_STREAM_ID)) sender(
        .clk(net_clk),.rst_n(net_rst_n),.enabled(1'b1),.request(request),.complete(complete),.valid(valid),
        .timestamp0(timestamp0),.timestamp1(timestamp1),.rd_addr(rd_addr),.pixel0(pixel0),.pixel1(pixel1),.pixel_ready(pixel_ready),
        .tx_start(tx_start),.tx_done(tx_done),.tx_req(tx_req),.tx_data(tx_data),.tx_length(tx_length),
        .sent_pairs(sent_pairs),.dropped_pairs(dropped_pairs),.packet_count(packet_count),.pair_id(pair_id),.state(send_state));
    wire gmii_clk,gmii_en;wire [7:0] gmii_data;
    eth_udp_tx_gmii udp(.clk125m(net_clk),.reset_p(!net_rst_n),.tx_en_pulse(tx_start),.tx_done(tx_done),
        .dst_mac(`BXV_DST_MAC),.src_mac(48'h020000725001),.dst_ip(`BXV_DST_IP),.src_ip(`BXV_SRC_IP),
        .dst_port(16'd5000),.src_port(16'd5000),.data_length(tx_length),.payload_req_o(tx_req),.payload_dat_i(tx_data),
        .gmii_tx_clk(gmii_clk),.gmii_txen(gmii_en),.gmii_txd(gmii_data));
    gmii_to_rgmii phy_output(.reset_n(net_rst_n),.gmii_tx_clk(gmii_clk),.gmii_txd(gmii_data),.gmii_txen(gmii_en),
        .gmii_txer(1'b0),.rgmii_tx_clk(rgmii_tx_clk),.rgmii_txd(rgmii_txd),.rgmii_txen(rgmii_txen));
    // UART仅报告经过注册的Gray计数与单比特活动状态，不跨域直采二进制计数。
    reg [31:0] sent_gray,drop_gray,packet_gray;
    reg net_heartbeat;
    always @(posedge net_clk or negedge net_rst_n) begin
        if(!net_rst_n) begin sent_gray<=0;drop_gray<=0;packet_gray<=0;net_heartbeat<=0;end
        else begin
            sent_gray<=sent_pairs^(sent_pairs>>1);drop_gray<=dropped_pairs^(dropped_pairs>>1);
            packet_gray<=packet_count^(packet_count>>1);net_heartbeat<=microseconds[19];
        end
    end
    (* ASYNC_REG="TRUE" *) reg [95:0] stats_meta,stats_sync;
    (* ASYNC_REG="TRUE" *) reg heartbeat_meta,heartbeat_sync;
    reg [31:0] sys_cycles;
    always @(posedge sys_clk or negedge sys_rst_n) begin
        if(!sys_rst_n) begin stats_meta<=0;stats_sync<=0;heartbeat_meta<=0;heartbeat_sync<=0;sys_cycles<=0;end
        else begin
            stats_meta<={sent_gray,drop_gray,packet_gray};stats_sync<=stats_meta;
            heartbeat_meta<=net_heartbeat;heartbeat_sync<=heartbeat_meta;sys_cycles<=sys_cycles+1'b1;
        end
    end
    // 功能：还原只读遥测计数；参数32位Gray计数；返回二进制计数。
    function [31:0] decode_gray(input [31:0] value);
        integer j;begin for(j=0;j<32;j=j+1) decode_gray[j]=^(value>>j);end
    endfunction
    (* ASYNC_REG="TRUE" *) reg [6:0] memory_health_meta,memory_health_sync;
    always @(posedge sys_clk or negedge sys_rst_n)
        if(!sys_rst_n)begin memory_health_meta<=0;memory_health_sync<=0;end
        else begin
            memory_health_meta<={network_overflow,display_overflow,display_underflow,ddr_calibrated};
            memory_health_sync<=memory_health_meta;
        end
    // 原保留bit15作为DDR错误锁存摘要；其余UART格式与80A2相同。
    wire [255:0] telemetry={8'he3,8'd1,|memory_health_sync[6:1],hdmi_status,heartbeat_sync,config_error,configured,
        sys_cycles,32'(`BXV_STREAM_ID),decode_gray(stats_sync[95:64]),decode_gray(stats_sync[63:32]),
        decode_gray(stats_sync[31:0]),32'(`BXV_SRC_IP),32'(`BXV_DST_IP)};
    // USB输出独占控制协议，历史UART周期遥测不连接到同一引脚。
    assign bt_tx=1'b1;
    assign Led={|config_error || |memory_health_sync[6:1],memory_health_sync[0] && heartbeat_sync};
    // HDMI三缓冲与网络冻结帧分配独立DDR区域，避免互相覆盖。
    stereo_ddr_display display(.sys_clk(sys_clk),.sys_rst_n(sys_rst_n),.reference_clk(net_clk),.configured(configured),
        .ready(ready),.display_lock(display_lock),.overflow(display_overflow),.underflow(display_underflow),
        .mem_clk(mem_clk),.mem_rst_n(mem_rst_n),
        .req_valid({ddr_req_valid[7:6],ddr_req_valid[3:2]}),.req_write({ddr_req_write[7:6],ddr_req_write[3:2]}),
        .req_ready({ddr_req_ready[7:6],ddr_req_ready[3:2]}),.rsp_valid({ddr_rsp_valid[7:6],ddr_rsp_valid[3:2]}),
        .req_addr({ddr_req_addr[203+:29],ddr_req_addr[174+:29],ddr_req_addr[87+:29],ddr_req_addr[58+:29]}),
        .req_data({ddr_req_data[2047:1536],ddr_req_data[1023:512]}),.rsp_data(ddr_rsp_data),
        .c0_pclk(c0_pclk),.c0_vsync(c0_vsync),.c0_href(c0_href),.c0_data(c0_data),
        .c1_pclk(c1_pclk),.c1_vsync(c1_vsync),.c1_href(c1_href),.c1_data(c1_data),
        .hdmi_clk_p(hdmi_clk_p),.hdmi_clk_n(hdmi_clk_n),.hdmi_dat_p(hdmi_dat_p),.hdmi_dat_n(hdmi_dat_n),
        .hdmi_oe(hdmi_oe),.status(hdmi_status));
    bx72_ddr_native memory(.board_clk(Clk),.init_rst_n(sys_rst_n && display_lock && (&configured)),.user_rst_n(mem_rst_n),
        .user_clk(mem_clk),.calibrated(ddr_calibrated),.reset_active(ddr_reset_active),.req_valid(ddr_req_valid),.req_write(ddr_req_write),
        .req_ready(ddr_req_ready),.req_addr(ddr_req_addr),.req_data(ddr_req_data),.rsp_valid(ddr_rsp_valid),.rsp_data(ddr_rsp_data),
        .O_ddr_addr(O_ddr_addr),.O_ddr_ba(O_ddr_ba),.O_ddr_cs_n(O_ddr_cs_n),.O_ddr_ras_n(O_ddr_ras_n),
        .O_ddr_cas_n(O_ddr_cas_n),.O_ddr_we_n(O_ddr_we_n),.O_ddr_clk(O_ddr_clk),.O_ddr_clk_n(O_ddr_clk_n),
        .O_ddr_cke(O_ddr_cke),.O_ddr_odt(O_ddr_odt),.O_ddr_reset_n(O_ddr_reset_n),.O_ddr_dqm(O_ddr_dqm),
        .IO_ddr_dq(IO_ddr_dq),.IO_ddr_dqs(IO_ddr_dqs),.IO_ddr_dqs_n(IO_ddr_dqs_n));
    // 电机与视频共享片内21MHz时钟，控制不依赖以太网链路，复位键只撤销运动。
    wire motor_rst_n,motor_active,motor_fault;
    reset_sync motor_reset(sys_clk,sys_rst_n&&Rst_n,motor_rst_n);
    motor_forward_soc motor(.clk(sys_clk),.rst_n(motor_rst_n),.bt_rx(uart_rx),.bt_tx(uart_tx),
        .perf_net_clk(net_clk),.perf_net_rst_n(net_rst_n&&motor_rst_n),.perf_request(request),
        .perf_complete(complete),.perf_valid(valid),.perf_state(send_state),
        .perf_sent(sent_pairs),.perf_dropped(dropped_pairs),.perf_packets(packet_count),
        .encoder_a(encoder_a),.encoder_b(encoder_b),.motor_in1(motor_in1),
        .motor_in2(motor_in2),.motor_pwm(motor_pwm),.motor_active(motor_active),.motor_fault(motor_fault));
endmodule
