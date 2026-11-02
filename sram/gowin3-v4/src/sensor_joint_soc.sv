// 功能：JDY串口与四轮限幅试转的独立裸机子系统，不依赖网络/视频时钟。
// 参数：CLK_HZ为系统时钟Hz，默认片内OSC标称21MHz；UART9600，双重300ms超时。
// 输入：rst_n低有效同步释放复位；bt_rx及编码器AB在下层同步，须为兼容3.3V电平。
// 输出：按FL/FR/RL/RR排列的TB6612控制；默认PWM=0，无自动启动或Flash操作。
module sensor_joint_soc #(parameter integer CLK_HZ=21000000,UART_BAUD=9600)(
    inout wire tof_scl,tof_sda,inout wire [3:0] tof_xshut,
    input wire clk,rst_n,bt_rx,imu_miso,imu_int1,imu_int3,
    output wire imu_sck,imu_mosi,output wire [1:0] imu_cs_n,
    input wire [3:0] encoder_a,encoder_b,
    output wire bt_tx,
    output wire [3:0] motor_in1,motor_in2,motor_pwm,
    output wire motor_active,motor_fault
);
    wire cyc,stb,we,ack,comm_ack,motion_ack,comm_active,comm_fault;
    wire [31:0] addr,wdata,rdata,comm_data,motion_data;
    wire [3:0] sel;
    wire imu_select=addr[31:12]==20'h20004;
    wire imu_ack,imu_allowed;wire [31:0] imu_data;
    wire motion_select=addr[31:12]==20'h20003;
    wire tof_select=addr[31:12]==20'h20005;
    wire tof_ack;wire [31:0] tof_data;
    assign ack=tof_select?tof_ack:(imu_select?imu_ack:(motion_select?motion_ack:comm_ack));
    assign rdata=tof_select?tof_data:(imu_select?imu_data:(motion_select?motion_data:comm_data));
    Gowin_PicoRV32_Top cpu(.clk_in(clk),.resetn_in(rst_n),.wbuart_rx(1'b1),.wbuart_tx(),
        .slv_ext_cyc_o(cyc),.slv_ext_stb_o(stb),.slv_ext_we_o(we),.slv_ext_ack_i(ack),
        .slv_ext_adr_o(addr),.slv_ext_wdata_o(wdata),.slv_ext_rdata_i(rdata),
        .slv_ext_sel_o(sel),.irq_in(12'b0));
    esp_diag_wb #(.CLK_HZ(CLK_HZ),.UART_BAUD(UART_BAUD),.TIMEOUT_CYCLES(CLK_HZ*3/10)) communication(
        .clk(clk),.rst_n(rst_n),.wb_cyc(cyc&&!motion_select&&!imu_select&&!tof_select),.wb_stb(stb&&!motion_select&&!imu_select&&!tof_select),
        .wb_we(we),.wb_addr(addr),.wb_wdata(wdata),.wb_sel(sel),.wb_ack(comm_ack),.wb_rdata(comm_data),
        .esp_rx(bt_rx),.esp_tx(bt_tx),.active(comm_active),.fault(comm_fault),.uptime_ms());
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
endmodule
