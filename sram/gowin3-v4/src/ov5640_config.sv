// 功能：复用芯路恒 SCCB 引擎/原始 ROM，先软复位/待机，再校验ID并配置800×480 RGB565。
// 参数：SYS_HZ 单位 Hz；三个等待参数单位时钟周期，仿真可缩短。
// 接口：rst_n 低有效；done 表示 ID 正确且全部写入 ACK；fault/fault_code 锁存至复位。
// fault_code：0无故障，1读ID无应答，2错误ID，3读超时，4写无应答，5写超时，15非法状态。
// SCCB 只驱动低电平；板级必须有经资料确认的上拉。
module ov5640_config #(
    parameter integer SYS_HZ=50_000_000,
    parameter integer BOOT_CYCLES=1_100_000,
    parameter integer RESET_CYCLES=1_100_000,
    parameter integer SETTLE_CYCLES=15_000_000
)(input wire clk, rst_n, output reg done, fault,
  output reg [15:0] chip_id, output reg [7:0] index,
  inout wire scl, sda, output reg [3:0] fault_code);
    wire [23:0] lut;
    ov5640_init_table_rgb #(.IMAGE_WIDTH(16'd800),.IMAGE_HEIGHT(16'd480),
        .IMAGE_FLIP_EN(1'b0),.IMAGE_MIRROR_EN(1'b0)) rom(.clk(clk),.addr(index),.q(lut));
    reg wr, rd;
    wire rw_done, nack, scl_out;
    wire [7:0] rd_data;
    reg [15:0] addr;
    reg [7:0] wr_data;
    reg [31:0] delay_cycles;
    i2c_control #(.SYS_CLOCK(SYS_HZ),.SCL_CLOCK(100_000)) bus_engine(
        .Clk(clk),.Rst_n(rst_n),.wrreg_req(wr),.rdreg_req(rd),
        .addr(addr),.addr_mode(1'b1),.wrdata(wr_data),.rddata(rd_data),
        .device_id(8'h78),.RW_Done(rw_done),.ack(nack),
        .dly_cnt_max(delay_cycles),.i2c_sclk(scl_out),.i2c_sdat(sda));
    assign scl = scl_out ? 1'bz : 1'b0;
    localparam BOOT=0, ID_REQ=1, ID_WAIT=2, LUT_WAIT=3, WRITE_REQ=4,
               WRITE_WAIT=5, SETTLE=6, FINISH=7, FAILED=8;
    reg [3:0] state;
    reg id_low;
    reg before_id;
    reg [31:0] timer;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state<=BOOT; timer<=0; wr<=0; rd<=0; done<=0; fault<=0;
            chip_id<=0; index<=0; id_low<=0; before_id<=1; addr<=0; wr_data<=0; delay_cycles<=0; fault_code<=0;
        end else begin
            wr<=0; rd<=0;
            case(state)
                BOOT: if(timer>=BOOT_CYCLES-1) begin timer<=0; state<=LUT_WAIT; end
                      else timer<=timer+1'b1;
                ID_REQ: begin addr<=id_low ? 16'h300b : 16'h300a; rd<=1;
                              delay_cycles<=0; timer<=0; state<=ID_WAIT; end
                ID_WAIT: begin
                    timer<=timer+1'b1;
                    if(rw_done) begin
                        if(nack || rd_data!=(id_low ? 8'h40 : 8'h56)) begin
                            fault_code<=nack ? 4'd1 : 4'd2;
                            if(id_low) chip_id[7:0]<=rd_data; else chip_id[15:8]<=rd_data;
                            state<=FAILED;
                        end else if(!id_low) begin chip_id[15:8]<=rd_data; id_low<=1; state<=ID_REQ; end
                        else begin chip_id[7:0]<=rd_data; state<=LUT_WAIT; end
                    end else if(timer>=SYS_HZ/10) begin fault_code<=3; state<=FAILED; end
                end
                // 同步 ROM 在索引更新后留一拍，避免写入上一个寄存器值。
                LUT_WAIT: state<=WRITE_REQ;
                WRITE_REQ: begin addr<=lut[23:8]; wr_data<=lut[7:0]; wr<=1;
                    delay_cycles<=(index==1) ? RESET_CYCLES : 0;
                    timer<=0; state<=WRITE_WAIT; end
                WRITE_WAIT: begin
                    timer<=timer+1'b1;
                    if(rw_done) begin
                        if(nack) begin fault_code<=4; state<=FAILED; end
                        // 原表0..2为pad时钟、软复位、待机；复位后的等待按Linux驱动20~25ms取22ms。
                        // 未连接独立RESETB控制时先完成这三项，再读ID；其余配置必须通过ID门控。
                        else if(before_id && index==2) begin before_id<=0; index<=3; state<=ID_REQ; end
                        else if(index==251) begin timer<=0; state<=SETTLE; end
                        else begin index<=index+1'b1; state<=LUT_WAIT; end
                    end else if(timer>=SYS_HZ/10) begin fault_code<=5; state<=FAILED; end
                end
                SETTLE: if(timer>=SETTLE_CYCLES-1) begin done<=1; state<=FINISH; end
                        else timer<=timer+1'b1;
                FAILED: begin fault<=1; done<=0; end
                FINISH: done<=1;
                default: begin fault_code<=15; state<=FAILED; end
            endcase
        end
    end
endmodule
