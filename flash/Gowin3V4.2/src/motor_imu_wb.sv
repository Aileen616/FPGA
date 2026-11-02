// 功能：五线BMI088 SPI、微秒时基、航向模式的30ms独立新样本租约。
// 接口：0x20004000页；00身份，04时钟Hz，08时间us，0C SPI状态，10命令，20..2C发送，30..3C接收。
// 60写0/1关闭/启用航向保护（仅电机停止时）；64写严格递增非零样本序号续租；68读故障。
// 航向样本序号不能替代蓝牙命令续租；超时故障须停机后显式关闭保护再启用。
// 本轮频率对照采用500kHz以缩短事务；200Hz传感器ODR和故障保护不变，稳定性须实测。
module motor_imu_wb #(parameter integer CLK_HZ=21000000,LEASE_US=30000,SPI_HZ=500000)(
    input wire clk,rst_n,wb_cyc,wb_stb,wb_we,
    input wire [31:0] wb_addr,wb_wdata,input wire [3:0] wb_sel,
    output reg wb_ack,output reg [31:0] wb_rdata,
    input wire miso,motor_active,int1,int3,output wire sck,mosi,output wire [1:0] cs_n,
    output wire allowed
);
    reg [31:0] now_us,stamp,sequence_q;
    integer us_div;
    // 单比特异步输入只进入两级同步器；后级边沿捕获全部位于本地时钟域。
    (* ASYNC_REG="TRUE" *) reg [1:0] irq_meta,irq_sync;
    reg [1:0] irq_previous,snap_levels;
    reg irq_enabled;
    reg [31:0] acc_edges,gyro_edges,acc_stamp,gyro_stamp;
    reg [31:0] snap_acc,snap_gyro,snap_acc_stamp,snap_gyro_stamp,snap_time;
    reg [127:0] tx;
    wire [127:0] rx;
    reg start,abort,chip,completed,command_error,required,expired,have_sample;
    reg [4:0] length;
    // 0x14半周期分频：仅停止且SPI空闲时修改，限制名义SCK在50kHz..2MHz。
    reg [15:0] spi_div;
    reg [9:0] trace_index; // 仅改变诊断存储读地址，不影响SPI或故障状态。
    wire [7:0] trace_data;
    wire [31:0] trace_status,trace_rx,trace_total;
    wire busy,done;
    wire stale=!have_sample || (now_us-stamp)>=LEASE_US;
    assign allowed=!required || (!expired&&!stale);
    bmi088_spi_master #(.HALF_CYCLES((CLK_HZ+2*SPI_HZ-1)/(2*SPI_HZ)),.RUNTIME_DIVIDER(1)) spi(
        .clk(clk),.rst_n(rst_n),.start(start),.abort(abort),.chip(chip),.length(length),
        .tx(tx),.rx(rx),.miso(miso),.sck(sck),.mosi(mosi),.cs_n(cs_n),.busy(busy),.done(done),.half_cycles(spi_div),.trace_index(trace_index),.trace_data(trace_data),
        .trace_status(trace_status),.trace_rx(trace_rx),.trace_total(trace_total));
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            trace_index<=0;now_us<=0;us_div<=0;stamp<=0;sequence_q<=0;tx<=0;start<=0;abort<=0;chip<=0;
            length<=1;completed<=0;command_error<=0;required<=0;expired<=0;have_sample<=0;wb_ack<=0;wb_rdata<=0;
            spi_div<=(CLK_HZ+2*SPI_HZ-1)/(2*SPI_HZ);
            irq_meta<=0;irq_sync<=0;irq_previous<=0;irq_enabled<=0;
            acc_edges<=0;gyro_edges<=0;acc_stamp<=0;gyro_stamp<=0;
            snap_acc<=0;snap_gyro<=0;snap_acc_stamp<=0;snap_gyro_stamp<=0;snap_time<=0;snap_levels<=0;
        end else begin
            irq_meta<={int3,int1};irq_sync<=irq_meta;irq_previous<=irq_sync;
            if(irq_enabled)begin
                if(irq_sync[0]&&!irq_previous[0])begin acc_edges<=acc_edges+1'b1;acc_stamp<=now_us;end
                if(irq_sync[1]&&!irq_previous[1])begin gyro_edges<=gyro_edges+1'b1;gyro_stamp<=now_us;end
            end
            if(us_div==CLK_HZ/1000000-1) begin us_div<=0;now_us<=now_us+1'b1;end
            else us_div<=us_div+1;
            start<=0;abort<=0;
            if(done)completed<=1;
            if(required&&stale)expired<=1;
            if(!wb_cyc||!wb_stb)wb_ack<=0;
            else if(!wb_ack)begin
                wb_ack<=1;wb_rdata<=0;
                if(wb_addr[31:8]==24'h200040&&wb_addr[1:0]==0&&(!wb_we||wb_sel==15))begin
                    if(wb_we)case(wb_addr[7:0])
                        // 1原子快照；0停止捕获；2清计数并启用，仅停止且航向关闭时可改变捕获状态。
                        8'h40:if(wb_wdata==1)begin
                            snap_acc<=acc_edges;snap_gyro<=gyro_edges;snap_acc_stamp<=acc_stamp;
                            snap_gyro_stamp<=gyro_stamp;snap_time<=now_us;snap_levels<=irq_sync;
                        end else if(!motor_active&&!required&&(wb_wdata==0||wb_wdata==2))begin
                            irq_enabled<=wb_wdata==2;acc_edges<=0;gyro_edges<=0;acc_stamp<=0;gyro_stamp<=0;
                        end
                        8'h10:if(wb_wdata==32'h100)begin abort<=1;completed<=0;command_error<=0;end
                            else if(wb_wdata[0])begin
                                if(busy||start)command_error<=1;
                                else begin chip<=wb_wdata[1];length<={1'b0,wb_wdata[7:4]}+5'd1;
                                    start<=1;completed<=0;command_error<=0;end
                            end
                        8'h20,8'h24,8'h28,8'h2c:if(!busy&&!start)tx[wb_addr[3:2]*32+:32]<=wb_wdata;
                        8'h14:if(!motor_active&&!required&&!busy&&!start&&
                                  wb_wdata>=((CLK_HZ+3999999)/4000000)&&wb_wdata>=3&&
                                  wb_wdata<=((CLK_HZ+99999)/100000))spi_div<=wb_wdata[15:0];
                              else command_error<=1;
                        8'h78:if(!motor_active&&wb_wdata<1024)trace_index<=wb_wdata[9:0];
                        8'h60:if(!motor_active)begin
                            if(wb_wdata==0)begin required<=0;expired<=0;end
                            else if(wb_wdata==1&&!stale&&!expired)required<=1;
                        end
                        8'h64:if(wb_wdata!=0&&wb_wdata>sequence_q)begin
                            sequence_q<=wb_wdata;stamp<=now_us;have_sample<=1;
                        end
                        default:command_error<=1;
                    endcase
                    else case(wb_addr[7:0])
                        8'h00:wb_rdata<=32'h424d4902;
                        8'h04:wb_rdata<=CLK_HZ;
                        8'h08:wb_rdata<=now_us;
                        8'h0c:wb_rdata<={29'b0,command_error,completed,(busy|start)};
                        8'h14:wb_rdata<={16'b0,spi_div};
                        8'h40:wb_rdata<={31'b0,irq_enabled};
                        8'h44:wb_rdata<={30'b0,snap_levels};
                        8'h48:wb_rdata<=snap_acc;
                        8'h4c:wb_rdata<=snap_gyro;
                        8'h50:wb_rdata<=snap_acc_stamp;
                        8'h54:wb_rdata<=snap_gyro_stamp;
                        8'h58:wb_rdata<=snap_time;
                        8'h30,8'h34,8'h38,8'h3c:wb_rdata<=rx[wb_addr[3:2]*32+:32];
                        8'h60:wb_rdata<={31'b0,required};
                        8'h64:wb_rdata<=sequence_q;
                        8'h70:wb_rdata<=trace_status;
                        8'h74:wb_rdata<=trace_rx;
                        8'h78:wb_rdata<={22'b0,trace_index};
                        8'h7c:wb_rdata<={24'b0,trace_data};
                        8'h80:wb_rdata<=trace_total;
                        8'h68:wb_rdata<={30'b0,stale,expired};
                        default:wb_rdata<=0;
                    endcase
                end
            end
        end
    end
endmodule
