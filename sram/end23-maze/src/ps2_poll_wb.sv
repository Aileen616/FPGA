// 功能：PS2九字节只读轮询及原子寄存器快照；不直接产生电机命令。
// 参数：CLK_HZ为系统时钟；HALF_CYCLES为16us半周期；POLL_CYCLES为30ms间隔。
// 输入：DAT异步串行数据、只读Wishbone；输出：空闲高电平CMD/CS/CLK及快照。
// 寄存器：0标识，4完成序号，8有效标志，12按键原始低有效，16摇杆RX/RY/LX/LY。
module ps2_poll_wb #(
    parameter integer CLK_HZ=21000000,
    parameter integer HALF_CYCLES=CLK_HZ/62500,
    parameter integer POLL_CYCLES=CLK_HZ*3/100
)(input wire clk,rst_n,ps2_dat,input wire cyc,stb,
  input wire [31:0] addr,output wire ack,output reg [31:0] rdata,
  output reg ps2_cmd,ps2_cs_n,ps2_clk);
    (* ASYNC_REG="TRUE" *) reg [1:0] dat_sync;
    reg [31:0] timer,packet_seq;
    reg [71:0] packet;
    reg [15:0] buttons;
    reg [31:0] axes;
    reg valid;
    reg [6:0] bit_index;
    reg [2:0] state;
    localparam IDLE=0,SETUP=1,LOW=2,HIGH=3,FINISH=4;
    wire [71:0] command_bits=72'h000000000000004201;
    assign ack=cyc&&stb;
    always @* begin
        case(addr[7:2])
        0:rdata=32'h50533231;
        1:rdata=packet_seq;
        2:rdata={31'b0,valid};
        3:rdata={16'b0,buttons};
        4:rdata=axes;
        default:rdata=0;
        endcase
    end
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n)begin
            dat_sync<=3;timer<=0;packet_seq<=0;packet<=0;buttons<=16'hffff;
            axes<=32'h80808080;valid<=0;bit_index<=0;state<=IDLE;
            ps2_cmd<=1;ps2_cs_n<=1;ps2_clk<=1;
        end else begin
            dat_sync<={dat_sync[0],ps2_dat};
            if(timer!=0)timer<=timer-1;
            else case(state)
            IDLE:begin
                ps2_cs_n<=0;ps2_clk<=1;ps2_cmd<=1;packet<=0;bit_index<=0;
                timer<=HALF_CYCLES-1;state<=SETUP;
            end
            SETUP:begin
                ps2_clk<=0;ps2_cmd<=command_bits[bit_index];
                timer<=HALF_CYCLES-1;state<=LOW;
            end
            LOW:begin
                // 在上升沿前取样：接收器自下降沿起已有完整半周期建立时间。
                packet[bit_index]<=dat_sync[1];ps2_clk<=1;
                timer<=HALF_CYCLES-1;state<=HIGH;
            end
            HIGH:begin
                if(bit_index==71)begin state<=FINISH;end
                else begin bit_index<=bit_index+1;state<=SETUP;end
            end
            FINISH:begin
                ps2_cs_n<=1;ps2_cmd<=1;ps2_clk<=1;packet_seq<=packet_seq+1;
                valid<=packet[23:8]==16'h5a73;
                buttons<=packet[39:24];axes<=packet[71:40];
                timer<=POLL_CYCLES-1;state<=IDLE;
            end
            default:begin state<=IDLE;valid<=0;ps2_cs_n<=1;end
            endcase
        end
    end
endmodule
