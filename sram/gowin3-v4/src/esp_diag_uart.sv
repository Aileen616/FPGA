// 功能：115200/8N1诊断UART；CLK_HZ为实际时钟Hz，DEPTH固定64字节。
// RX两级同步，采样起始位/停止位；溢出和帧错误输出单拍，交给硬件保护锁存。
// pop须在valid时置1；tx_start只在tx_busy=0时使用。复位TX为空闲高电平。
module esp_diag_uart #(parameter integer CLK_HZ=50000000, BAUD=115200)(
    input wire clk,rst_n,rx,pop,tx_start,clear,input wire [7:0] tx_data,
    output reg tx,output wire tx_busy,valid,output wire [7:0] data,
    output reg error
);
    localparam integer DIV=(CLK_HZ+BAUD/2)/BAUD;
    (* ASYNC_REG="TRUE" *) reg rx_meta,rx_sync;
    reg [1:0] rs;
    reg [31:0] rc,tc;
    reg [2:0] bit_index;
    reg [7:0] shift;
    reg [9:0] tx_shift;
    reg [3:0] tx_left;
    reg [7:0] fifo[0:63];
    reg [5:0] head,tail;
    reg [6:0] count;
    wire take=pop && count!=0;
    wire received=rs==3 && rc==0 && rx_sync;
    wire put=received && (count<64 || take);
    assign valid=count!=0;
    assign data=fifo[tail];
    assign tx_busy=tx_left!=0;
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin rx_meta<=1;rx_sync<=1;end
        else begin rx_meta<=rx;rx_sync<=rx_meta;end
    end
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            rs<=0;rc<=0;bit_index<=0;shift<=0;head<=0;tail<=0;count<=0;error<=0;
            tx<=1;tx_left<=0;tx_shift<=10'h3ff;tc<=0;
        end else if(clear) begin
            // 同步清队列和收发状态，恢复后仍须新命令，不重放故障前数据。
            rs<=0;rc<=0;bit_index<=0;shift<=0;head<=0;tail<=0;count<=0;error<=0;
            tx<=1;tx_left<=0;tx_shift<=10'h3ff;tc<=0;
        end else begin
            error<=0;
            if(take) tail<=tail+1'b1;
            if(put) begin fifo[head]<=shift;head<=head+1'b1;end
            case({put,take})
                2'b10:count<=count+1'b1;
                2'b01:count<=count-1'b1;
                default:begin end
            endcase
            if(received && !put) error<=1;
            case(rs)
                0:if(!rx_sync) begin rs<=1;rc<=DIV/2-1;end
                1:if(rc!=0) rc<=rc-1'b1;
                  else if(rx_sync) rs<=0;
                  else begin rs<=2;rc<=DIV-1;bit_index<=0;end
                2:if(rc!=0) rc<=rc-1'b1;
                  else begin
                      shift[bit_index]<=rx_sync;rc<=DIV-1;
                      if(bit_index==7) rs<=3;else bit_index<=bit_index+1'b1;
                  end
                3:if(rc!=0) rc<=rc-1'b1;
                  else begin rs<=0;if(!rx_sync) error<=1;end
            endcase
            if(tx_left==0) begin
                tx<=1;
                if(tx_start) begin tx_shift<={1'b1,tx_data,1'b0};tx<=0;tx_left<=10;tc<=DIV-1;end
            end else if(tc!=0) tc<=tc-1'b1;
            else begin
                tc<=DIV-1;tx_left<=tx_left-1'b1;
                tx_shift<={1'b1,tx_shift[9:1]};tx<=tx_shift[1];
            end
        end
    end
`ifndef SYNTHESIS
    initial if(DIV<4) $fatal(1,"UART clock too low");
`endif
endmodule
