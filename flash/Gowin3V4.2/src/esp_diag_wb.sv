// 功能：无电机的诊断外设和独立100ms租约保护，经典Wishbone每笔只执行一次。
// CLK_HZ为Hz且为1000整数倍，TIMEOUT_CYCLES默认100ms；保护不依赖CPU轮询。
// 0x20002000 ID，+04毫秒，+08状态，+0c命令，+10序号/续期，+14 RX，+18 TX，+1c串口错误。
// 命令：1清故障，2启动测试，4停止；状态bit0有效、bit1锁存。无任何动力端口。
module esp_diag_wb #(parameter integer CLK_HZ=50000000,TIMEOUT_CYCLES=CLK_HZ/10,UART_BAUD=115200)(
    input wire clk,rst_n,wb_cyc,wb_stb,wb_we,input wire [31:0] wb_addr,wb_wdata,
    input wire [3:0] wb_sel,output reg wb_ack,output reg [31:0] wb_rdata,
    input wire esp_rx,output wire esp_tx,output reg active,fault,
    output wire [31:0] uptime_ms
);
    reg held;
    reg [31:0] age,last_seq,ms,ms_div;
    wire valid,tx_busy,uart_error;
    reg serial_fault;
    // 仅停机显式开启的本地自主模式；默认仍保留原手动单错停机语义。
    reg local_confirm,previous_error;
    reg [7:0] error_count;
    reg [31:0] error_since;
    wire new_error=uart_error&&!previous_error;
    wire repeated_error=local_confirm&&new_error&&error_count>=2&&
        (ms-error_since)>=50&&(ms-error_since)<=200;
    reg uart_clear;
    wire [7:0] rx_data;
    wire request=wb_cyc && wb_stb;
    wire access=request && !held;
    wire legal=wb_addr[31:5]==27'h1000100 && wb_addr[1:0]==0 && (!wb_we || wb_sel==15);
    wire read_rx=access && legal && !wb_we && wb_addr[4:0]==5'h14;
    wire send_tx=access && legal && wb_we && wb_addr[4:0]==5'h18 && !tx_busy;
    wire expired=active && age>=TIMEOUT_CYCLES-1;
    assign uptime_ms=ms;
    esp_diag_uart #(.CLK_HZ(CLK_HZ),.BAUD(UART_BAUD)) uart(.clk(clk),.rst_n(rst_n),.rx(esp_rx),
        .clear(uart_clear),.pop(read_rx),.tx_start(send_tx),.tx_data(wb_wdata[7:0]),.tx(esp_tx),
        .tx_busy(tx_busy),.valid(valid),.data(rx_data),.error(uart_error));
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            held<=0;wb_ack<=0;wb_rdata<=0;age<=0;last_seq<=0;ms<=0;ms_div<=0;
            active<=0;fault<=1;serial_fault<=0;uart_clear<=0;
            local_confirm<=0;previous_error<=0;error_count<=0;error_since<=0;
        end else begin
            uart_clear<=0;previous_error<=uart_error;
            if(ms_div==CLK_HZ/1000-1) begin ms_div<=0;ms<=ms+1'b1;end
            else ms_div<=ms_div+1'b1;
            if(active && age<TIMEOUT_CYCLES) age<=age+1'b1;
            if(!request) begin held<=0;wb_ack<=0;end
            if(access) begin
                held<=1;wb_ack<=1;wb_rdata<=0;
                if(!legal) begin active<=0;fault<=1;wb_rdata<=32'hbad00001;end
                else if(wb_we) case(wb_addr[4:0])
                    5'h0c:case(wb_wdata)
                        1:if(!active && !serial_fault) begin fault<=0;last_seq<=0;end
                        2:if(!fault && !active && !serial_fault) begin active<=1;age<=0;last_seq<=0;end
                        4:begin active<=0;fault<=1;end
                        // 只在已停车时清通信错误并丢弃旧字节；保持故障和禁能，不能启动。
                        8:if(!active) begin serial_fault<=0;uart_clear<=1;fault<=1;last_seq<=0;end
                        16:if(!active)begin local_confirm<=1;error_count<=0;end
                        32:if(!active)begin local_confirm<=0;error_count<=0;end
                        // 仅丢弃坏包，不清动力故障、不改序号和租约年龄。
                        64:if(local_confirm||!active)begin serial_fault<=0;uart_clear<=1;end
                        default:begin active<=0;fault<=1;end
                    endcase
                    5'h10:if(active && !fault && wb_wdata>last_seq) begin
                        last_seq<=wb_wdata;age<=0;
                    end
                    5'h18:if(tx_busy) begin serial_fault<=1;active<=0;fault<=1;end
                    default:begin active<=0;fault<=1;end
                endcase
                else case(wb_addr[4:0])
                    0:wb_rdata<=32'h45535001;
                    4:wb_rdata<=ms;
                    8:wb_rdata<={29'b0,local_confirm,fault,active};
                    5'h10:wb_rdata<=last_seq;
                    5'h14:wb_rdata<=valid?{24'b0,rx_data}:32'h100;
                    5'h18:wb_rdata<={23'b0,tx_busy,8'b0};
                    5'h1c:wb_rdata<={31'b0,serial_fault};
                    default:wb_rdata<=0;
                endcase
            end
            // 超时与UART错误优先于同拍续期/清故障/启动，不允许边界复活。
            if(expired || (uart_error&&!local_confirm) || repeated_error) begin active<=0;fault<=1;end
            // 错误锁存必须清除后才能观察下一个独立错误，禁止逐周期重复累计同一标志。
            if(local_confirm&&uart_error)uart_clear<=1;
            if(local_confirm&&new_error)begin
                if(!error_count||(ms-error_since)>200)begin error_since<=ms;error_count<=1;end
                else if(error_count!=255)error_count<=error_count+1'b1;
            end
            if(uart_error) serial_fault<=1;
        end
    end
endmodule
