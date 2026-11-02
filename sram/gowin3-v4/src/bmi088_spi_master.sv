// 功能：BMI088模式0、MSB优先、双片选SPI；一次事务1..16字节，全程保持CS。
// 参数：HALF_CYCLES为半周期系统拍数，50MHz下25对应1MHz，必须>=3。
// 接口：start仅空闲接受；length=1..16；tx/rx低字节先传；abort立即释放总线。
module bmi088_spi_master #(parameter integer HALF_CYCLES=25,RUNTIME_DIVIDER=0)(
    input wire clk,rst_n,start,abort,chip,input wire [4:0] length,
    input wire [127:0] tx,input wire miso,
    output reg sck,mosi,output reg [1:0] cs_n,
    output reg busy,done,output reg [127:0] rx,
    input wire [15:0] half_cycles,
    input wire [9:0] trace_index,
    output reg [7:0] trace_data,
    output wire [31:0] trace_status,
    output reg [31:0] trace_rx,trace_total
);
    reg [127:0] buffer_tx;
    reg [4:0] count,length_q;
    reg [2:0] bit_index;
    reg hold_end;
    integer divider;
    reg [15:0] period_q; // 每笔事务锁定半周期拍数，中途改变输入不能扭曲时钟。
    (* ASYNC_REG="TRUE" *) reg miso_meta,miso_sync;
    // 只旁路观察gyro单字节ID事务；首个非0F读数冻结，复位前不会覆盖或清除。
    // 每字节按位记录busy、CS_G、CS_A、SCK、MOSI、MISO原输入、两级同步值。
    // 原输入仅供诊断，不进入SPI接收和任何控制判据。
    reg [7:0] trace_mem[0:1023];
    reg trace_active,trace_frozen,trace_overflow;
    reg [10:0] trace_used;
    assign trace_status={trace_frozen,trace_overflow,19'b0,trace_used};
    always @(posedge clk) begin
        trace_data<=trace_mem[trace_index];
        if(trace_active&&trace_used<1024)
            trace_mem[trace_used[9:0]]<={busy,cs_n,sck,mosi,miso,miso_meta,miso_sync};
    end
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n)begin
            trace_active<=0;trace_frozen<=0;trace_overflow<=0;trace_used<=0;
            trace_rx<=0;trace_total<=0;
        end else if(!trace_frozen)begin
            if(start&&!busy&&chip&&length==2&&tx[7:0]==8'h80)begin
                trace_active<=1;trace_used<=0;trace_overflow<=0;
            end else if(trace_active)begin
                if(trace_used<1024)trace_used<=trace_used+1'b1;
                else trace_overflow<=1;
                if(abort)trace_active<=0;
                else if(done)begin
                    trace_active<=0;trace_total<=trace_total+1'b1;trace_rx<=rx[31:0];
                    if(rx[15:8]!=8'h0f)trace_frozen<=1;
                end
            end
        end
    end
    always @(posedge clk or negedge rst_n)
        if(!rst_n) begin miso_meta<=0;miso_sync<=0;end
        else begin miso_meta<=miso;miso_sync<=miso_meta;end
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            sck<=0;mosi<=0;cs_n<=3;busy<=0;done<=0;rx<=0;buffer_tx<=0;
            count<=0;length_q<=0;bit_index<=7;hold_end<=0;divider<=0;period_q<=HALF_CYCLES;
        end else begin
            done<=0;
            if(abort) begin sck<=0;mosi<=0;cs_n<=3;busy<=0;hold_end<=0;end
            else if(!busy) begin
                if(start && length>=1 && length<=16) begin
                    buffer_tx<=tx;rx<=0;length_q<=length;count<=0;bit_index<=7;
                    cs_n<=chip?2'b01:2'b10;mosi<=tx[7];sck<=0;
                    period_q<=RUNTIME_DIVIDER?half_cycles:HALF_CYCLES;
                    divider<=(RUNTIME_DIVIDER?half_cycles:HALF_CYCLES)-1;busy<=1;hold_end<=0;
                end
            end else if(divider!=0) divider<=divider-1;
            else begin
                divider<=period_q-1;
                if(hold_end) begin cs_n<=3;mosi<=0;busy<=0;done<=1;hold_end<=0;end
                else if(!sck) begin
                    sck<=1;rx[count*8+bit_index]<=miso_sync;
                end else begin
                    sck<=0;
                    if(bit_index==0) begin
                        if(count+1==length_q) hold_end<=1;
                        else begin count<=count+1'b1;bit_index<=7;mosi<=buffer_tx[(count+1)*8+7];end
                    end else begin bit_index<=bit_index-1'b1;mosi<=buffer_tx[count*8+bit_index-1];end
                end
            end
        end
    end
endmodule
