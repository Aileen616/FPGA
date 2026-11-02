// 功能：定窗口测速，输出有符号 counts/s；固定整数倍率走寄存器乘法，其他倍率使用逐拍除法。
// 参数：CLK_HZ、WINDOW_CYCLES 为正数且不超过 2^31-1；窗口至少 70 拍。
// 输入：position 为模2^32计数，单窗口净位移绝对值必须小于2^31；ready 属于 clk 域。
// 输出：valid 为一次结果脉冲；quantized 表示仅0/1计数；overflow 表示结果饱和。
// 默认50MHz、500000拍，即10ms；流水延迟不改变计数窗口时长，ready撤销会丢弃在途结果。
module speed_sample #(parameter integer CLK_HZ=50000000, WINDOW_CYCLES=500000)(
    input wire clk,rst_n,ready,input wire signed [31:0] position,
    output reg signed [31:0] cps,output reg valid,quantized,overflow
);
    localparam integer CW=$clog2(WINDOW_CYCLES);
    localparam integer INTEGER_SCALE=CLK_HZ/WINDOW_CYCLES;
    reg [CW-1:0] elapsed;
    reg primed,launch;
    reg signed [31:0] previous,displacement;
    reg [1:0] stage;
    reg signed [63:0] scaled;
    reg result_quantized;
    wire signed [63:0] final_value;
    wire result_valid;
    generate if (CLK_HZ % WINDOW_CYCLES == 0) begin: exact_scale
        assign final_value=scaled;
        assign result_valid=stage==2;
    end else begin: fractional_scale
        reg [63:0] quotient;
        reg [32:0] remainder;
        reg [6:0] remaining;
        reg negative,done;
        wire [32:0] shifted={remainder[31:0],quotient[63]};
        wire subtract=shifted >= WINDOW_CYCLES;
        wire [32:0] next_remainder=subtract ? shifted-WINDOW_CYCLES : shifted;
        assign final_value=negative ? -$signed(quotient) : $signed(quotient);
        assign result_valid=done;
        always @(posedge clk or negedge rst_n) begin
            if (!rst_n) begin quotient<=0;remainder<=0;remaining<=0;negative<=0;done<=0;end
            else if (!ready) begin remaining<=0;done<=0;end
            else begin
                done<=0;
                if(stage==2) begin
                    quotient<=scaled[63] ? -scaled : scaled;
                    negative<=scaled[63];remainder<=0;remaining<=64;
                end else if(remaining!=0) begin
                    quotient<={quotient[62:0],subtract};remainder<=next_remainder;
                    remaining<=remaining-1'b1;
                    if(remaining==1) done<=1;
                end
            end
        end
    end endgenerate
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            elapsed<=0;primed<=0;previous<=0;displacement<=0;launch<=0;stage<=0;
            scaled<=0;cps<=0;valid<=0;quantized<=1;overflow<=0;result_quantized<=1;
        end else begin
            valid<=0;launch<=0;
            if(!ready) begin
                elapsed<=0;primed<=0;stage<=0;cps<=0;overflow<=0;quantized<=1;
            end else begin
                if(!primed) begin previous<=position;elapsed<=0;primed<=1;end
                else if(elapsed==WINDOW_CYCLES-1) begin
                    previous<=position;displacement<=position-previous;elapsed<=0;launch<=1;
                end else elapsed<=elapsed+1'b1;
                // 计数采样、缩放和结果饱和分拍执行，避免组合除法成为系统长路径。
                if(launch) begin
                    scaled<=$signed(displacement)*$signed((CLK_HZ%WINDOW_CYCLES==0)?INTEGER_SCALE:CLK_HZ);
                    result_quantized<=displacement>=-1 && displacement<=1;stage<=2;
                end else stage<=0;
                if(result_valid) begin
                    valid<=1;quantized<=result_quantized;
                    overflow<=final_value>64'sd2147483647 || final_value< -64'sd2147483648;
                    if(final_value>64'sd2147483647) cps<=32'sh7fffffff;
                    else if(final_value< -64'sd2147483648) cps<=32'sh80000000;
                    else cps<=final_value[31:0];
                end
            end
        end
    end
endmodule
