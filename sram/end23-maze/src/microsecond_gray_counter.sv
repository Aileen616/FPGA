// 功能：125MHz时钟域的64位微秒计数及同步Gray时间戳，每125拍同时更新一次。
// 输入：clk为125MHz时钟；rst_n为低有效异步复位，应由外部同步释放。
// 输出：microseconds单位us，64位无符号自然回绕；time_gray始终等于当前binary的Gray编码。
// 接口不包含初值或启动命令；复位全部为0，释放后第125个上升沿首次输出1及其Gray码。
// 用提前一拍注册的进位条件拆开64位加法；计数更新之间有125拍，进位标志必先稳定。
// 更新binary及Gray仍发生在原分频截止拍，不改变摄像头冻结帧时间戳或心跳的逐拍语义。
module microsecond_gray_counter(
    input wire clk,rst_n,
    output reg [63:0] microseconds,time_gray
);
    reg [6:0] us_div;                // 0..124，单位系统时钟拍。
    reg carry16,carry32,carry48;     // 各16位段的预计算进位，不能直接串联四段加法器。
    wire [63:0] incremented;
    assign incremented[15:0]=microseconds[15:0]+16'd1;
    assign incremented[31:16]=microseconds[31:16]+{15'b0,carry16};
    assign incremented[47:32]=microseconds[47:32]+{15'b0,carry32};
    assign incremented[63:48]=microseconds[63:48]+{15'b0,carry48};
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            us_div<=0;microseconds<=0;time_gray<=0;
            carry16<=0;carry32<=0;carry48<=0;
        end else begin
            // 这里仅归约生成寄存器输入；四段加法仅依赖已经注册的进位位。
            carry16<=&microseconds[15:0];
            carry32<=&microseconds[31:0];
            carry48<=&microseconds[47:0];
            if(us_div==7'd124) begin
                us_div<=0;
                microseconds<=incremented;
                time_gray<=incremented^(incremented>>1);
            end else us_div<=us_div+1'b1;
        end
    end
endmodule
