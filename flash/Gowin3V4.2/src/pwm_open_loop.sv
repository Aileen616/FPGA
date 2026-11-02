// 功能：signed16驱动力生成逻辑PWM/方向/使能，不定义未知H桥的IN1/IN2制动真值表。
// 参数：PERIOD_CYCLES>=2，默认2500；BLANK_CYCLES>=1，默认100。50MHz时为20kHz/2us。
// 幅值：1..32766对应floor(幅值*周期/32768)高电平拍；32767特例为100%，0禁能。
// 使用常数乘法与右移，避免M1实现中的组合除法长路径；这些电气参数仍须驱动实测。
module pwm_open_loop #(parameter integer PERIOD_CYCLES=2500,BLANK_CYCLES=100)(
    input wire clk,rst_n,allowed,input wire signed [15:0] effort,
    output wire pwm_mag,output reg dir,output wire enable
);
    localparam integer PW=$clog2(PERIOD_CYCLES);
    localparam integer BW=$clog2(BLANK_CYCLES+1);
    reg [PW-1:0] phase;
    reg [BW-1:0] blank;
    reg [PW:0] duty;
    wire requested_dir=effort[15];
    wire [16:0] magnitude=effort[15] ? -$signed({effort[15],effort}) : {1'b0,effort};
    wire [15:0] limited=magnitude>32767 ? 16'd32767 : magnitude[15:0];
    wire [63:0] product=limited*64'(PERIOD_CYCLES);
    wire [63:0] requested_duty=limited==32767 ? PERIOD_CYCLES : product>>15;
    assign enable=rst_n && allowed && blank==0 && requested_dir==dir && limited!=0 && duty!=0;
    assign pwm_mag=enable && phase<duty;
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin phase<=0;blank<=BLANK_CYCLES;dir<=0;duty<=0;end
        // 零目标必须丢弃旧占空比；重启先等待空窗和新周期，防止输出上一条命令的脉冲。
        else if(!allowed || limited==0) begin phase<=0;blank<=BLANK_CYCLES;duty<=0;end
        else if(requested_dir!=dir && limited!=0) begin
            dir<=requested_dir;blank<=BLANK_CYCLES;phase<=0;duty<=0;
        end else if(blank!=0) begin blank<=blank-1'b1;phase<=0;duty<=0;end
        else if(phase==PERIOD_CYCLES-1) begin phase<=0;duty<=requested_duty[PW:0];end
        else phase<=phase+1'b1;
    end
endmodule
