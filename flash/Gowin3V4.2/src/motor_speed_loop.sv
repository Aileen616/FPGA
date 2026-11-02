// 功能：单轮10ms编码器测速、单向PI、起步限额和编码器保护；参数与位置单位均明确保留。
// CLK_HZ为系统Hz且10ms窗口至少70拍；STEP为每10ms增加的驱动力，默认256。
// allowed由外层租约/模式/掩码/100ms换向空窗共同许可；target_cps为-200000..200000。
// position为原始32位模计数；encoder_invert仅改变反馈正方向；delta为每拍合法AB增量。
// kp/ki为非负Q16.16，limit为1..16384；输出有符号effort但物理方向只取target符号。
// 保护reason[0]：有驱动却连续500ms无净正向进展，不能区分堵转与断线；[1]连续3窗口反向。
// reason[2]：活动期间出现任意非法AB跳变；[3]测速溢出。fault为一拍请求，由外层全轮锁存。
module motor_speed_loop #(
    parameter integer CLK_HZ=21000000,
    parameter integer STEP=256
)(
    input wire clk,rst_n,allowed,encoder_ready,encoder_invert,
    input wire signed [31:0] position,target_cps,
    input wire signed [1:0] delta,
    input wire [31:0] illegal_count,kp_q16,ki_tick_q16,
    input wire [15:0] effort_limit,
    output wire signed [31:0] speed_cps,
    output wire signed [15:0] effort,
    output reg fault,
    output reg [3:0] reason
);
    localparam integer WINDOW=CLK_HZ/100;
    wire signed [31:0] raw_speed;
    wire sample_valid,sample_overflow;
    reg previous_sign;
    wire enabled=allowed && target_cps!=0 && previous_sign==target_cps[31];
    wire inverted_overflow=encoder_invert && raw_speed==32'sh80000000;
    assign speed_cps=encoder_invert ? (inverted_overflow ? 32'sh7fffffff : -raw_speed) : raw_speed;
    wire signed [31:0] target_magnitude=target_cps[31] ? -target_cps : target_cps;
    wire direction_overflow=target_cps[31] && speed_cps==32'sh80000000;
    wire signed [31:0] directed_speed=target_cps[31] ?
        (direction_overflow ? 32'sh7fffffff : -speed_cps) : speed_cps;
    reg [15:0] dynamic_limit;
    wire [16:0] stepped_limit={1'b0,dynamic_limit}+STEP;
    wire [15:0] next_limit=stepped_limit>effort_limit ? effort_limit : stepped_limit[15:0];
    wire [15:0] pi_effort;
    wire pi_valid;
    wire signed [63:0] integral_q16;
    assign effort=!enabled ? 16'sd0 : (target_cps[31] ? -$signed(pi_effort) : $signed(pi_effort));
    speed_sample #(.CLK_HZ(CLK_HZ),.WINDOW_CYCLES(WINDOW)) sampler(
        .clk(clk),.rst_n(rst_n),.ready(encoder_ready),.position(position),
        .cps(raw_speed),.valid(sample_valid),.quantized(),.overflow(sample_overflow));
    pi_speed_unipolar controller(.clk(clk),.rst_n(rst_n),.enable(enabled),.tick(sample_valid),
        .target_cps(target_magnitude),.measured_cps(directed_speed),.kp_q16(kp_q16),.ki_tick_q16(ki_tick_q16),
        .effort_limit(next_limit),.effort(pi_effort),.integral_q16(integral_q16),.valid(pi_valid));
    reg [5:0] empty_windows;
    reg [1:0] wrong_windows;
    reg [31:0] previous_illegal;
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            previous_sign<=0;dynamic_limit<=0;fault<=0;reason<=0;
            empty_windows<=0;wrong_windows<=0;previous_illegal<=0;
        end else begin
            previous_sign<=target_cps[31];fault<=0;reason<=0;previous_illegal<=illegal_count;
            if(!enabled) begin
                dynamic_limit<=0;empty_windows<=0;wrong_windows<=0;
            end else begin
                if(!encoder_ready) begin fault<=1;reason[0]<=1;end
                if(illegal_count!=previous_illegal) begin fault<=1;reason[2]<=1;end
                if(sample_valid) begin
                    dynamic_limit<=next_limit;
                    if(sample_overflow || inverted_overflow || direction_overflow) begin fault<=1;reason[3]<=1;end
                    // 只有实际形成驱动力后才累计缺边沿时间，起步和超速零输出不作断线判断。
                    // 同一窗口往返抖动净位移为0，不能通过任意边沿刷新无进展计时。
                    if(pi_effort<256 || directed_speed>0) empty_windows<=0;
                    else if(empty_windows>=49) begin fault<=1;reason[0]<=1;end
                    else empty_windows<=empty_windows+1'b1;
                    if(pi_effort<256 || directed_speed>=0) wrong_windows<=0;
                    else if(wrong_windows>=2) begin fault<=1;reason[1]<=1;end
                    else wrong_windows<=wrong_windows+1'b1;
                end
            end
        end
    end
endmodule
