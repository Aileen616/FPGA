// 功能：八拍单向速度PI，输出0..limit；目标方向由外层选择，本模块绝不输出反向驱动力。
// 输入：tick间隔至少9拍；target_cps为0..200000，measured_cps为沿目标方向的有符号counts/s。
// kp/ki为0..7FFFFFFF的非负Q16.16；Ki已经包含10ms周期；limit为1..16384，必须包含起步斜坡。
// 输出：effort为无符号幅值；integral_q16为Q16积分；valid仅在结果完成时为1。
// 上下限与条件积分使用同一limit，超速时下限为0；禁能取消在途运算并清积分和输出。
module pi_speed_unipolar(
    input wire clk,rst_n,enable,tick,
    input wire signed [31:0] target_cps,measured_cps,
    input wire [31:0] kp_q16,ki_tick_q16,
    input wire [15:0] effort_limit,
    output reg [15:0] effort,
    output reg signed [63:0] integral_q16,
    output reg valid
);
    reg [2:0] stage;
    reg signed [32:0] error_cps;
    reg [30:0] kp,ki;
    reg signed [63:0] p_term,i_step,clamped_i,chosen_i;
    reg signed [64:0] candidate,trial_total,total,limit_q16;
    reg [15:0] limit_effort;
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            stage<=0;error_cps<=0;kp<=0;ki<=0;p_term<=0;i_step<=0;clamped_i<=0;chosen_i<=0;
            candidate<=0;trial_total<=0;total<=0;limit_q16<=0;limit_effort<=0;
            effort<=0;integral_q16<=0;valid<=0;
        end else begin
            valid<=0;
            if(!enable || effort_limit==0 || effort_limit>16384) begin
                stage<=0;effort<=0;integral_q16<=0;
            end else case(stage)
                0:if(tick) begin
                    error_cps<={target_cps[31],target_cps}-{measured_cps[31],measured_cps};
                    kp<=kp_q16[30:0];ki<=ki_tick_q16[30:0];stage<=1;
                    limit_q16<=$signed({33'b0,effort_limit,16'b0});limit_effort<=effort_limit;
                end
                1:begin
                    p_term<=$signed(error_cps)*$signed({1'b0,kp});
                    i_step<=$signed(error_cps)*$signed({1'b0,ki});stage<=2;
                end
                2:begin candidate<={integral_q16[63],integral_q16}+{i_step[63],i_step};stage<=3;end
                3:begin
                    if(candidate>limit_q16) clamped_i<=limit_q16[63:0];
                    else if(candidate<0) clamped_i<=0;
                    else clamped_i<=candidate[63:0];
                    stage<=4;
                end
                4:begin trial_total<={p_term[63],p_term}+{clamped_i[63],clamped_i};stage<=5;end
                5:begin
                    // 正误差不能继续推高已饱和输出，负误差不能在零输出时积累反向驱动力。
                    if((trial_total>limit_q16 && error_cps>0)||(trial_total<0 && error_cps<0))
                        chosen_i<=integral_q16;
                    else chosen_i<=clamped_i;
                    stage<=6;
                end
                6:begin total<={p_term[63],p_term}+{chosen_i[63],chosen_i};stage<=7;end
                7:begin
                    integral_q16<=chosen_i;valid<=1;stage<=0;
                    if(total>limit_q16) effort<=limit_effort;
                    else if(total<0) effort<=0;
                    else effort<=total[31:16];
                end
            endcase
        end
    end
endmodule
