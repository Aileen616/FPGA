// 功能：八拍流水定点速度PI，保留条件积分、积分限幅和输出饱和；禁能立即清积分。
// 输入：tick间隔至少9拍；本四轮核心窗口至少70拍，不允许重叠提交。
// kp/ki非负Q16.16且bit31=0；Ki已包含控制周期；输入在tick拍锁存。
// 输出：effort为-32767..32767；integral_q16为Q16积分；valid在新结果提交时脉冲一拍。
// 每个乘法、65位加法、限幅各自寄存，避免反馈到PWM的单拍长算术链。
module pi_speed_pipeline(
    input wire clk,rst_n,enable,tick,
    input wire signed [31:0] target_cps,measured_cps,
    input wire [31:0] kp_q16,ki_tick_q16,input wire [15:0] effort_limit,
    output reg signed [15:0] effort,
    output reg signed [63:0] integral_q16,output reg valid
);
    // 上限0禁能，1..32767有效；在tick拍锁存，输出与抗积分饱和采用同一个限制。
    reg signed [64:0] limit_q16;
    reg signed [15:0] limit_effort;
    reg [2:0] stage;
    reg signed [32:0] error_cps;
    reg [30:0] kp,ki;
    reg signed [63:0] p_term,i_step,clamped_i,chosen_i;
    reg signed [64:0] candidate,trial_total,total;
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            stage<=0;error_cps<=0;kp<=0;ki<=0;p_term<=0;i_step<=0;clamped_i<=0;
            chosen_i<=0;candidate<=0;trial_total<=0;total<=0;effort<=0;integral_q16<=0;valid<=0;
            limit_q16<=0;limit_effort<=0;
        end else begin
            valid<=0;
            if(!enable || effort_limit==0 || effort_limit>32767) begin stage<=0;effort<=0;integral_q16<=0;end
            else case(stage)
                0:if(tick) begin
                    error_cps<={target_cps[31],target_cps}-{measured_cps[31],measured_cps};
                    kp<=kp_q16[30:0];ki<=ki_tick_q16[30:0];stage<=1;
                    limit_q16<=$signed({33'b0,effort_limit,16'b0});limit_effort<=$signed(effort_limit);
                end
                1:begin
                    p_term<=$signed(error_cps)*$signed({1'b0,kp});
                    i_step<=$signed(error_cps)*$signed({1'b0,ki});stage<=2;
                end
                2:begin candidate<={integral_q16[63],integral_q16}+{i_step[63],i_step};stage<=3;end
                3:begin
                    if(candidate>limit_q16) clamped_i<=limit_q16[63:0];
                    else if(candidate< -limit_q16) clamped_i<= -$signed(limit_q16[63:0]);
                    else clamped_i<=candidate[63:0];
                    stage<=4;
                end
                4:begin trial_total<={p_term[63],p_term}+{clamped_i[63],clamped_i};stage<=5;end
                5:begin
                    // 推向饱和的误差冻结积分，反向误差允许退出饱和。
                    if((trial_total>limit_q16 && error_cps>0)||(trial_total< -limit_q16 && error_cps<0))
                        chosen_i<=integral_q16;
                    else chosen_i<=clamped_i;
                    stage<=6;
                end
                6:begin total<={p_term[63],p_term}+{chosen_i[63],chosen_i};stage<=7;end
                7:begin
                    integral_q16<=chosen_i;valid<=1;stage<=0;
                    if(total>limit_q16) effort<=limit_effort;
                    else if(total< -limit_q16) effort<= -limit_effort;
                    else effort<=total[31:16]; // 二补码切片与算术右移16后取signed16一致。
                end
            endcase
        end
    end
endmodule
