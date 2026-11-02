// 功能：异步 AB 四倍频解码；正序 00→01→11→10→00，位置为有符号 32 位 counts。
// delta：每系统拍 -1/0/+1；direction：最后合法步进方向，1 为正；非法跳变只累计诊断。
// 参数：FILTER_CYCLES为同步后AB组合连续稳定拍数，默认4；21MHz下约190.5ns。
// 前提：合法AB状态至少保持FILTER_CYCLES拍；本过滤不能修复接线或持续的非法AB电平。
module encoder_ab #(parameter integer FILTER_CYCLES=4)(
    input wire clk, rst_n, a_async, b_async,
    output reg signed [31:0] position,
    output reg signed [1:0] delta,
    output reg direction,
    output reg [31:0] illegal_count,
    output wire ready,
    output wire [1:0] trace_sample,trace_accepted,
    output wire trace_qualified,trace_initialized,trace_illegal
);
    (* ASYNC_REG = "TRUE" *) reg [1:0] ab_meta, ab_sync;
    localparam integer FILTER_BITS=(FILTER_CYCLES<2)?1:$clog2(FILTER_CYCLES+1);
    reg [2:0] startup; // 等待同步链充满后才开始稳定资格检查。
    reg [1:0] previous,candidate;
    reg [FILTER_BITS-1:0] stable_count; // 当前组合连续出现次数，达到阈值后饱和。
    reg initialized; // 首次稳定组合只建立基准，不产生位置或非法跳变。
    wire qualified=(FILTER_CYCLES==1)||((candidate==ab_sync)&&(stable_count>=FILTER_CYCLES-1));
    assign ready = initialized;
    // 被动引出真实解码判据；不反向连接任何计数或保护路径。
    assign trace_sample=ab_sync;
    assign trace_accepted=previous;
    assign trace_qualified=qualified&&(&startup);
    assign trace_initialized=initialized;
    assign trace_illegal=trace_qualified&&initialized&&((previous^ab_sync)==2'b11);
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ab_meta <= 0; ab_sync <= 0; startup <= 0; previous <= 0;
            candidate<=0;stable_count<=0;initialized<=0;
            position <= 0; delta <= 0; direction <= 0; illegal_count <= 0;
        end else begin
            ab_meta <= {a_async,b_async}; ab_sync <= ab_meta;
            startup <= {startup[1:0],1'b1}; delta <= 0;
            if (&startup) begin
                if(candidate!=ab_sync)begin candidate<=ab_sync;stable_count<=1;end
                else if(stable_count<FILTER_CYCLES)stable_count<=stable_count+1'b1;
                // 对整个AB组合资格检查，避免分路不同延迟制造本不存在的相位顺序。
                if(qualified)begin
                    previous<=ab_sync;initialized<=1;
                    if(initialized)case ({previous,ab_sync})
                    4'b0001,4'b0111,4'b1110,4'b1000: begin
                        position <= position + 32'sd1; delta <= 1; direction <= 1;
                    end
                    4'b0010,4'b1011,4'b1101,4'b0100: begin
                        position <= position - 32'sd1; delta <= -1; direction <= 0;
                    end
                    4'b0011,4'b1100,4'b0110,4'b1001: illegal_count <= illegal_count + 1'b1;
                    default: begin end
                    endcase
                end
            end
        end
    end
endmodule

// 功能：被动冻结每次驱动期间的首次非法AB跳变；只观察，无控制反馈。
// 参数：无。所有输入同属clk域；sample_ab/accepted_ab来自实际解码器同步及历史状态。
// 输入：非法判据与解码器同拍；active只界定记录窗口，effort为有符号驱动力而非轮速。
// 输出：512位只读记录，16个32位字；字0低位valid，未记录时全零。
module encoder_edge_capture(
    input wire clk,rst_n,active,illegal_event,qualified,initialized,
    input wire [1:0] sample_ab,accepted_ab,
    input wire pwm,in1,in2,drive_allowed,
    input wire signed [15:0] effort,
    input wire [31:0] illegal_count,
    output reg [511:0] capture
);
    reg active_previous,pwm_previous,history_valid;
    reg [1:0] sample_previous;
    reg [31:0] ticks,accepted_age,sample_age,pwm_age,last_sample_duration;
    reg [3:0] last_sample_transition;
    wire sample_changed=sample_ab!=sample_previous;
    wire accepted_changed=initialized&&qualified&&(sample_ab!=accepted_ab);
    // 功能：计数饱和递增；v为0..2^32-1拍；返回不回绕的拍数。
    function [31:0] increment(input [31:0] v);
        begin increment=(&v)?v:v+1'b1;end
    endfunction
    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)begin
            capture<=0;active_previous<=0;pwm_previous<=0;history_valid<=0;
            sample_previous<=0;ticks<=0;accepted_age<=0;sample_age<=0;pwm_age<=0;
            last_sample_duration<=0;last_sample_transition<=0;
        end else begin
            ticks<=ticks+1'b1;active_previous<=active;pwm_previous<=pwm;
            sample_previous<=sample_ab;
            accepted_age<=accepted_changed?0:increment(accepted_age);
            sample_age<=sample_changed?0:increment(sample_age);
            pwm_age<=(pwm!=pwm_previous)?0:increment(pwm_age);
            if(sample_changed)begin
                history_valid<=1;last_sample_duration<=increment(sample_age);
                last_sample_transition<={sample_previous,sample_ab};
            end
            // 新一次显式启动只清诊断，不影响原保护；停车时保留供长串口查询。
            if(active&&!active_previous)capture<=0;
            if(active&&illegal_event&&(!capture[0]||!active_previous))begin
                capture<=0;
                // 字0：version[31:24]=1，history[13]、drive[12]、IN2/IN1/PWM[11:9]，
                // initialized[8]、qualified[7]、accepted[6:5]、sample[4:3]、active[1]、valid[0]。
                capture[31:0]<={8'd1,10'd0,history_valid,drive_allowed,in2,in1,pwm,
                               initialized,qualified,accepted_ab,sample_ab,1'b0,active,1'b1};
                capture[63:32]<=ticks;
                // 年龄表示距前次边沿的拍数；同拍出现的新边沿记0，21MHz名义每拍47.62ns。
                capture[95:64]<=increment(accepted_age);
                capture[127:96]<=sample_changed?0:increment(sample_age);
                capture[159:128]<=(pwm!=pwm_previous)?0:increment(pwm_age);
                capture[191:160]<={{16{effort[15]}},effort};
                capture[223:192]<=illegal_count;
                capture[255:224]<=sample_changed?increment(sample_age):last_sample_duration;
                capture[287:256]<=sample_changed?{28'd0,sample_previous,sample_ab}:{28'd0,last_sample_transition};
                // 字9..15保留0；不将逻辑电平记录解释为电压、毛刺幅值或机械运动。
            end
        end
    end
endmodule
