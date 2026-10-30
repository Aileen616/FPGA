// 功能：检测持续有驱动却无合法编码器边沿，以及单窗口非法AB跳变过多；故障由上层锁存。
// 输入均属于clk域；tick为测速更新；edge_seen每次合法边沿为1；窗口为实际测速窗口。
// no_edge_limit：连续无边沿窗口数，0禁用；illegal_limit：单窗口非法跳变阈值，0禁用。
// min_target为非负counts/s，min_effort为0..32767；低于门槛、禁能或换向时重置无边沿计时。
// 输出 fault 为一拍故障请求；原因bit0无边沿、bit1非法跳变。不能区分堵转与断线。
module encoder_guard(
    input wire clk,rst_n,allowed,tick,edge_seen,
    input wire signed [31:0] target_cps,input wire signed [15:0] effort,
    input wire [31:0] illegal_count,no_edge_limit,min_target,illegal_limit,
    input wire [15:0] min_effort,
    output reg fault,output reg [1:0] reason
);
    reg saw_edge,last_direction;
    reg [31:0] empty_windows,previous_illegal;
    wire [32:0] target_abs=target_cps[31] ? -$signed({target_cps[31],target_cps}) : {1'b0,target_cps};
    wire [16:0] effort_abs=effort[15] ? -$signed({effort[15],effort}) : {1'b0,effort};
    wire qualified=target_abs>=min_target && target_abs!=0 && effort_abs>=min_effort && effort_abs!=0;
    wire [31:0] illegal_delta=illegal_count-previous_illegal;
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin saw_edge<=0;empty_windows<=0;previous_illegal<=0;last_direction<=0;fault<=0;reason<=0;end
        else begin
            fault<=0;reason<=0;
            if(!allowed) begin
                saw_edge<=0;empty_windows<=0;previous_illegal<=illegal_count;last_direction<=target_cps[31];
            end else begin
                if(edge_seen) saw_edge<=1;
                if(tick) begin
                    saw_edge<=0;previous_illegal<=illegal_count;last_direction<=target_cps[31];
                    if(!qualified || last_direction!=target_cps[31] || saw_edge || edge_seen || no_edge_limit==0)
                        empty_windows<=0;
                    else if(empty_windows>=no_edge_limit-1) begin fault<=1;reason[0]<=1;end
                    else empty_windows<=empty_windows+1'b1;
                    if(illegal_limit!=0 && illegal_delta>=illegal_limit) begin fault<=1;reason[1]<=1;end
                end
            end
        end
    end
endmodule
