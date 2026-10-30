// 功能：四路AB编码器、测速、独立PI和PWM，以及共享命令/急停/故障监督。
// 轮序：bit0或低32位起依次FL、FR、RL、RR；counts/s及位置均为signed32。
// ENCODER_INVERT每位交换相应A/B以校准计数正向；默认0，必须与车轮实际前进方向校准。
// 所有安全输入、控制总线均需在clk域；仅AB由内部同步。复位需异步断言、同步释放。
// 参数工程起点：50MHz、10ms窗口、100ms命令超时、20kHz PWM、2us换向空窗。
// WINDOW_CYCLES至少70；未包含实际PIN、软核总线或电平转换，不能直接替代板级顶层。
module robot_drive_core #(
    parameter integer CLK_HZ=50000000,WINDOW_CYCLES=500000,WATCHDOG_CYCLES=5000000,
    PWM_PERIOD_CYCLES=2500,BLANK_CYCLES=100,RAMP_STEP_CPS=100,MAX_TARGET_CPS=1000000,
    parameter [3:0] ENCODER_INVERT=4'b0000
)(
    input wire clk,rst_n,input wire [3:0] encoder_a,encoder_b,driver_fault,
    input wire e_stop,sensor_fault,obstacle_stop,controller_alive,
    input wire wr_en,rd_en,input wire [11:0] addr,input wire [31:0] wdata,
    output wire [31:0] rdata,output wire bus_error,
    output wire [3:0] pwm_mag,dir,enable,
    output wire drive_allowed,fault_latched,output wire [5:0] fault_reason
);
    wire [127:0] position,measured_cps,target_cps,ramped_cps,kp_q16,ki_tick_q16,illegal_count;
    wire [63:0] effort,effort_limits;
    wire [3:0] encoder_ready,sample_valid,quantized,measurement_overflow,guard_fault;
    wire [7:0] guard_reason;
    reg [7:0] encoder_fault_detail;
    wire all_ready=&encoder_ready,control_tick=&sample_valid;
    wire armed,cmd_stale,arm,disarm,clear_fault,new_command;
    wire [31:0] command_age,no_edge_limit,min_target,illegal_limit;
    wire [15:0] min_effort;
    reg [31:0] sample_sequence;
    reg pi_tick;
    // 所有测速器使用共同ready与参数，计数窗口一致；PI延后一拍使用新的斜坡目标。
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin pi_tick<=0;sample_sequence<=0;encoder_fault_detail<=0;end
        else begin
            pi_tick<=control_tick;
            if(control_tick) sample_sequence<=sample_sequence+1'b1;
            if(|guard_fault) encoder_fault_detail<=encoder_fault_detail|guard_reason;
            else if(clear_fault) encoder_fault_detail<=0;
        end
    end
    robot_drive_regs #(.CLK_HZ(CLK_HZ),.WINDOW_CYCLES(WINDOW_CYCLES),
        .WATCHDOG_CYCLES(WATCHDOG_CYCLES),.MAX_TARGET_CPS(MAX_TARGET_CPS)) regs(
        .clk(clk),.rst_n(rst_n),.wr_en(wr_en),.rd_en(rd_en),.addr(addr),.wdata(wdata),.rdata(rdata),.bus_error(bus_error),
        .armed(armed),.fault_latched(fault_latched),.drive_allowed(drive_allowed),.fault_reason(fault_reason),
        .measured_cps(measured_cps),.position(position),.illegal_count(illegal_count),.quantized(quantized),
        .measurement_overflow(measurement_overflow),.encoder_ready(encoder_ready),.encoder_fault_detail(encoder_fault_detail),
        .sample_sequence(sample_sequence),.active_target_cps(target_cps),.kp_q16(kp_q16),.ki_tick_q16(ki_tick_q16),
        .no_edge_limit(no_edge_limit),.min_target(min_target),.min_effort(min_effort),.illegal_limit(illegal_limit),
        .effort(effort),.effort_limits(effort_limits),
        .arm(arm),.disarm(disarm),.clear_fault(clear_fault),.new_command(new_command),.cmd_stale(cmd_stale),.command_age(command_age));
    safety_supervisor safety(.clk(clk),.rst_n(rst_n),.e_stop(e_stop),.driver_fault(|driver_fault),
        .cmd_stale(cmd_stale),.sensor_fault(sensor_fault || (|measurement_overflow) || (|guard_fault)),
        .obstacle_stop(obstacle_stop),.softcore_alive(controller_alive),.arm(arm),.disarm(disarm),
        .clear_fault(clear_fault),.new_command(new_command),.drive_allowed(drive_allowed),
        .armed(armed),.fault_latched(fault_latched),.fault_reason(fault_reason));
    genvar w;
    generate for(w=0;w<4;w=w+1) begin: wheels
        wire signed [1:0] edge_delta;
        encoder_ab encoder(.clk(clk),.rst_n(rst_n),
            .a_async(ENCODER_INVERT[w]?encoder_b[w]:encoder_a[w]),
            .b_async(ENCODER_INVERT[w]?encoder_a[w]:encoder_b[w]),
            .position(position[w*32+:32]),.delta(edge_delta),.direction(),
            .illegal_count(illegal_count[w*32+:32]),.ready(encoder_ready[w]));
        speed_sample #(.CLK_HZ(CLK_HZ),.WINDOW_CYCLES(WINDOW_CYCLES)) speed(
            .clk(clk),.rst_n(rst_n),.ready(all_ready),.position($signed(position[w*32+:32])),
            .cps(measured_cps[w*32+:32]),.valid(sample_valid[w]),.quantized(quantized[w]),.overflow(measurement_overflow[w]));
        target_ramp #(.STEP_CPS(RAMP_STEP_CPS)) ramp(.clk(clk),.rst_n(rst_n),.enable(drive_allowed),
            .tick(control_tick),.target_cps($signed(target_cps[w*32+:32])),.ramped_cps(ramped_cps[w*32+:32]));
        pi_speed_pipeline pi(.clk(clk),.rst_n(rst_n),.enable(drive_allowed),.tick(pi_tick),
            .target_cps($signed(ramped_cps[w*32+:32])),.measured_cps($signed(measured_cps[w*32+:32])),
            .kp_q16(kp_q16[w*32+:32]),.ki_tick_q16(ki_tick_q16[w*32+:32]),
            .effort_limit(effort_limits[w*16+:16]),.effort(effort[w*16+:16]),.integral_q16(),.valid());
        pwm_open_loop #(.PERIOD_CYCLES(PWM_PERIOD_CYCLES),.BLANK_CYCLES(BLANK_CYCLES)) pwm(
            .clk(clk),.rst_n(rst_n),.allowed(drive_allowed && effort_limits[w*16+:16]!=0),.effort($signed(effort[w*16+:16])),
            .pwm_mag(pwm_mag[w]),.dir(dir[w]),.enable(enable[w]));
        encoder_guard guard(.clk(clk),.rst_n(rst_n),.allowed(drive_allowed),.tick(control_tick),
            .edge_seen(edge_delta!=0),.target_cps($signed(ramped_cps[w*32+:32])),.effort($signed(effort[w*16+:16])),
            .illegal_count(illegal_count[w*32+:32]),.no_edge_limit(no_edge_limit),.min_target(min_target),
            .min_effort(min_effort),.illegal_limit(illegal_limit),.fault(guard_fault[w]),.reason(guard_reason[w*2+:2]));
    end endgenerate
endmodule
