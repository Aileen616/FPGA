// 功能：MOTOR3四轮开环及显式配置的10ms单向速度PI，默认禁能、300ms独立租约。
// 通道bit0..3=板3/2/4/1=RL/FR/FL/RR；effort是驱动力而非实测速度，硬限16384即50%。
// 00 ID=4D465703，04 active/fault/configured，08命令1清故障/4停止/8原子快照。
// 0C mask[3:0]/invert[7:4]，10序号，14开环MOV3=4D4F5633/闭环CLS1=434C5331。
// 20..2C原始位置快照，30..3C非法AB快照，40最大幅值，44序号，48年龄，4C运行拍数。
// 50..5C开环signed16 shadow，60..6C实际signed16反馈；均32位符号扩展，范围±16384。
// 70能力434C5031，74模式0开环/1闭环，78编码器invert，7C各轮PI配置完成位。
// 80..8C KpQ16，90..9C Ki_tickQ16，A0..AC limit，B0..BC signed target_cps shadow(±200000)。
// C0..CC速度快照，D0快照序号，D4故障位，D8固定10000us；DC同步A[3:0]/B[7:4]/ready[11:8]。
// E0..EC实际effort快照，F0..FC闭环目标快照；原始位置不随encoder invert变化。
// D4位0租约/硬限，1通信丢失，2非法写/配置，3非法提交，31复位未清故障；
// 各轮故障[11:8]500ms无净正向进展，[15:12]连续3窗口反向，[19:16]非法AB，[23:20]测速溢出。
// D4为实时锁存故障原因，不属于08=8冻结的快照组，停机后无需额外快照即可诊断。
// 参数/模式/invert仅停机可改并废弃shadow；每轮须完整写Kp/Ki/limit且Kp或Ki非零才可闭环。
// Kp/Ki非负Q16.16且bit31=0，limit默认4096但须显式写，硬限16384；不提供Kd，未知写锁存停。
// MAX_RUN_CYCLES=0连续续期，非零为不可延长的单次最长拍数；RAMP_STEP默认256/10ms。
// STOP/CLEAR/改invert不能跳过100ms全局连续禁能换向空窗；禁能清积分，不自动重新使能。
// TB6612禁能为IN1/IN2=11、PWM=0，仅说明电气状态；ready仅说明同步完成而非传感器健康。
module motor_forward_wb #(
    parameter integer CLK_HZ=21000000,PWM_HZ=20000,TIMEOUT_CYCLES=CLK_HZ*3/10,
    parameter integer MAX_RUN_CYCLES=0,EFFORT_MAX=16384,RAMP_STEP=256
)(
    input wire clk,rst_n,wb_cyc,wb_stb,wb_we,
    input wire [31:0] wb_addr,wb_wdata,input wire [3:0] wb_sel,
    output reg wb_ack,output reg [31:0] wb_rdata,
    input wire safety_allowed,input wire [3:0] encoder_a,encoder_b,
    output wire [3:0] motor_in1,motor_in2,motor_pwm,output reg motor_active,motor_fault
);
    localparam integer PERIOD_CYCLES=CLK_HZ/PWM_HZ;
    localparam integer BLANK_CYCLES=(CLK_HZ/500000<1)?1:CLK_HZ/500000;
    localparam integer RAMP_CYCLES=(CLK_HZ/100<1)?1:CLK_HZ/100;
    localparam integer REVERSE_CYCLES=(CLK_HZ/10<1)?1:CLK_HZ/10;
    localparam integer EFFORT_LIMIT=(EFFORT_MAX>19660)?19660:((EFFORT_MAX<1)?1:EFFORT_MAX);
    localparam integer STEP=(RAMP_STEP>4096)?4096:((RAMP_STEP<1)?1:RAMP_STEP);
    reg held,closed_mode;
    reg [3:0] wheel_mask,dir_invert,encoder_invert,target_written,cps_written;
    reg [31:0] shadow_seq,last_seq;reg shadow_valid;
    reg signed [15:0] shadow_effort[0:3],target_effort[0:3],actual_effort[0:3];
    reg signed [31:0] shadow_cps[0:3],target_cps[0:3];
    reg [31:0] kp[0:3],ki[0:3];reg [15:0] pi_limit[0:3];
    reg [2:0] pid_written[0:3]; // 已就绪后改任一字段，仅该轮重新收集完整三字段。
    wire [3:0] pid_ready,encoder_ready,closed_fault,target_nonzero,target_sign;
    wire [3:0] closed_reason[0:3];wire signed [15:0] closed_effort[0:3];
    wire signed [31:0] speed_cps[0:3];wire signed [1:0] encoder_delta[0:3];
    reg [31:0] age,runtime,ramp_age,quiet_age;
    reg [3:0] previous_dir,previous_valid;reg [15:0] effort;
    reg [127:0] position_snapshot,illegal_snapshot,speed_snapshot,effort_snapshot,target_snapshot;
    reg [31:0] snapshot_seq,fault_reason;
    wire [127:0] position,illegal_count;
    wire [511:0] edge_capture[0:3];
    (* ASYNC_REG="TRUE" *) reg [3:0] a_meta,a_sync,b_meta,b_sync;
    wire [3:0] pwm_mag,pwm_dir,pwm_enable;wire unused_stby;
    wire request=wb_cyc && wb_stb;
    wire access=request && !held;
    wire legal=(wb_addr[31:8]==24'h200030 || (!wb_we && wb_addr[31:8]==24'h200031)) && wb_addr[1:0]==0 && (!wb_we || wb_sel==4'hf);
    wire configured=wheel_mask!=0;
    wire expired=motor_active && (age>=TIMEOUT_CYCLES-1 || (MAX_RUN_CYCLES!=0 && runtime>=MAX_RUN_CYCLES-1));
    wire safety_lost=motor_active && !safety_allowed;
    wire lease_allowed=rst_n && motor_active && !motor_fault && safety_allowed;
    reg reverse_requested,valid_write;
    wire direction_ready=!reverse_requested || quiet_age>=REVERSE_CYCLES;
    wire drive_allowed=lease_allowed && direction_ready && !(|closed_fault);
    wire fresh_commit=shadow_valid && shadow_seq>last_seq && configured && !motor_fault && safety_allowed && !expired &&
        ((!closed_mode && wb_wdata==32'h4d4f5633 && &target_written) ||
         (closed_mode && wb_wdata==32'h434c5331 && &cps_written && (pid_ready & wheel_mask)==wheel_mask));
    wire write_access=access && wb_we;
    wire write_fault=write_access && (!legal || !valid_write);
    wire configuration_write=write_access && legal && valid_write &&
        (wb_addr[7:0]==8'h0c || wb_addr[7:0]==8'h74 || wb_addr[7:0]==8'h78 ||
         wb_addr[7:4]==4'h8 || wb_addr[7:4]==4'h9 || wb_addr[7:4]==4'ha);
    reg [31:0] new_faults;integer i,j;
    // 功能：求受限signed16幅值；value范围±16384；返回0..16384。
    function [15:0] magnitude(input signed [15:0] value);
        begin magnitude=value[15]?-value:value;end
    endfunction
    // 功能：开环每10ms向目标移动STEP；参数范围±16384；返回不越过目标的signed16值。
    function signed [15:0] ramp_next(input signed [15:0] current_value,input signed [15:0] target_value);
        reg signed [16:0] distance;
        begin
            distance=$signed({target_value[15],target_value})-$signed({current_value[15],current_value});
            if(distance>STEP) ramp_next=current_value+STEP;
            else if(distance< -STEP) ramp_next=current_value-STEP;
            else ramp_next=target_value;
        end
    endfunction
    always @* begin
        effort=0;reverse_requested=0;
        for(j=0;j<4;j=j+1) begin
            if(magnitude(actual_effort[j])>effort) effort=magnitude(actual_effort[j]);
            if(wheel_mask[j] && target_nonzero[j] && previous_valid[j] &&
               (target_sign[j]^dir_invert[j])!=previous_dir[j]) reverse_requested=1;
        end
        valid_write=0;
        case(wb_addr[7:0])
            8'h08:valid_write=(wb_wdata==1 && !motor_active) || wb_wdata==4 || wb_wdata==8;
            8'h0c:valid_write=!motor_active && wb_wdata[31:8]==0 && wb_wdata[3:0]!=0;
            8'h10:valid_write=1;
            8'h14:valid_write=fresh_commit;
            8'h50,8'h54,8'h58,8'h5c:valid_write=wb_wdata=={{16{wb_wdata[15]}},wb_wdata[15:0]} &&
                $signed(wb_wdata)>= -EFFORT_LIMIT && $signed(wb_wdata)<=EFFORT_LIMIT;
            8'h74:valid_write=!motor_active && (wb_wdata==0 ||
                (wb_wdata==1 && configured && (pid_ready & wheel_mask)==wheel_mask));
            8'h78:valid_write=!motor_active && wb_wdata[31:4]==0;
            8'h80,8'h84,8'h88,8'h8c,8'h90,8'h94,8'h98,8'h9c:valid_write=!motor_active && !wb_wdata[31];
            8'ha0,8'ha4,8'ha8,8'hac:valid_write=!motor_active && wb_wdata>=1 && wb_wdata<=EFFORT_LIMIT && wb_wdata<=16384;
            8'hb0,8'hb4,8'hb8,8'hbc:valid_write=$signed(wb_wdata)>= -200000 && $signed(wb_wdata)<=200000;
            default:valid_write=0;
        endcase
        new_faults=0;
        if(expired) new_faults[0]=1;
        if(safety_lost) new_faults[1]=1;
        if(write_fault) begin
            if(legal && wb_addr[7:0]==8'h14) new_faults[3]=1;else new_faults[2]=1;
        end
        for(j=0;j<4;j=j+1) if(motor_active && closed_mode && wheel_mask[j] && closed_fault[j]) begin
            new_faults[8+j]=closed_reason[j][0];new_faults[12+j]=closed_reason[j][1];
            new_faults[16+j]=closed_reason[j][2];new_faults[20+j]=closed_reason[j][3];
        end
    end
    genvar w;
    generate for(w=0;w<4;w=w+1) begin:wheel
        assign pid_ready[w]=&pid_written[w] && ((|kp[w]) || (|ki[w]));
        assign target_nonzero[w]=closed_mode ? target_cps[w]!=0 : target_effort[w]!=0;
        assign target_sign[w]=closed_mode ? target_cps[w][31] : target_effort[w][15];
        wire [1:0] trace_sample,trace_accepted;
        wire trace_qualified,trace_initialized,trace_illegal;
        encoder_ab encoder(.clk(clk),.rst_n(rst_n),.a_async(encoder_a[w]),.b_async(encoder_b[w]),
            .position(position[w*32+:32]),.delta(encoder_delta[w]),.direction(),
            .illegal_count(illegal_count[w*32+:32]),.ready(encoder_ready[w]),
            .trace_sample(trace_sample),.trace_accepted(trace_accepted),.trace_qualified(trace_qualified),
            .trace_initialized(trace_initialized),.trace_illegal(trace_illegal));
        encoder_edge_capture recorder(.clk(clk),.rst_n(rst_n),.active(motor_active),
            .illegal_event(trace_illegal),.qualified(trace_qualified),.initialized(trace_initialized),
            .sample_ab(trace_sample),.accepted_ab(trace_accepted),.pwm(motor_pwm[w]),
            .in1(motor_in1[w]),.in2(motor_in2[w]),.drive_allowed(drive_allowed),
            .effort(actual_effort[w]),.illegal_count(illegal_count[w*32+:32]),.capture(edge_capture[w]));
        motor_speed_loop #(.CLK_HZ(CLK_HZ),.STEP(STEP)) speed_loop(
            .clk(clk),.rst_n(rst_n),.allowed(drive_allowed && closed_mode && wheel_mask[w]),
            .encoder_ready(encoder_ready[w]),.encoder_invert(encoder_invert[w]),
            .position(position[w*32+:32]),.target_cps(target_cps[w]),.delta(encoder_delta[w]),
            .illegal_count(illegal_count[w*32+:32]),.kp_q16(kp[w]),.ki_tick_q16(ki[w]),
            .effort_limit(pi_limit[w]),.speed_cps(speed_cps[w]),.effort(closed_effort[w]),
            .fault(closed_fault[w]),.reason(closed_reason[w]));
        pwm_open_loop #(.PERIOD_CYCLES(PERIOD_CYCLES),.BLANK_CYCLES(BLANK_CYCLES)) pwm(
            .clk(clk),.rst_n(rst_n),.allowed(drive_allowed && wheel_mask[w] && target_nonzero[w]),
            .effort(actual_effort[w]),.pwm_mag(pwm_mag[w]),.dir(pwm_dir[w]),.enable(pwm_enable[w]));
    end endgenerate
    tb6612_adapter driver(.rst_n(rst_n),.drive_allowed(drive_allowed),.pwm_mag(pwm_mag),
        .dir(pwm_dir^dir_invert),.enable(pwm_enable),.in1(motor_in1),.in2(motor_in2),.pwm(motor_pwm),.stby(unused_stby));
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            held<=0;wb_ack<=0;wb_rdata<=0;wheel_mask<=0;dir_invert<=0;encoder_invert<=0;closed_mode<=0;
            shadow_seq<=0;last_seq<=0;shadow_valid<=0;target_written<=0;cps_written<=0;
            age<=0;runtime<=0;ramp_age<=0;quiet_age<=0;previous_dir<=0;previous_valid<=0;
            motor_active<=0;motor_fault<=1;fault_reason<=32'h80000000;snapshot_seq<=0;
            position_snapshot<=0;illegal_snapshot<=0;speed_snapshot<=0;effort_snapshot<=0;target_snapshot<=0;
            a_meta<=0;a_sync<=0;b_meta<=0;b_sync<=0;
            for(i=0;i<4;i=i+1) begin
                shadow_effort[i]<=0;target_effort[i]<=0;actual_effort[i]<=0;shadow_cps[i]<=0;target_cps[i]<=0;
                kp[i]<=0;ki[i]<=0;pi_limit[i]<=(EFFORT_LIMIT<4096)?EFFORT_LIMIT:4096;pid_written[i]<=0;
            end
        end else begin
            a_meta<=encoder_a;a_sync<=a_meta;b_meta<=encoder_b;b_sync<=b_meta;
            if(!request) begin held<=0;wb_ack<=0;end
            if(|pwm_enable) quiet_age<=0;else if(quiet_age<REVERSE_CYCLES) quiet_age<=quiet_age+1'b1;
            for(i=0;i<4;i=i+1) if(pwm_enable[i]) begin
                previous_dir[i]<=pwm_dir[i]^dir_invert[i];previous_valid[i]<=1;
            end
            if(lease_allowed && reverse_requested && quiet_age>=REVERSE_CYCLES) begin
                for(i=0;i<4;i=i+1) if(wheel_mask[i] && target_nonzero[i]) begin
                    previous_dir[i]<=target_sign[i]^dir_invert[i];previous_valid[i]<=1;
                end
            end
            if(motor_active) begin
                if(age<TIMEOUT_CYCLES) age<=age+1'b1;
                if(runtime!=32'hffffffff) runtime<=runtime+1'b1;
            end
            if(!drive_allowed) begin ramp_age<=0;for(i=0;i<4;i=i+1) actual_effort[i]<=0;end
            else if(closed_mode) begin
                ramp_age<=0;for(i=0;i<4;i=i+1) actual_effort[i]<=wheel_mask[i] ? closed_effort[i] : 16'sd0;
            end else begin
                if(ramp_age>=RAMP_CYCLES-1) ramp_age<=0;else ramp_age<=ramp_age+1'b1;
                for(i=0;i<4;i=i+1) begin
                    if(!wheel_mask[i] || target_effort[i]==0) actual_effort[i]<=0;
                    else if(ramp_age>=RAMP_CYCLES-1) actual_effort[i]<=ramp_next(actual_effort[i],target_effort[i]);
                end
            end
            if(access) begin
                held<=1;wb_ack<=1;wb_rdata<=0;
                if(!legal) wb_rdata<=32'hbad00001;
                else if(wb_we && valid_write) begin
                    case(wb_addr[7:0])
                        8'h08:case(wb_wdata)
                            1:begin
                                motor_fault<=0;fault_reason<=0;last_seq<=0;shadow_seq<=0;
                                shadow_valid<=0;target_written<=0;cps_written<=0;age<=0;runtime<=0;ramp_age<=0;
                                for(i=0;i<4;i=i+1) begin actual_effort[i]<=0;target_effort[i]<=0;target_cps[i]<=0;end
                            end
                            4:begin
                                motor_active<=0;ramp_age<=0;shadow_valid<=0;target_written<=0;cps_written<=0;
                                for(i=0;i<4;i=i+1) begin actual_effort[i]<=0;target_effort[i]<=0;target_cps[i]<=0;end
                            end
                            8:begin
                                position_snapshot<=position;illegal_snapshot<=illegal_count;snapshot_seq<=snapshot_seq+1'b1;
                                for(i=0;i<4;i=i+1) begin
                                    speed_snapshot[i*32+:32]<=speed_cps[i];
                                    effort_snapshot[i*32+:32]<={{16{actual_effort[i][15]}},actual_effort[i]};
                                    target_snapshot[i*32+:32]<=target_cps[i];
                                end
                            end
                        endcase
                        8'h0c:begin wheel_mask<=wb_wdata[3:0];dir_invert<=wb_wdata[7:4];end
                        8'h10:begin shadow_seq<=wb_wdata;shadow_valid<=1;end
                        8'h50,8'h54,8'h58,8'h5c:begin
                            shadow_effort[wb_addr[3:2]]<=wb_wdata[15:0];target_written[wb_addr[3:2]]<=1;
                        end
                        8'h74:closed_mode<=wb_wdata[0];
                        8'h78:encoder_invert<=wb_wdata[3:0];
                        8'h80,8'h84,8'h88,8'h8c:begin
                            kp[wb_addr[3:2]]<=wb_wdata;
                            pid_written[wb_addr[3:2]]<=(pid_written[wb_addr[3:2]]==7)?3'b001:pid_written[wb_addr[3:2]]|3'b001;
                        end
                        8'h90,8'h94,8'h98,8'h9c:begin
                            ki[wb_addr[3:2]]<=wb_wdata;
                            pid_written[wb_addr[3:2]]<=(pid_written[wb_addr[3:2]]==7)?3'b010:pid_written[wb_addr[3:2]]|3'b010;
                        end
                        8'ha0,8'ha4,8'ha8,8'hac:begin
                            pi_limit[wb_addr[3:2]]<=wb_wdata[15:0];
                            pid_written[wb_addr[3:2]]<=(pid_written[wb_addr[3:2]]==7)?3'b100:pid_written[wb_addr[3:2]]|3'b100;
                        end
                        8'hb0,8'hb4,8'hb8,8'hbc:begin shadow_cps[wb_addr[3:2]]<=wb_wdata;cps_written[wb_addr[3:2]]<=1;end
                        8'h14:begin
                            shadow_valid<=0;target_written<=0;cps_written<=0;motor_active<=1;last_seq<=shadow_seq;age<=0;
                            for(i=0;i<4;i=i+1) begin
                                if(closed_mode) begin target_cps[i]<=shadow_cps[i];if(shadow_cps[i]==0) actual_effort[i]<=0;end
                                else begin target_effort[i]<=shadow_effort[i];if(shadow_effort[i]==0) actual_effort[i]<=0;end
                            end
                            if(!motor_active) begin runtime<=0;ramp_age<=0;for(i=0;i<4;i=i+1) actual_effort[i]<=0;end
                        end
                    endcase
                end else if(!wb_we) begin
                    // 0x20003100起每轮64字节，只读扩展页；写扩展页仍走原非法写停车。
                    if(wb_addr[8])wb_rdata<=edge_capture[wb_addr[7:6]][wb_addr[5:2]*32+:32];
                    else case(wb_addr[7:0])
                        8'h00:wb_rdata<=32'h4d465703;
                        8'h04:wb_rdata<={29'b0,configured,motor_fault,motor_active};
                        8'h0c:wb_rdata<={24'b0,dir_invert,wheel_mask};
                        8'h10:wb_rdata<=shadow_seq;
                        8'h20,8'h24,8'h28,8'h2c:wb_rdata<=position_snapshot[wb_addr[3:2]*32+:32];
                        8'h30,8'h34,8'h38,8'h3c:wb_rdata<=illegal_snapshot[wb_addr[3:2]*32+:32];
                        8'h40:wb_rdata<={16'b0,effort};8'h44:wb_rdata<=last_seq;8'h48:wb_rdata<=age;8'h4c:wb_rdata<=runtime;
                        8'h50,8'h54,8'h58,8'h5c:wb_rdata<={{16{shadow_effort[wb_addr[3:2]][15]}},shadow_effort[wb_addr[3:2]]};
                        8'h60,8'h64,8'h68,8'h6c:wb_rdata<={{16{actual_effort[wb_addr[3:2]][15]}},actual_effort[wb_addr[3:2]]};
                        8'h70:wb_rdata<=32'h434c5031;8'h74:wb_rdata<={31'b0,closed_mode};8'h78:wb_rdata<={28'b0,encoder_invert};
                        8'h7c:wb_rdata<={28'b0,pid_ready};
                        8'h80,8'h84,8'h88,8'h8c:wb_rdata<=kp[wb_addr[3:2]];
                        8'h90,8'h94,8'h98,8'h9c:wb_rdata<=ki[wb_addr[3:2]];
                        8'ha0,8'ha4,8'ha8,8'hac:wb_rdata<={16'b0,pi_limit[wb_addr[3:2]]};
                        8'hb0,8'hb4,8'hb8,8'hbc:wb_rdata<=shadow_cps[wb_addr[3:2]];
                        8'hc0,8'hc4,8'hc8,8'hcc:wb_rdata<=speed_snapshot[wb_addr[3:2]*32+:32];
                        8'hd0:wb_rdata<=snapshot_seq;8'hd4:wb_rdata<=fault_reason;8'hd8:wb_rdata<=32'd10000;
                        8'hdc:wb_rdata<={20'b0,encoder_ready,b_sync,a_sync};
                        8'he0,8'he4,8'he8,8'hec:wb_rdata<=effort_snapshot[wb_addr[3:2]*32+:32];
                        8'hf0,8'hf4,8'hf8,8'hfc:wb_rdata<=target_snapshot[wb_addr[3:2]*32+:32];
                        default:wb_rdata<=32'hbad00002;
                    endcase
                end
            end
            if(configuration_write) begin
                shadow_valid<=0;target_written<=0;cps_written<=0;ramp_age<=0;
                for(i=0;i<4;i=i+1) begin target_effort[i]<=0;target_cps[i]<=0;actual_effort[i]<=0;end
            end
            // 保护压过同拍续期/清故障；输出立即撤下，显式CLEAR和全新提交后才能重新使能。
            if(|new_faults) begin
                motor_active<=0;motor_fault<=1;fault_reason<=fault_reason|new_faults;
                ramp_age<=0;shadow_valid<=0;target_written<=0;cps_written<=0;
                for(i=0;i<4;i=i+1) begin actual_effort[i]<=0;target_effort[i]<=0;target_cps[i]<=0;end
            end
        end
    end
endmodule
