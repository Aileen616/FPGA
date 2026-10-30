// 功能：独立统计软核调度、总线等待和视频发送时间，提供跨域一致快照；不参与控制准入。
// 参数：CLK_HZ为软核标称频率Hz；网络计数单位为net_clk周期，生产配置125MHz。
// 接口：0x14写1申请快照，0x10写队列长度，0x18写1记录入队失败，0x1c写1开始旁路循环统计。
// 计数器32位自然回绕；最大时长饱和，溢出/饱和标志锁存。读取不清零，不刷新任何运动租约。
module runtime_perf_wb #(parameter integer CLK_HZ=21000000,PASSIVE_LOOP=1)(
    input wire clk,rst_n,wb_cyc,wb_stb,wb_we,
    input wire [31:0] wb_addr,wb_wdata,input wire [3:0] wb_sel,
    output reg wb_ack,output reg [31:0] wb_rdata,
    input wire observe_cyc,observe_stb,observe_ack,observe_we,input wire [31:0] observe_addr,
    input wire net_clk,net_rst_n,request,input wire [1:0] complete,valid,
    input wire [4:0] send_state,
    input wire [31:0] sent_pairs,dropped_pairs,packet_count);
    reg [31:0] cycles,loops,loop_gap,loop_max,wb_count,wb_wait,wait_run,wait_max,tx_high,tx_drop;
    reg loop_seen,observed,loop_tracking;
    reg [31:0] sys_flags,snapshot_seq;
    reg [255:0] sys_snapshot;
    reg snapshot_req,busy,sys_valid,net_valid,rejected;
    reg [511:0] net_snapshot;
    wire access=wb_cyc&&wb_stb&&!wb_ack;
    wire write_word=access&&wb_we&&wb_sel==4'hf&&wb_addr[1:0]==0;
    // 本候选固件主循环只在入口读取0x20002004一次；启动读在使能统计之前，不能计入循环。
    wire heartbeat=PASSIVE_LOOP?(loop_tracking&&observe_cyc&&observe_stb&&!observed&&!observe_we&&observe_addr==32'h20002004):
                               (write_word&&wb_addr[7:0]==8'h10);
    wire take_snapshot=write_word&&wb_addr[7:0]==8'h14&&wb_wdata==1;
    wire net_reset_n;
    reset_sync net_reset(net_clk,net_rst_n,net_reset_n);
    (* ASYNC_REG="TRUE" *) reg req_meta,req_sync;
    (* ASYNC_REG="TRUE" *) reg ack_meta,ack_sync;
    reg [1:0] settle;
    reg net_ack;
    reg [511:0] net_hold;
    reg [31:0] net_cycles,requests,wait_now,wait_peak,build_cycles,send_cycles,busy_cycles;
    reg [31:0] good0,good1,bad0,bad1,busy_now,busy_peak,net_flags;
    reg request_d;
    reg [1:0] complete_d;
    reg [4:0] state_d;
    reg wait_saturated,busy_saturated;

    // 功能：按8位段展开加一进位，避免125MHz路径串联32位加法；value为原计数；返回自然回绕的新值。
    function [31:0] inc32(input [31:0] value);
        begin
            inc32[7:0]=value[7:0]+8'd1;
            inc32[15:8]=value[15:8]+{7'b0,&value[7:0]};
            inc32[23:16]=value[23:16]+{7'b0,&value[15:0]};
            inc32[31:24]=value[31:24]+{7'b0,&value[23:0]};
        end
    endfunction

    // 功能：32位时长饱和递增；value为周期数；返回不回绕的周期数。
    function [31:0] sat_inc(input [31:0] value);
        begin sat_inc=(&value)?value:value+1'b1;end
    endfunction

    always @(posedge net_clk or negedge net_reset_n) begin
        if(!net_reset_n) begin
            req_meta<=0;req_sync<=0;net_ack<=0;net_hold<=0;
            net_cycles<=0;requests<=0;wait_now<=0;wait_peak<=0;build_cycles<=0;send_cycles<=0;busy_cycles<=0;
            good0<=0;good1<=0;bad0<=0;bad1<=0;busy_now<=0;busy_peak<=0;net_flags<=0;
            request_d<=0;complete_d<=0;state_d<=0;wait_saturated<=0;busy_saturated<=0;
        end else begin
            req_meta<=snapshot_req;req_sync<=req_meta;
            request_d<=request;complete_d<=complete;state_d<=send_state;
            net_cycles<=inc32(net_cycles);
            if(&net_cycles)net_flags[0]<=1;
            if(request!=request_d)requests<=inc32(requests);
            if(complete[0]!=complete_d[0])begin
                if(valid[0])good0<=inc32(good0);else bad0<=inc32(bad0);
            end
            if(complete[1]!=complete_d[1])begin
                if(valid[1])good1<=inc32(good1);else bad1<=inc32(bad1);
            end
            // 分开计等待完整帧、组包和MAC发送；不能把这些计数解释成CPU占用率。
            if(send_state==1)begin
                if(!wait_saturated)wait_now<=inc32(wait_now);
                if(wait_now==32'hfffffffe)wait_saturated<=1;
            end
            else begin
                wait_now<=0;wait_saturated<=0;
                if(state_d==1&&wait_now>wait_peak)wait_peak<=wait_now;
            end
            if(send_state!=0)begin
                if(!busy_saturated)busy_now<=inc32(busy_now);
                if(busy_now==32'hfffffffe)busy_saturated<=1;
                busy_cycles<=inc32(busy_cycles);
            end else begin busy_now<=0;busy_saturated<=0;if(busy_now>busy_peak)busy_peak<=busy_now;end
            if(send_state==8)send_cycles<=inc32(send_cycles);
            if(send_state!=0&&send_state!=1&&send_state!=8&&send_state!=9)build_cycles<=inc32(build_cycles);
            if(wait_saturated||busy_saturated)net_flags[1]<=1;
            if(req_sync!=net_ack)begin
                // 源端负载在ack发出前锁存，此后保持至下一请求；目的端同步ack后再等两拍。
                net_hold<={net_flags,busy_peak,bad1,bad0,good1,good0,busy_cycles,send_cycles,
                           build_cycles,wait_peak,wait_now,packet_count,dropped_pairs,sent_pairs,requests,net_cycles};
                net_ack<=req_sync;
            end
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            cycles<=0;loops<=0;loop_gap<=0;loop_max<=0;wb_count<=0;wb_wait<=0;wait_run<=0;wait_max<=0;
            tx_high<=0;tx_drop<=0;loop_seen<=0;observed<=0;loop_tracking<=0;sys_flags<=0;snapshot_seq<=0;
            sys_snapshot<=0;net_snapshot<=0;snapshot_req<=0;busy<=0;sys_valid<=0;net_valid<=0;rejected<=0;
            ack_meta<=0;ack_sync<=0;settle<=0;wb_ack<=0;wb_rdata<=0;
        end else begin
            cycles<=cycles+1'b1;
            if(&cycles)sys_flags[0]<=1;
            if(loop_seen)loop_gap<=sat_inc(loop_gap);
            if(heartbeat)begin
                loops<=loops+1'b1;
                if(loop_seen&&sat_inc(loop_gap)>loop_max)loop_max<=sat_inc(loop_gap);
                loop_seen<=1;loop_gap<=0;
            end
            if(write_word&&wb_addr[7:0]==8'h10&&wb_wdata<=1023&&wb_wdata>tx_high)tx_high<=wb_wdata;
            if(write_word&&wb_addr[7:0]==8'h1c&&wb_wdata==1)loop_tracking<=1;
            if(write_word&&wb_addr[7:0]==8'h18&&wb_wdata==1)tx_drop<=tx_drop+1'b1;
            if(observe_cyc&&observe_stb)begin
                if(!observed)begin wb_count<=wb_count+1'b1;observed<=1;end
                if(!observe_ack)begin wb_wait<=wb_wait+1'b1;wait_run<=sat_inc(wait_run);end
                else begin if(wait_run>wait_max)wait_max<=wait_run;wait_run<=0;end
            end else begin
                observed<=0;if(wait_run>wait_max)wait_max<=wait_run;wait_run<=0;
            end
            if((&loop_gap)||(&wait_run))sys_flags[1]<=1;
            ack_meta<=net_ack;ack_sync<=ack_meta;
            settle<={settle[0],busy&&(ack_sync==snapshot_req)};
            if(busy&&ack_sync==snapshot_req&&settle[1])begin
                net_snapshot<=net_hold;net_valid<=1;busy<=0;
            end
            if(take_snapshot)begin
                if(busy)rejected<=1;
                else begin
                    sys_snapshot<={tx_drop,tx_high,wait_max,wb_wait,wb_count,loop_max,loops,cycles};
                    snapshot_req<=!snapshot_req;busy<=1;sys_valid<=1;net_valid<=0;rejected<=0;
                    snapshot_seq<=snapshot_seq+1'b1;settle<=0;
                end
            end
            if(!wb_cyc||!wb_stb)wb_ack<=0;
            else if(!wb_ack)begin
                wb_ack<=1;wb_rdata<=0;
                if(wb_addr[1:0]==0)begin
                    case(wb_addr[7:0])
                        8'h00:wb_rdata<=32'h50524631;
                        8'h04:wb_rdata<={28'b0,rejected,sys_valid,net_valid,busy};
                        8'h08:wb_rdata<=snapshot_seq;
                        8'h0c:wb_rdata<=CLK_HZ;
                        8'h1c:wb_rdata<=sys_flags;
                        default:begin
                            if(wb_addr[7:0]>=8'h20&&wb_addr[7:0]<8'h40)
                                wb_rdata<=sys_snapshot[(wb_addr[4:2])*32+:32];
                            else if(wb_addr[7:0]>=8'h40&&wb_addr[7:0]<8'h80)
                                wb_rdata<=net_snapshot[(wb_addr[5:2])*32+:32];
                        end
                    endcase
                end
            end
        end
    end
endmodule
