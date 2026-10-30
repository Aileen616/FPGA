// 功能：在系统时钟域保留短暂失锁事件，避免半秒一次的UART漏报。
// 输入：clk为持续系统时钟，rst_n低有效，locked_async为PLL初始化后的锁定信号。
// 输出：locked_sync为同步锁定状态；loss_count为首次锁定后失锁次数，0..15饱和。
// 异步清零将窄失锁脉冲拉长到至少一个系统周期；极密集事件可能合并，不能用于抖动测量。
module hdmi_lock_monitor(
    input wire clk,rst_n,locked_async,
    output wire locked_sync,output reg [3:0] loss_count);
    (* ASYNC_REG="TRUE" *) reg [1:0] lock_release;
    reg previous_lock;
    wire release_n=rst_n && locked_async;
    always @(posedge clk or negedge release_n)
        if(!release_n) lock_release<=0;
        else lock_release<={lock_release[0],1'b1};
    assign locked_sync=lock_release[1];
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin previous_lock<=0;loss_count<=0;end
        else begin
            previous_lock<=locked_sync;
            if(previous_lock && !locked_sync && loss_count!=15)
                loss_count<=loss_count+1'b1;
        end
    end
endmodule
