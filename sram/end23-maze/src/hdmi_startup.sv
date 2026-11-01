// 功能：PLL锁定后，在输出时钟关闭期间释放DVI复位，再延迟开启PLL输出。
// clk为21MHz控制时钟；rst_n低有效；locked须来自同域PLL_INIT。
// reset_n为DVI低有效复位；clock_enable接PLL ENCLK0/1，不用逻辑门切时钟。
module hdmi_startup(input wire clk,rst_n,locked,output reg reset_n,clock_enable);
    reg [5:0] count;
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin count<=0;reset_n<=0;clock_enable<=0;end
        else if(!locked) begin count<=0;reset_n<=0;clock_enable<=0;end
        else begin
            if(count!=63) count<=count+1'b1;
            // 给PLL输出门控充分关断时间；两次动作间隔24个控制周期，约1.14us。
            if(count==7) reset_n<=1;
            if(count==31) clock_enable<=1;
        end
    end
endmodule
