// 功能：双时钟同步读RAM队列；只同步Gray指针，数据经双口RAM传递。
// 参数：WIDTH为负载位数，LGDEPTH>=2为队列深度的二进制对数。
// 接口：两侧复位必须来自同一视频复位源；valid/ready握手时数据传送一次。
module ddr_async_fifo #(parameter WIDTH=286, LGDEPTH=6)(
    input wire wclk,wrst_n,input wire wvalid,output wire wready,input wire [WIDTH-1:0] wdata,
    input wire rclk,rrst_n,output reg rvalid,input wire rready,output reg [WIDTH-1:0] rdata);
    (* ram_style="block" *) reg [WIDTH-1:0] mem[0:(1<<LGDEPTH)-1];
    reg [LGDEPTH:0] wp,rp,wgray,rgray;
    (* ASYNC_REG="TRUE" *) reg [LGDEPTH:0] rg_meta,rg_sync,wg_meta,wg_sync;
    wire [LGDEPTH:0] wn=wp+1'b1;
    wire full=wgray=={~rg_sync[LGDEPTH:LGDEPTH-1],rg_sync[LGDEPTH-2:0]};
    wire empty=rgray==wg_sync;
    assign wready=!full;
    always @(posedge wclk) if(wvalid && wready) mem[wp[LGDEPTH-1:0]]<=wdata;
    always @(posedge wclk or negedge wrst_n)
        if(!wrst_n) begin wp<=0;wgray<=0;rg_meta<=0;rg_sync<=0;end
        else begin
            rg_meta<=rgray;rg_sync<=rg_meta;
            if(wvalid && wready) begin wp<=wn;wgray<=wn^(wn>>1);end
        end
    // 将RAM输出寄存在读域；已预取的数据只在消费者握手后覆盖。
    always @(posedge rclk) if((!rvalid || rready) && !empty) rdata<=mem[rp[LGDEPTH-1:0]];
    always @(posedge rclk or negedge rrst_n)
        if(!rrst_n) begin rp<=0;rgray<=0;rvalid<=0;wg_meta<=0;wg_sync<=0;end
        else begin
            wg_meta<=wgray;wg_sync<=wg_meta;
            if(!rvalid || rready) begin
                rvalid<=!empty;
                if(!empty) begin rp<=rp+1'b1;rgray<=(rp+1'b1)^((rp+1'b1)>>1);end
            end
        end
endmodule
