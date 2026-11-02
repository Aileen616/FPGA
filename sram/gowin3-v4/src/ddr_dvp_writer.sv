// 功能：DVP完整帧打包为16像素突发；DDR末尾读屏障完成后才发布元数据。
// 参数：BASE/STRIDE单位32位DQ字；SNAPSHOT=1冻结全图，否则三缓冲并降采样2倍。
// 接口：request为网络toggle；display_ack/read_bank为显示换帧确认；published为源域toggle。
// malformed或FIFO溢出只产生无效完成，禁止将不完整帧送入HDMI。
module ddr_dvp_writer #(parameter WIDTH=800,HEIGHT=480,SNAPSHOT=1,
    parameter [28:0] BASE=0,STRIDE=29'd16384)(
    input wire pclk,cam_rst_n,vsync,href,input wire [7:0] data,input wire [63:0] time_gray,
    input wire request,display_ack,input wire [1:0] read_bank,
    output reg published,frame_valid,output reg [1:0] published_bank,
    output reg [63:0] timestamp,output reg overflow,
    input wire mem_clk,mem_rst_n,output wire req_valid,req_write,input wire req_ready,
    output wire [28:0] req_addr,output wire [255:0] req_data,input wire rsp_valid);
    localparam PIXELS=WIDTH*HEIGHT/4;
    (* ASYNC_REG="TRUE" *) reg req_meta,req_sync,ack_meta,ack_sync,flush_meta,flush_sync;
    (* ASYNC_REG="TRUE" *) reg [63:0] time_meta,time_sync;
    reg [1:0] read_meta,read_sync,ack_settle;
    reg flush_ack,flush_tag,vs_d,href_d,capture,malformed,closing,waiting;
    reg [1:0] write_bank;
    reg [12:0] bytes_in_line;reg [11:0] lines;reg [16:0] count;
    reg [7:0] high_byte;reg [255:0] pack;
    reg out_valid;wire out_ready;reg [285:0] out_data;
    reg saved_valid,saved_request;reg [63:0] saved_time;
    wire store=capture && !vsync && href && bytes_in_line[0] && bytes_in_line<2*WIDTH && lines<HEIGHT
        && count<PIXELS && (!bytes_in_line[1] && !lines[0]);
    // 功能：解码采集时间Gray码；参数64位微秒Gray计数；返回二进制微秒。
    function [63:0] decode_gray(input [63:0] value);
        integer j;begin for(j=0;j<64;j=j+1) decode_gray[j]=^(value>>j);end
    endfunction
    // 功能：选择非显示且非刚完成的第三块；参数有效bank为0..2；返回空闲bank。
    function [1:0] free_bank(input [1:0] reading,writing);
        begin if(reading!=0 && writing!=0)free_bank=0;else if(reading!=1 && writing!=1)free_bank=1;else free_bank=2;end
    endfunction
    wire fifo_valid,fifo_ready;wire [285:0] fifo_data;
    ddr_async_fifo fifo(pclk,cam_rst_n,out_valid,out_ready,out_data,mem_clk,mem_rst_n,fifo_valid,fifo_ready,fifo_data);
    reg barrier_pending;
    assign req_valid=fifo_valid && !barrier_pending;
    assign req_write=!fifo_data[285];
    assign req_addr=fifo_data[284:256];assign req_data=fifo_data[255:0];
    assign fifo_ready=req_ready && !barrier_pending;
    always @(posedge mem_clk or negedge mem_rst_n)
        if(!mem_rst_n) begin flush_ack<=0;barrier_pending<=0;end
        else begin
            if(req_valid && req_ready && fifo_data[285])barrier_pending<=1;
            if(barrier_pending && rsp_valid)begin barrier_pending<=0;flush_ack<=~flush_ack;end
        end
    always @(posedge pclk or negedge cam_rst_n) begin
        if(!cam_rst_n) begin
            req_meta<=0;req_sync<=0;ack_meta<=0;ack_sync<=0;ack_settle<=0;read_meta<=0;read_sync<=0;
            flush_meta<=0;flush_sync<=0;flush_tag<=0;time_meta<=0;time_sync<=0;
            published<=0;frame_valid<=0;published_bank<=0;timestamp<=0;overflow<=0;
            vs_d<=0;href_d<=0;capture<=0;malformed<=0;closing<=0;waiting<=0;write_bank<=SNAPSHOT?0:1;
            bytes_in_line<=0;lines<=0;count<=0;high_byte<=0;pack<=0;out_valid<=0;out_data<=0;
            saved_valid<=0;saved_request<=0;saved_time<=0;
        end else begin
            req_meta<=request;req_sync<=req_meta;ack_meta<=display_ack;ack_sync<=ack_meta;
            ack_settle<={ack_settle[0],ack_sync==published};read_meta<=read_bank;read_sync<=read_meta;
            flush_meta<=flush_ack;flush_sync<=flush_meta;time_meta<=time_gray;time_sync<=time_meta;
            vs_d<=vsync;href_d<=href;
            if(out_valid && out_ready)out_valid<=0;
            if(waiting && flush_sync!=flush_tag)begin
                waiting<=0;flush_tag<=flush_sync;
                if(SNAPSHOT)begin published<=saved_request;frame_valid<=saved_valid;timestamp<=saved_time;end
                else if(saved_valid && ack_sync==published && ack_settle[1])begin
                    published_bank<=write_bank;published<=~published;frame_valid<=1;
                    write_bank<=free_bank(read_sync,write_bank);
                end
            end
            // 屏障也是队列元素，保证所有前序写事务已交给控制器并由后继读排空。
            if(closing && !out_valid)begin
                out_valid<=1;out_data<={1'b1,BASE+write_bank*STRIDE,256'b0};closing<=0;waiting<=1;
            end
            if(!vsync && vs_d && !capture && !closing && !waiting && !out_valid
                && (!SNAPSHOT || req_sync!=published))begin
                capture<=1;malformed<=0;bytes_in_line<=0;lines<=0;count<=0;pack<=0;
                saved_request<=req_sync;saved_time<=decode_gray(time_sync);
            end else if(vsync && !vs_d && capture)begin
                capture<=0;closing<=1;
                saved_valid<=!malformed && !href && !href_d && lines==HEIGHT && count==PIXELS;
            end else if(capture && !vsync)begin
                if(href)begin
                    if(!bytes_in_line[0])high_byte<=data;
                    if(bytes_in_line<8191)bytes_in_line<=bytes_in_line+1'b1;else malformed<=1;
                    if(bytes_in_line>=2*WIDTH || lines>=HEIGHT)malformed<=1;
                    if(store)begin
                        pack[count[3:0]*16+:16]<={high_byte,data};count<=count+1'b1;
                        if(count[3:0]==15)begin
                            if(!out_valid || out_ready)begin
                                out_valid<=1;out_data<={1'b0,BASE+write_bank*STRIDE+((count>>4)<<3),{high_byte,data},pack[239:0]};
                            end else begin malformed<=1;overflow<=1;end
                        end
                    end
                end else if(href_d)begin
                    if(bytes_in_line!=2*WIDTH)malformed<=1;
                    bytes_in_line<=0;if(lines<4095)lines<=lines+1'b1;else malformed<=1;
                end
            end
        end
    end
endmodule
