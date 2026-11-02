// 功能：保持80A2网络冻结帧握手和采集时间戳，完整像素只存DDR。
// 接口：complete/valid/timestamp处于网口域；数据ready支持DDR可变延迟。
module ddr_snapshot #(parameter [28:0] BASE=0)(
    input wire pclk,cam_rst_n,vsync,href,input wire [7:0] data,input wire [63:0] time_gray,
    input wire net_clk,net_rst_n,request,input wire [16:0] rd_addr,
    output wire [15:0] pixel,output wire pixel_ready,output reg complete,valid,
    output reg [63:0] timestamp_us,output wire overflow,
    input wire mem_clk,mem_rst_n,output wire wr_valid,wr_write,input wire wr_ready,
    output wire [28:0] wr_addr,output wire [255:0] wr_data,input wire wr_response,
    output wire rd_valid,input wire rd_ready,output wire [28:0] rd_address,
    input wire rd_response,input wire [255:0] rd_data);
    wire published,frame_valid;wire [63:0] timestamp;
    ddr_dvp_writer #(.BASE(BASE)) writer(.pclk(pclk),.cam_rst_n(cam_rst_n),.vsync(vsync),.href(href),.data(data),
        .time_gray(time_gray),.request(request),.display_ack(1'b0),.read_bank(2'b0),
        .published(published),.frame_valid(frame_valid),.published_bank(),.timestamp(timestamp),.overflow(overflow),
        .mem_clk(mem_clk),.mem_rst_n(mem_rst_n),.req_valid(wr_valid),.req_write(wr_write),.req_ready(wr_ready),
        .req_addr(wr_addr),.req_data(wr_data),.rsp_valid(wr_response));
    (* ASYNC_REG="TRUE" *) reg done_meta,done_sync;reg [1:0] settle;
    always @(posedge net_clk or negedge net_rst_n)
        if(!net_rst_n)begin done_meta<=0;done_sync<=0;complete<=0;valid<=0;settle<=0;timestamp_us<=0;end
        else begin
            done_meta<=published;done_sync<=done_meta;settle<={settle[0],done_sync!=complete};
            if(done_sync!=complete && settle[1])begin complete<=done_sync;valid<=frame_valid;timestamp_us<=timestamp;end
        end
    ddr_snapshot_reader #(.BASE(BASE)) reader(net_clk,net_rst_n,complete,rd_addr,pixel,pixel_ready,
        mem_clk,mem_rst_n,rd_valid,rd_ready,rd_address,rd_response,rd_data);
endmodule
