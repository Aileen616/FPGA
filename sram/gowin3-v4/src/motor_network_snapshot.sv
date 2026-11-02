// 功能：保持8C4F冻结帧采集握手，分组寄存读地址以降低网络读口扇出。
// 参数：WIDTH/HEIGHT为正偶数，AW为像素地址位宽；每组最多8192像素，末组仅实例化实际深度。
// 接口：rd_addr在上升沿采样，经过地址寄存、RAM读取、RAM输出寄存、分组选择四拍得到rd_data。
// 发送端必须匹配四拍读延迟；完成toggle及时间戳的跨域保持约定与原工程相同。
module motor_network_snapshot #(
    parameter integer WIDTH=320, HEIGHT=240, AW=$clog2(WIDTH*HEIGHT)
)(input wire pclk,cam_rst_n,vsync,href,input wire [7:0] data,
  input wire net_clk,net_rst_n,request,input wire [63:0] time_gray,
  input wire [AW-1:0] rd_addr,output reg [15:0] rd_data,
  output reg complete,valid,output reg [63:0] timestamp_us);
    localparam integer LOCAL_AW=AW<13?AW:13,DEPTH=1<<LOCAL_AW;
    localparam integer BANKS=(WIDTH*HEIGHT+DEPTH-1)/DEPTH,SEL_AW=BANKS>1?$clog2(BANKS):1;
    wire [15:0] bank_data[0:BANKS-1];
    reg [SEL_AW-1:0] select0,select1,select2;
    (* ASYNC_REG="TRUE" *) reg req_meta,req_sync;
    (* ASYNC_REG="TRUE" *) reg [63:0] time_meta,time_sync;
    reg completed,frame_valid,vs_d,href_d,capturing,malformed;
    reg [63:0] captured_time;
    reg [12:0] bytes_in_line;
    reg [11:0] lines;
    reg [AW:0] count;
    reg [7:0] high_byte;
    wire store=capturing&&href&&!vsync&&bytes_in_line[0]
               &&bytes_in_line<2*WIDTH&&lines<HEIGHT&&count<WIDTH*HEIGHT;
    // 功能：还原共同微秒时间戳；value为64位Gray计数；返回二进制微秒数。
    function [63:0] decode_gray(input [63:0] value);
        integer j;begin for(j=0;j<64;j=j+1)decode_gray[j]=^(value>>j);end
    endfunction
    genvar b;
    generate for(b=0;b<BANKS;b=b+1)begin:banks
        localparam integer WORDS=(WIDTH*HEIGHT-b*DEPTH)<DEPTH?(WIDTH*HEIGHT-b*DEPTH):DEPTH;
        (* ram_style="block" *) reg [15:0] memory[0:WORDS-1];
        // 保留每组本地地址副本，避免综合合并后重新驱动整帧所有RAM。
        (* syn_preserve=1, syn_keep=1 *) reg [LOCAL_AW-1:0] local_address;
        reg [15:0] read_pipe,bank_pipe;
        always @(posedge pclk)if(store&&(count>>LOCAL_AW)==b)
            memory[count[LOCAL_AW-1:0]]<={high_byte,data};
        always @(posedge net_clk)begin
            local_address<=rd_addr[LOCAL_AW-1:0];
            read_pipe<=memory[local_address];
            bank_pipe<=read_pipe;
        end
        assign bank_data[b]=bank_pipe;
    end endgenerate
    always @(posedge net_clk)begin
        select0<=rd_addr>>LOCAL_AW;select1<=select0;select2<=select1;
        rd_data<=select2<BANKS?bank_data[select2]:16'b0;
    end
    always @(posedge pclk or negedge cam_rst_n)begin
        if(!cam_rst_n)begin
            req_meta<=0;req_sync<=0;time_meta<=0;time_sync<=0;
            completed<=0;frame_valid<=0;captured_time<=0;vs_d<=0;href_d<=0;
            capturing<=0;malformed<=0;bytes_in_line<=0;lines<=0;count<=0;high_byte<=0;
        end else begin
            req_meta<=request;req_sync<=req_meta;time_meta<=time_gray;time_sync<=time_meta;
            vs_d<=vsync;href_d<=href;
            if(!vsync&&vs_d&&!capturing&&req_sync!=completed)begin
                capturing<=1;malformed<=0;bytes_in_line<=0;lines<=0;count<=0;
                captured_time<=decode_gray(time_sync);
            end else if(vsync&&!vs_d&&capturing)begin
                capturing<=0;completed<=req_sync;
                frame_valid<=!malformed&&!href&&!href_d&&lines==HEIGHT&&count==WIDTH*HEIGHT;
            end else if(capturing&&!vsync)begin
                if(href)begin
                    if(!bytes_in_line[0])high_byte<=data;
                    if(bytes_in_line<8191)bytes_in_line<=bytes_in_line+1'b1;else malformed<=1;
                    if(bytes_in_line>=2*WIDTH||lines>=HEIGHT)malformed<=1;
                    if(store)count<=count+1'b1;
                end else if(href_d)begin
                    if(bytes_in_line!=2*WIDTH)malformed<=1;
                    bytes_in_line<=0;if(lines<4095)lines<=lines+1'b1;else malformed<=1;
                end
            end
        end
    end
    (* ASYNC_REG="TRUE" *) reg done_meta,done_sync;
    reg [1:0] settle;
    always @(posedge net_clk or negedge net_rst_n)begin
        if(!net_rst_n)begin done_meta<=0;done_sync<=0;settle<=0;complete<=0;valid<=0;timestamp_us<=0;end
        else begin
            done_meta<=completed;done_sync<=done_meta;settle<={settle[0],done_sync!=complete};
            if(done_sync!=complete&&settle[1])begin complete<=done_sync;valid<=frame_valid;timestamp_us<=captured_time;end
        end
    end
endmodule
