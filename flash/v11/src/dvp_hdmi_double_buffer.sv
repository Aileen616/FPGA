// 功能：偶数行列抽样RGB565双缓冲；只在消隐换帧，发布后等待显示确认再采下一帧，防止撕裂。
// 参数：WIDTH/HEIGHT为DVP原图正整数尺寸，默认800×480，宽高须为偶数；AW为单帧像素地址位宽。
// 接口：DVP高字节先到；rd_addr为0..WIDTH*HEIGHT/4-1；rd_data为一拍同步读结果。
// 两个bank只用单比特toggle交接，无多位异步bank号；背压时允许跳过相机帧。
module dvp_hdmi_double_buffer #(
    parameter integer WIDTH=800, HEIGHT=480, AW=$clog2(WIDTH*HEIGHT/4)
)(input wire pclk,cam_rst_n,vsync,href,input wire [7:0] data,
  input wire rd_clk,rd_rst_n,rd_vblank,input wire [AW-1:0] rd_addr,
  output wire [15:0] rd_data,output reg rd_valid,
  output reg [15:0] frames,bad_frames,displayed,displayed_gray);
    localparam integer PIXELS=WIDTH*HEIGHT/4;
    (* ram_style="block" *) reg [15:0] mem0 [0:PIXELS-1];
    (* ram_style="block" *) reg [15:0] mem1 [0:PIXELS-1];
    reg request,acknowledge,write_bank,read_bank,read_bank_d;
    (* ASYNC_REG="TRUE" *) reg ack_meta,ack_sync,req_meta,req_sync;
    reg [1:0] ack_settle,req_settle;
    reg vs_d,href_d,armed,capture,malformed;
    reg [12:0] bytes_in_line;
    reg [11:0] lines;
    reg [AW:0] count;
    reg [7:0] high_byte;
    reg [15:0] q0,q1;
    wire store=capture && !vsync && href && bytes_in_line[0]
               && !bytes_in_line[1] && !lines[0]
               && bytes_in_line<2*WIDTH && lines<HEIGHT && count<PIXELS;
    // 只写当前空闲bank。RAM不复位，首次完整帧交接前rd_valid始终为0。
    always @(posedge pclk) begin
        if(store && !write_bank) mem0[count[AW-1:0]]<={high_byte,data};
        if(store && write_bank) mem1[count[AW-1:0]]<={high_byte,data};
    end
    always @(posedge rd_clk) begin
        q0<=mem0[rd_addr];q1<=mem1[rd_addr];read_bank_d<=read_bank;
    end
    assign rd_data=read_bank_d?q1:q0;
    always @(posedge pclk or negedge cam_rst_n) begin
        if(!cam_rst_n) begin
            ack_meta<=0;ack_sync<=0;ack_settle<=0;request<=0;write_bank<=1;
            vs_d<=0;href_d<=0;armed<=0;capture<=0;malformed<=0;
            bytes_in_line<=0;lines<=0;count<=0;high_byte<=0;frames<=0;bad_frames<=0;
        end else begin
            ack_meta<=acknowledge;ack_sync<=ack_meta;
            ack_settle<={ack_settle[0],ack_sync==request};
            vs_d<=vsync;href_d<=href;
            if(vsync && !vs_d) begin
                armed<=1;
                if(capture) begin
                    if(!malformed && !href && !href_d && lines==HEIGHT && count==PIXELS) begin
                        // request的新值同时指定发布bank；确认前不再改写任一正在使用的bank。
                        request<=~request;frames<=frames+1'b1;
                    end else bad_frames<=bad_frames+1'b1;
                end
                capture<=0;
            end else if(!vsync && vs_d) begin
                capture<=armed && ack_sync==request && ack_settle[1];
                write_bank<=~ack_sync;
                bytes_in_line<=0;lines<=0;count<=0;malformed<=0;
            end else if(capture && !vsync) begin
                if(href) begin
                    if(!bytes_in_line[0]) high_byte<=data;
                    if(bytes_in_line<8191) bytes_in_line<=bytes_in_line+1'b1;
                    else malformed<=1;
                    if(bytes_in_line>=2*WIDTH || lines>=HEIGHT) malformed<=1;
                    if(store) count<=count+1'b1;
                end else if(href_d) begin
                    if(bytes_in_line!=2*WIDTH) malformed<=1;
                    bytes_in_line<=0;
                    if(lines<4095) lines<=lines+1'b1;else malformed<=1;
                end
            end
        end
    end
    always @(posedge rd_clk or negedge rd_rst_n) begin
        if(!rd_rst_n) begin
            req_meta<=0;req_sync<=0;req_settle<=0;acknowledge<=0;read_bank<=0;
            rd_valid<=0;displayed<=0;displayed_gray<=0;
        end else begin
            req_meta<=request;req_sync<=req_meta;
            req_settle<={req_settle[0],req_sync!=acknowledge};
            if(rd_vblank && req_sync!=acknowledge && req_settle[1]) begin
                read_bank<=req_sync;acknowledge<=req_sync;rd_valid<=1;
                displayed<=displayed+1'b1;
                displayed_gray<=(displayed+16'd1)^((displayed+16'd1)>>1);
            end
        end
    end
endmodule
