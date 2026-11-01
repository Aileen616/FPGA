// 功能：双时钟三缓冲DVP采集；完整帧在显示消隐时交换，写入不等待上帧显示确认。
// 参数WIDTH、HEIGHT为偶数原图尺寸；每路缓存宽高减半，RGB565高字节先到。
// read_bank及published_bank是稳定的多位握手负载，仅在对应toggle同步后使用。
module dvp_triple_framebuffer #(
    parameter integer WIDTH=320, HEIGHT=240,
    parameter integer PIXELS=WIDTH*HEIGHT/4,
    parameter integer AW=$clog2(PIXELS),
    parameter integer MEM_AW=$clog2(3*PIXELS)
)(input wire pclk,cam_rst_n,vsync,href,input wire [7:0] data,
  input wire rd_clk,rd_rst_n,rd_vblank,input wire [AW-1:0] rd_addr,
  output reg [15:0] rd_data,output reg rd_valid,
  output reg [15:0] frames,bad_frames,displayed,
  output reg [15:0] frames_gray,bad_gray,displayed_gray);
    (* ram_style="block" *) reg [15:0] mem [0:3*PIXELS-1];
    reg request,acknowledge;
    reg [1:0] ack_settle,req_settle;
    reg [1:0] write_bank,published_bank,read_bank;
    (* ASYNC_REG="TRUE" *) reg ack_meta,ack_sync,req_meta,req_sync;
    (* ASYNC_REG="TRUE" *) reg [1:0] read_bank_meta,read_bank_sync,pub_meta,pub_sync;
    reg vs_d,href_d,armed,capture,malformed;
    reg [12:0] bytes_in_line;
    reg [11:0] lines;
    reg [AW:0] count;
    reg [7:0] high_byte;
    reg write_enable;
    reg [MEM_AW-1:0] write_address;
    reg [15:0] write_pixel;
    wire vs_rise=vsync && !vs_d;
    wire vs_fall=!vsync && vs_d;
    wire store_pixel=capture && !vsync && href && bytes_in_line[0]
                     && !bytes_in_line[1] && !lines[0]
                     && bytes_in_line<2*WIDTH && lines<HEIGHT && count<PIXELS;
    // 选择既非当前显示、也非刚写完的第三块；当前帧只能在前次握手完成时发布。
    function [1:0] free_bank(input [1:0] reading,input [1:0] writing);
        begin
            if(reading!=0 && writing!=0) free_bank=0;
            else if(reading!=1 && writing!=1) free_bank=1;
            else free_bank=2;
        end
    endfunction
    // RAM不复位；有效标志在第一次完整帧被显示端接收后才置位。
    always @(posedge pclk) if(write_enable) mem[write_address]<=write_pixel;
    always @(posedge rd_clk) rd_data<=mem[read_bank*PIXELS+rd_addr];
    always @(posedge pclk or negedge cam_rst_n) begin
        if(!cam_rst_n) begin write_enable<=0;write_address<=0;write_pixel<=0;end
        else begin
            write_enable<=store_pixel;
            write_address<=write_bank*PIXELS+count;
            write_pixel<={high_byte,data};
        end
    end
    always @(posedge pclk or negedge cam_rst_n) begin
        if(!cam_rst_n) begin
            ack_meta<=0;ack_sync<=0;ack_settle<=0;read_bank_meta<=0;read_bank_sync<=0;
            request<=0;write_bank<=1;published_bank<=0;
            vs_d<=0;href_d<=0;armed<=0;capture<=0;malformed<=0;
            bytes_in_line<=0;lines<=0;count<=0;high_byte<=0;
            frames<=0;bad_frames<=0;frames_gray<=0;bad_gray<=0;
        end else begin
            ack_meta<=acknowledge;ack_sync<=ack_meta;
            // bank号可以比toggle晚到一个目标周期；再等两拍才允许选择新空闲块。
            ack_settle<={ack_settle[0],ack_sync==request};
            read_bank_meta<=read_bank;read_bank_sync<=read_bank_meta;
            vs_d<=vsync;href_d<=href;
            if(vs_rise) begin
                armed<=1;
                if(capture) begin
                    if(!malformed && !href && !href_d && lines==HEIGHT && count==PIXELS) begin
                        if(ack_sync==request && ack_settle[1]) begin
                            published_bank<=write_bank;
                            request<=~request;
                            write_bank<=free_bank(read_bank_sync,write_bank);
                            frames<=frames+1'b1;
                            frames_gray<=(frames+16'd1)^((frames+16'd1)>>1);
                        end
                        // 消费端尚未确认时丢弃此帧，当前空闲写块继续用于下一帧。
                    end else begin
                        bad_frames<=bad_frames+1'b1;
                        bad_gray<=(bad_frames+16'd1)^((bad_frames+16'd1)>>1);
                    end
                end
                capture<=0;
            end else if(vs_fall) begin
                capture<=armed;
                bytes_in_line<=0;lines<=0;count<=0;malformed<=0;
            end else if(capture && !vsync) begin
                if(href) begin
                    if(!bytes_in_line[0]) high_byte<=data;
                    if(bytes_in_line<8191) bytes_in_line<=bytes_in_line+1'b1;
                    else malformed<=1;
                    if(bytes_in_line>=2*WIDTH || lines>=HEIGHT) malformed<=1;
                    if(store_pixel) count<=count+1'b1;
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
            req_meta<=0;req_sync<=0;req_settle<=0;pub_meta<=0;pub_sync<=0;
            acknowledge<=0;read_bank<=0;rd_valid<=0;
            displayed<=0;displayed_gray<=0;
        end else begin
            req_meta<=request;req_sync<=req_meta;
            // 只在帧号toggle与bank号均稳定后接受，避免多位bank号瞬态组合。
            req_settle<={req_settle[0],req_sync!=acknowledge};
            pub_meta<=published_bank;pub_sync<=pub_meta;
            if(rd_vblank && req_sync!=acknowledge && req_settle[1]) begin
                read_bank<=pub_sync;acknowledge<=req_sync;rd_valid<=1;
                displayed<=displayed+1'b1;
                displayed_gray<=(displayed+16'd1)^((displayed+16'd1)>>1);
            end
        end
    end
endmodule
