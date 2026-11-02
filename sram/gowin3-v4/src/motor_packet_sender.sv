// 功能：电机整合工程专用 BXV1 发送器；基于归档 E3D5，匹配四拍快照读延迟。
// 包头、像素字节顺序及分片规则保持 E3D5 不变，另增加一拍本地读地址等待。
// 参数：WIDTH/HEIGHT像素；PERIOD_CYCLES请求周期；STREAM_ID为部署时生成的会话号。
// 接口：快照请求到发送结束保持不变；时间戳取自VSYNC采集事件，pair_id仅表示候选配对。
module motor_packet_sender #(
    parameter integer WIDTH=320,HEIGHT=240,AW=$clog2(WIDTH*HEIGHT),
    parameter integer PERIOD_CYCLES=12_500_000,
    parameter [31:0] STREAM_ID=32'h20260923
)(input wire clk,rst_n,enabled,output reg request,
  input wire [1:0] complete,valid,input wire [63:0] timestamp0,timestamp1,
  output reg [AW-1:0] rd_addr,input wire [15:0] pixel0,pixel1,
  output reg tx_start,input wire tx_done,tx_req,output reg [7:0] tx_data,
  output reg [15:0] tx_length,output reg [31:0] sent_pairs,dropped_pairs,packet_count,
  output reg [31:0] pair_id,output reg [4:0] state);
    localparam integer FRAME_BYTES=WIDTH*HEIGHT*2,CHUNKS=(FRAME_BYTES+1399)/1400;
    localparam IDLE=0,WAIT_FRAME=1,HEADER=2,ADDR=3,RAM_WAIT=4,LOW_BYTE=5,HIGH_BYTE=6,START=7,SEND=8,GAP=9,PREPARE=10,RAM_LATCH=11,RAM_PIPE=12,RAM_BANK=13,RAM_ADDRESS=14;
    (* ram_style="block" *) reg [7:0] packet [0:2047];
    reg [31:0] timer;
    reg camera;
    reg [15:0] chunk;
    reg [15:0] x,y;
    reg [AW-1:0] row_base;
    reg [10:0] pos,payload_index;
    reg [15:0] chunk_bytes;
    reg [7:0] gap_count;
    reg [15:0] held_pixel;
    reg [63:0] time0,time1;
    reg [319:0] header_shift;
    wire [63:0] selected_time=camera?time1:time0;
    wire [15:0] selected_pixel=camera?pixel1:pixel0;
    wire [319:0] header={32'h42585631,8'd1,{7'b0,camera},8'd1,8'd0,
                        STREAM_ID,pair_id,pair_id,selected_time,16'(WIDTH),16'(HEIGHT),
                        chunk,16'(CHUNKS),32'(FRAME_BYTES)};
    // 厂家payload_req与数据同拍消费；同步RAM提前读下一字节，首字节在包头发送期间预取。
    wire [10:0] payload_address=tx_req?payload_index+11'd1:payload_index;
    always @(posedge clk) tx_data<=packet[payload_address];
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) payload_index<=0;
        else if(tx_start) payload_index<=0;
        else if(tx_req) payload_index<=payload_index+1'b1;
        else if(state!=SEND) payload_index<=0;
    end
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            state<=IDLE;request<=0;timer<=0;camera<=0;chunk<=0;x<=0;y<=0;row_base<=WIDTH*(HEIGHT-1);pos<=0;
            tx_start<=0;tx_length<=0;chunk_bytes<=0;rd_addr<=0;held_pixel<=0;
            gap_count<=0;time0<=0;time1<=0;header_shift<=0;pair_id<=0;sent_pairs<=0;dropped_pairs<=0;packet_count<=0;
        end else begin
            tx_start<=0;
            if(timer<PERIOD_CYCLES-1) timer<=timer+1'b1;
            case(state)
                IDLE: if(enabled && timer==PERIOD_CYCLES-1) begin
                    timer<=0;request<=!request;pair_id<=pair_id+1'b1;state<=WAIT_FRAME;
                end
                WAIT_FRAME: if(complete=={2{request}}) begin
                    if(&valid) begin
                        camera<=0;chunk<=0;x<=0;y<=0;row_base<=WIDTH*(HEIGHT-1);pos<=0;time0<=timestamp0;time1<=timestamp1;
                        chunk_bytes<=FRAME_BYTES<1400?FRAME_BYTES:1400;
                        tx_length<=(FRAME_BYTES<1400?FRAME_BYTES:1400)+40;state<=PREPARE;
                    end else begin dropped_pairs<=dropped_pairs+1'b1;state<=IDLE;end
                end
                PREPARE: begin header_shift<=header;state<=HEADER;end
                HEADER: begin
                    packet[pos]<=header_shift[319:312];header_shift<={header_shift[311:0],8'b0};
                    if(pos==39) begin pos<=40;state<=ADDR;end else pos<=pos+1'b1;
                end
                ADDR: begin
                    // 物理安装上下颠倒；完整原图逐行反向读取，与已验收HDMI显示方向一致。
                    rd_addr<=row_base+x;state<=RAM_WAIT;
                end
                // ADDR 后等待 RAM 内部两级和 bank 选择输出级，再锁存同一地址的数据。
                RAM_WAIT: state<=RAM_ADDRESS;
                RAM_ADDRESS: state<=RAM_PIPE;
                RAM_PIPE: state<=RAM_BANK;
                RAM_BANK: state<=RAM_LATCH;
                RAM_LATCH: begin held_pixel<=selected_pixel;state<=LOW_BYTE;end
                LOW_BYTE: begin
                    packet[pos]<=held_pixel[7:0];
                    pos<=pos+1'b1;state<=HIGH_BYTE;
                end
                HIGH_BYTE: begin
                    packet[pos]<=held_pixel[15:8];pos<=pos+1'b1;
                    if(x==WIDTH-1) begin x<=0;y<=y+1'b1;row_base<=row_base-WIDTH;end else x<=x+1'b1;
                    if(pos==chunk_bytes+39) state<=START;else state<=ADDR;
                end
                START: begin tx_start<=1;state<=SEND;end
                SEND: if(tx_done) begin packet_count<=packet_count+1'b1;gap_count<=0;state<=GAP;end
                GAP: begin
                    // 厂家tx_done早于最后FCS出端口；64拍覆盖流水线和至少12字节帧间隙。
                    if(gap_count==63) begin
                        if(chunk==CHUNKS-1) begin
                            if(camera) begin sent_pairs<=sent_pairs+1'b1;state<=IDLE;end
                            else begin camera<=1;chunk<=0;x<=0;y<=0;row_base<=WIDTH*(HEIGHT-1);pos<=0;
                                chunk_bytes<=FRAME_BYTES<1400?FRAME_BYTES:1400;
                                tx_length<=(FRAME_BYTES<1400?FRAME_BYTES:1400)+40;state<=PREPARE;end
                        end else begin
                            chunk<=chunk+1'b1;pos<=0;
                            if(chunk==CHUNKS-2) begin
                                chunk_bytes<=FRAME_BYTES-(CHUNKS-1)*1400;
                                tx_length<=FRAME_BYTES-(CHUNKS-1)*1400+40;
                            end else begin chunk_bytes<=1400;tx_length<=1440;end
                            state<=PREPARE;
                        end
                    end else gap_count<=gap_count+1'b1;
                end
                default:state<=IDLE;
            endcase
        end
    end
endmodule

