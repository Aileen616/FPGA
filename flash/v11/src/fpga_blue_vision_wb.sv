// 功能：原始DVP蓝色障碍/白地像素统计，冻结跨域快照供本地软核读取；无网络及帧RAM。
// 参数：WIDTH/HEIGHT为原图尺寸且为4倍数；ROI为下采样坐标，默认200x120中的中央底部。
// 接口：RGB565高字节先到；两个复位由同一源异步断言、各自同步释放；只读结果无电机接口。
module fpga_blue_vision_wb #(
    parameter integer WIDTH=800,HEIGHT=480,CLK_HZ=21000000,
    parameter integer X0=50,X1=149,Y0=60,Y1=119,
    parameter integer BLUE_MIN=80,BLUE_R_DELTA=32,BLUE_G_DELTA=16,WHITE_MIN=144,WHITE_SPREAD=48
)(input wire clk,rst_n,pclk,cam_rst_n,vsync,href,input wire [7:0] data,
  input wire cyc,stb,we,input wire [31:0] addr,wdata,input wire [3:0] sel,
  output wire ack,output reg [31:0] rdata);
    wire [31:0] image_rdata;
    endpoint_image_wb #(.WIDTH(WIDTH),.HEIGHT(HEIGHT)) image_capture(
      .clk(clk),.rst_n(rst_n),.pclk(pclk),.cam_rst_n(cam_rst_n),.vsync(vsync),.href(href),.data(data),
      .write_en(cyc&&stb&&we&&sel==4'hf),.addr(addr[11:0]),.wdata(wdata),.rdata(image_rdata));
    reg [31:0] ms,ms_gray,div_count;
    always @(posedge clk or negedge rst_n)
    if(!rst_n)begin ms<=0;ms_gray<=0;div_count<=0;end
    else if(div_count==CLK_HZ/1000-1)begin div_count<=0;ms<=ms+1'b1;ms_gray<=(ms+1)^((ms+1)>>1);end
    else div_count<=div_count+1'b1;
    (* ASYNC_REG="TRUE" *) reg [31:0] time_meta,time_sync;
    (* ASYNC_REG="TRUE" *) reg ack_meta,ack_sync,req_meta,req_sync;
    reg request,acknowledge;reg [1:0] settle;
    reg [223:0] payload; // 采集时间、有效位、蓝色数量、通道蓝/白数量、包围框，握手完成前保持。
    reg [31:0] frame_stamp,objects,occupied,floor_count,samples,glare_count;
    reg [15:0] xmin,ymin,xmax,ymax;
    reg [12:0] bytes_in_line;reg [11:0] lines;
    reg [7:0] high_byte;reg vs_d,href_d,armed,capturing,malformed;
    wire [15:0] pixel={high_byte,data};
    wire [8:0] red={1'b0,pixel[15:11],3'b0},green={1'b0,pixel[10:5],2'b0},blue={1'b0,pixel[4:0],3'b0};
    wire is_blue=blue>=BLUE_MIN&&blue>red+BLUE_R_DELTA&&blue>green+BLUE_G_DELTA;
    wire [8:0] max_rg=red>green?red:green,min_rg=red<green?red:green;
    wire [8:0] max_rgb=max_rg>blue?max_rg:blue,min_rgb=min_rg<blue?min_rg:blue;
    wire is_floor=!is_blue&&min_rgb>=WHITE_MIN&&(max_rgb-min_rgb)<=WHITE_SPREAD;
    wire [15:0] x={6'b0,bytes_in_line[12:3]},y={6'b0,lines[11:2]};
    wire sample=capturing&&href&&!vsync&&bytes_in_line[2:0]==1&&lines[1:0]==0&&bytes_in_line<2*WIDTH&&lines<HEIGHT;
    wire roi=x>=X0&&x<=X1&&y>=Y0&&y<=Y1;
    // 功能：Gray解码；g为32位毫秒Gray值；返回二进制时间，仅用于帧龄计算。
    function [31:0] gray_decode(input [31:0] g);
        integer i;begin for(i=0;i<32;i=i+1)gray_decode[i]=^(g>>i);end
    endfunction
    always @(posedge pclk or negedge cam_rst_n)begin
        if(!cam_rst_n)begin
            time_meta<=0;time_sync<=0;ack_meta<=0;ack_sync<=0;request<=0;payload<=0;
            frame_stamp<=0;objects<=0;occupied<=0;floor_count<=0;samples<=0;glare_count<=0;
            xmin<=16'hffff;ymin<=16'hffff;xmax<=0;ymax<=0;bytes_in_line<=0;lines<=0;high_byte<=0;
            vs_d<=0;href_d<=0;armed<=0;capturing<=0;malformed<=0;
        end else begin
            time_meta<=ms_gray;time_sync<=time_meta;ack_meta<=acknowledge;ack_sync<=ack_meta;
            vs_d<=vsync;href_d<=href;
            if(vsync&&!vs_d)begin
                armed<=1;capturing<=0;
                if(capturing&&request==ack_sync)begin
                    payload<={frame_stamp,{31'b0,(!malformed&&!href&&!href_d&&lines==HEIGHT&&samples==WIDTH*HEIGHT/16&&glare_count<((X1-X0+1)*(Y1-Y0+1)+3)/4)},
                              objects,occupied,floor_count,ymin,xmin,ymax,xmax};
                    request<=!request;
                end
            end else if(!vsync&&vs_d)begin
                capturing<=armed;malformed<=0;bytes_in_line<=0;lines<=0;samples<=0;
                frame_stamp<=gray_decode(time_sync);objects<=0;occupied<=0;floor_count<=0;glare_count<=0;
                xmin<=16'hffff;ymin<=16'hffff;xmax<=0;ymax<=0;
            end else if(capturing&&!vsync)begin
                if(href)begin
                    if(!bytes_in_line[0])high_byte<=data;
                    if(bytes_in_line<8191)bytes_in_line<=bytes_in_line+1'b1;else malformed<=1;
                    if(bytes_in_line>=2*WIDTH||lines>=HEIGHT)malformed<=1;
                    if(sample)begin
                        samples<=samples+1'b1;
                        // 大片三通道饱和不能被解释为可通行白地。
                        if(roi&&min_rgb>=240)glare_count<=glare_count+1'b1;
                        if(is_blue)begin
                            objects<=objects+1'b1;
                            if(x<xmin)xmin<=x;if(x>xmax)xmax<=x;
                            if(y<ymin)ymin<=y;if(y>ymax)ymax<=y;
                            if(roi)occupied<=occupied+1'b1;
                        end
                        if(roi&&is_floor)floor_count<=floor_count+1'b1;
                    end
                end else if(href_d)begin
                    if(bytes_in_line!=2*WIDTH)malformed<=1;
                    bytes_in_line<=0;if(lines<4095)lines<=lines+1'b1;else malformed<=1;
                end
            end
        end
    end
    reg [31:0] frame_seq,source_stamp;reg [191:0] latest;
    reg [255:0] snapshot;
    assign ack=cyc&&stb;
    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)begin req_meta<=0;req_sync<=0;settle<=0;acknowledge<=0;frame_seq<=0;source_stamp<=0;latest<=0;snapshot<=0;end
        else begin
            req_meta<=request;req_sync<=req_meta;settle<={settle[0],req_sync!=acknowledge};
            // 源端保持完整载荷，toggle两级同步后再等待两拍才取总线。
            if(req_sync!=acknowledge&&settle[1])begin
                latest<=payload[191:0];source_stamp<=payload[223:192];frame_seq<=frame_seq+1'b1;acknowledge<=req_sync;
            end
            if(cyc&&stb&&we&&addr[11:0]==12'h004&&sel==4'hf&&wdata==1)
                snapshot<={ms-source_stamp,latest[31:0],latest[63:32],latest[95:64],latest[127:96],latest[159:128],latest[191:160],frame_seq};
        end
    end
    always @*begin
        rdata=image_rdata;
        if(addr[11:0]==0)rdata=32'h46563331;
        else if(addr[11:0]>=16&&addr[11:0]<48)rdata=snapshot[(addr[5:2]-4)*32+:32];
    end
endmodule
