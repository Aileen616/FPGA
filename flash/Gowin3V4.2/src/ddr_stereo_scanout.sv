// 功能：输出1024×600扫描时序，两个160×120 RGB565源各放大3倍，保持4:3比例。
// 时序来源：香橙派Xorg实读DWE/8448屏幕EDID，1344×625总周期，+HS/-VS。
// 输入clk为像素时钟、rst_n低有效；valid/done须处于像素域，pixel为一拍同步RAM输出。
// 输出col=0..159,row=0..119；消隐交换DDR帧，fetch_start提前读取下一源行。
module ddr_stereo_scanout(
    input wire clk,rst_n,input wire [1:0] valid,done,
    input wire [15:0] pixel0,pixel1,
    output reg [7:0] col,output reg [6:0] row,output wire vblank,fetch_start,image_active,output wire [6:0] fetch_row,
    output reg hs,vs,de,output reg [7:0] red,green,blue);
    reg [10:0] x;
    reg [9:0] y;
    wire right=x>=512;
    wire image_area=x>=32 && x<992 && y>=120 && y<480;
    wire [9:0] local_x=right ? x-11'd512 : x-11'd32;
    wire [8:0] local_y=y-10'd120;
    reg [1:0] valid_d;
    reg [1:0] active_d,image_d,right_d,hs_d,vs_d,strip_d;
    wire [15:0] pixel=right_d[1]?pixel1:pixel0;
    wire selected_valid=right_d[1]?valid_d[1]:valid_d[0];
    wire selected_done=right_d[1]?done[1]:done[0];
    assign vblank=x==0 && y==600;
    assign image_active=image_d[0];
    // 第二次重复显示时预取下一行，为DDR争用留出至少一个完整扫描行。
    assign fetch_start=x==0 && y>=121 && y<477 && (y-120)%3==1;
    assign fetch_row=118-(y-120)/3;
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            x<=0;y<=0;col<=0;row<=0;valid_d<=0;active_d<=0;image_d<=0;right_d<=0;
            hs_d<=0;vs_d<=0;strip_d<=0;hs<=0;vs<=0;de<=0;red<=0;green<=0;blue<=0;
        end else begin
            if(x==1343) begin x<=0;if(y==624)y<=0;else y<=y+1'b1;end
            else x<=x+1'b1;
            // 与现有相机安装方向一致：只翻转显示行地址，不改网络原图或相机寄存器。
            col<=image_area ? local_x/3 : 0;row<=image_area ? 7'd119-local_y/3 : 0;
            valid_d<=valid;
            active_d<={active_d[0],x<1024 && y<600};
            image_d<={image_d[0],image_area};right_d<={right_d[0],right};
            hs_d<={hs_d[0],x>=1068 && x<1156};
            vs_d<={vs_d[0],!(y>=603 && y<609)};
            strip_d<={strip_d[0],y<8};
            hs<=hs_d[1];vs<=vs_d[1];de<=active_d[1];
            if(!active_d[1]) begin red<=0;green<=0;blue<=0;end
            else if(strip_d[1]) begin
                red<=selected_valid?0:8'hff;
                green<=selected_done?8'hff:0;blue<=selected_valid?8'hff:0;
            end else if(image_d[1] && selected_valid) begin
                red<={pixel[15:11],pixel[15:13]};green<={pixel[10:5],pixel[10:9]};blue<={pixel[4:0],pixel[4:2]};
            end else if(image_d[1] && !selected_valid) begin
                // 尚无完整相机帧显示暗红区域，避免把测试图误当成相机画面。
                red<=8'h40;green<=0;blue<=0;
            end else begin red<=8'h10;green<=8'h10;blue<=8'h10;end
        end
    end
endmodule
