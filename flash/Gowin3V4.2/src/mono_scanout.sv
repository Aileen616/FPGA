// 功能：按480×854原生HDMI时序扫描，将相机与文字一起旋转为854×480横向画布。
// 参数：无；像素时钟33.90625MHz，行672拍、场899行，HS/VS均为正极性。
// 输入：clk为像素时钟，rst_n低有效；valid表示完整相机帧有效，done表示相机初始化完成。
// 输入：pixel为上一轮地址的一拍同步RGB565读出值；本模块保留原三拍扫描输出管线。
// 输出：col/row为400×240片内图像坐标；scan_x/scan_y为与RGB/同步同拍的横向逻辑坐标。
// 输出：vblank在原生场消隐开始时单拍有效；fetch_start/fetch_row为未连接的旧DDR接口，固定0。
module mono_scanout(input wire clk,rst_n,valid,done,input wire [15:0] pixel,
    output reg [8:0] col,output reg [7:0] row,output wire vblank,fetch_start,image_active,
    output wire [7:0] fetch_row,output reg hs,vs,de,output reg [7:0] red,green,blue,
    output reg [10:0] scan_x,output reg [9:0] scan_y);
    // 原生坐标用于同步时序；逻辑坐标只在有效区内有显示意义，消隐时不访问越界图像。
    reg [10:0] x;reg [9:0] y;
    wire [10:0] logical_x={1'b0,y};
    wire [9:0] logical_y=10'd479-x[9:0];
    wire active=x<480&&y<854;
    wire area=active&&logical_x>=27&&logical_x<827;
    reg [1:0] active_d,image_d,hs_d,vs_d;reg valid_d;
    reg [10:0] xd,xdd;reg [9:0] yd,ydd;
    assign vblank=x==0&&y==854;
    // 当前显示直接随机读取片内双缓冲，无DDR预取任务，不能沿用旧横向逐行预取信号。
    assign fetch_start=1'b0;
    assign fetch_row=8'd0;
    assign image_active=image_d[0];
    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)begin
            x<=0;y<=0;col<=0;row<=0;active_d<=0;image_d<=0;hs_d<=0;vs_d<=0;valid_d<=0;
            hs<=0;vs<=0;de<=0;red<=0;green<=0;blue<=0;xd<=0;xdd<=0;yd<=0;ydd<=0;scan_x<=0;scan_y<=0;
        end else begin
            if(x==671)begin x<=0;if(y==898)y<=0;else y<=y+1'b1;end else x<=x+1'b1;
            // 横向画布左右各留27像素；源图保持2倍整数放大，不增加缓存或改变相机采集。
            col<=area?(logical_x-27)>>1:0;row<=area?logical_y>>1:0;
            valid_d<=valid;active_d<={active_d[0],active};image_d<={image_d[0],area};
            hs_d<={hs_d[0],x>=496&&x<576};vs_d<={vs_d[0],y>=855&&y<858};
            hs<=hs_d[1];vs<=vs_d[1];de<=active_d[1];
            // 与原RAM读延迟一致地传递逻辑坐标，使OSD和相机采用同一旋转方向。
            xd<=logical_x;xdd<=xd;scan_x<=xdd;yd<=logical_y;ydd<=yd;scan_y<=ydd;
            if(!active_d[1])begin red<=0;green<=0;blue<=0;end
            else if(image_d[1]&&valid_d)begin
                red<={pixel[15:11],pixel[15:13]};green<={pixel[10:5],pixel[10:9]};blue<={pixel[4:0],pixel[4:2]};
            end else begin red<=done?8'h10:8'h40;green<=8'h10;blue<=8'h10;end
        end
    end
endmodule
