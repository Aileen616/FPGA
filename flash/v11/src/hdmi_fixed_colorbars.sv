// 功能：连续输出600p八色竖条，不接收相机、缓存、OSD或网络数据。
// 接口：clk为现有像素时钟；rst_n异步低有效、须同步释放；RGB与HS/VS/DE同拍。
// 时序：有效1280×600，总1650×625，正极性HS、负极性VS 1068..1429、VS 603..729。
module hdmi_fixed_colorbars(
    input wire clk,rst_n,
    output reg hs,vs,de,
    output reg [7:0] red,green,blue,
    output reg [10:0] scan_x,output reg [9:0] scan_y,
    output wire vblank,output reg published
);
    reg [10:0] x;
    reg [9:0] y;
    reg [23:0] color;
    assign vblank=scan_x==0 && scan_y==600;
    // 每条160像素：白、黄、青、绿、品红、红、蓝、黑。
    always @* begin
        if(x<128)color=24'hffffff;
        else if(x<256)color=24'hffff00;
        else if(x<384)color=24'h00ffff;
        else if(x<512)color=24'h00ff00;
        else if(x<640)color=24'hff00ff;
        else if(x<768)color=24'hff0000;
        else if(x<896)color=24'h0000ff;
        else color=24'h000000;
    end
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n)begin
            x<=0;y<=0;hs<=0;vs<=0;de<=0;
            red<=0;green<=0;blue<=0;scan_x<=0;scan_y<=0;published<=0;
        end else begin
            if(x==1343)begin x<=0;if(y==624)y<=0;else y<=y+1'b1;end
            else x<=x+1'b1;
            scan_x<=x;scan_y<=y;
            hs<=x>=1068 && x<1156;vs<=!(y>=603 && y<609);de<=x<1024 && y<600;
            {red,green,blue}<=(x<1024 && y<600)?color:24'b0;
            if(x==0 && y==600)published<=~published;
        end
    end
endmodule
