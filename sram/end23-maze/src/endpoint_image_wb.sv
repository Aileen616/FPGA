// 功能：请求后冻结下一完整DVP帧的1/8彩色缩略图，仅用于停机采样。
// 参数：WIDTH/HEIGHT必须为8的倍数，默认800x480；无运动接口。
// 寄存器：100标识/写1请求，104就绪/忙，108像素索引，10c像素，110帧序号。
module endpoint_image_wb #(parameter WIDTH=800,HEIGHT=480)(
 input wire clk,rst_n,pclk,cam_rst_n,vsync,href,input wire [7:0] data,
 input wire write_en,input wire [11:0] addr,input wire [31:0] wdata,output reg [31:0] rdata);
 localparam N=WIDTH*HEIGHT/64;
 reg [15:0] memory[0:N-1]; // 完整帧完成后冻结，CPU读期间不覆盖。
 reg request,done,ready,busy;
 (* ASYNC_REG="TRUE" *) reg rq1,rq2,dn1,dn2;
 reg [1:0] settle;
 reg [31:0] frame_sequence;
 reg [12:0] index;
 reg [15:0] read_pixel;
 reg vs_d,href_d,capturing,malformed;
 reg [12:0] byte_count,count;
 reg [11:0] lines;
 reg [7:0] high_byte;
 wire sample=capturing&&href&&!vsync&&byte_count[3:0]==1&&lines[2:0]==0&&byte_count<2*WIDTH&&lines<HEIGHT&&count<N;
 // 双时钟RAM：采集域写、CPU域同步读；不复位RAM内容，未就绪时不对外发布。
 always @(posedge pclk) if(sample) memory[count]<={high_byte,data};
 always @(posedge clk) if(index<N) read_pixel<=memory[index]; else read_pixel<=0;
 always @(posedge pclk or negedge cam_rst_n)begin
  if(!cam_rst_n)begin rq1<=0;rq2<=0;done<=0;vs_d<=0;href_d<=0;capturing<=0;malformed<=0;byte_count<=0;lines<=0;count<=0;high_byte<=0;end
  else begin
   rq1<=request;rq2<=rq1;vs_d<=vsync;href_d<=href;
   if(vsync&&!vs_d)begin
    if(capturing&&!malformed&&!href&&!href_d&&lines==HEIGHT&&count==N)done<=rq2;
    capturing<=0;
   end else if(!vsync&&vs_d)begin
    capturing<=rq2!=done;malformed<=0;byte_count<=0;lines<=0;count<=0;
   end else if(capturing&&!vsync)begin
    if(href)begin
     if(!byte_count[0])high_byte<=data;
     if(byte_count<8191)byte_count<=byte_count+1'b1;else malformed<=1;
     if(byte_count>=2*WIDTH||lines>=HEIGHT)malformed<=1;
     if(sample)count<=count+1'b1;
    end else if(href_d)begin
     if(byte_count!=2*WIDTH)malformed<=1;
     byte_count<=0;if(lines<4095)lines<=lines+1'b1;else malformed<=1;
    end
   end
  end
 end
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin request<=0;ready<=0;busy<=0;dn1<=0;dn2<=0;settle<=0;frame_sequence<=0;index<=0;end
  else begin
   dn1<=done;dn2<=dn1;settle<={settle[0],busy&&dn2==request};
   if(busy&&dn2==request&&settle[1])begin busy<=0;ready<=1;frame_sequence<=frame_sequence+1'b1;end
   if(write_en&&addr==12'h100&&wdata==1&&!busy)begin request<=!request;ready<=0;busy<=1;settle<=0;end
   if(write_en&&addr==12'h108)index<=wdata[12:0];
  end
 end
 always @*begin
  rdata=0;
  case(addr)
   12'h100:rdata=32'h494d4731;
   12'h104:rdata={30'b0,busy,ready};
   12'h108:rdata={19'b0,index};
   12'h10c:rdata=ready?{16'b0,read_pixel}:0;
   12'h110:rdata=frame_sequence;
  endcase
 end
endmodule

