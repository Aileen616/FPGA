// 功能：一次完整VL53L1X寄存器读写，复用I2C字节核；不依赖CPU推进字节。
// 输入：go单拍、地址7位、寄存器16位、读1..17字节或写1字节；开始时锁定参数。
// 输出：done单拍，error表示NACK/仲裁/超时；结果低字节对应第一个寄存器。
// byte_cmd沿用START/STOP/READ/WRITE/NACK位，abort复位底层并释放总线。
module tof_register_fsm(
 input wire clk,rst_n,enable,go,input wire [31:0] now_us,
 input wire [6:0] address,input wire [15:0] reg_addr,input wire read_op,
 input wire [4:0] size,input wire [7:0] value,
 output reg busy,done,error,output reg [135:0] result,
 output reg [4:0] byte_cmd,output reg [7:0] byte_tx,output reg abort,
 input wire byte_done,byte_nack,arbitration,input wire [7:0] byte_rx
);
 reg [6:0] addr_q;reg [15:0] reg_q;reg read_q;reg [4:0] size_q,index;
 reg [7:0] value_q;reg [2:0] phase;reg waiting,gap;
 reg [31:0] total_start,byte_start;
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin busy<=0;done<=0;error<=0;result<=0;byte_cmd<=0;byte_tx<=0;abort<=0;
   addr_q<=0;reg_q<=0;read_q<=0;size_q<=0;index<=0;value_q<=0;phase<=0;waiting<=0;gap<=0;total_start<=0;byte_start<=0;end
  else begin
   done<=0;abort<=0;
   if(!enable)begin busy<=0;byte_cmd<=0;waiting<=0;gap<=0;end
   else if(!busy)begin
    byte_cmd<=0;
    if(go)begin
     if(size==0||size>17||(!read_op&&size!=1)||address<8||address>119)begin done<=1;error<=1;end
     else begin busy<=1;error<=0;result<=0;addr_q<=address;reg_q<=reg_addr;read_q<=read_op;size_q<=size;
      value_q<=value;index<=0;phase<=0;waiting<=0;gap<=0;total_start<=now_us;end
    end
   end else if(arbitration||(now_us-total_start)>=20000||(waiting&&(now_us-byte_start)>=2000))begin
    error<=1;done<=1;busy<=0;byte_cmd<=0;abort<=1;waiting<=0;
   end else if(waiting)begin
    if(byte_done)begin
     byte_cmd<=0;waiting<=0;gap<=1;
     if(byte_cmd[3]&&byte_nack)begin error<=1;done<=1;busy<=0;abort<=1;end
     else if(phase==4)begin
      if(read_q)result[index*8+:8]<=byte_rx;
      if(index+1==size_q)begin busy<=0;done<=1;end
      else index<=index+1'b1;
     end else if(phase==2&&!read_q)phase<=4;
     else phase<=phase+1'b1;
    end
   end else if(gap)gap<=0;
   else begin
    waiting<=1;byte_start<=now_us;
    case(phase)
     0:begin byte_cmd<=9;byte_tx<={addr_q,1'b0};end
     1:begin byte_cmd<=8;byte_tx<=reg_q[15:8];end
     2:begin byte_cmd<=8;byte_tx<=reg_q[7:0];end
     3:begin byte_cmd<=9;byte_tx<={addr_q,1'b1};end
     4:begin byte_cmd<=read_q?((index+1==size_q)?22:4):10;byte_tx<=value_q;end
     default:begin error<=1;done<=1;busy<=0;byte_cmd<=0;abort<=1;end
    endcase
   end
  end
 end
endmodule
