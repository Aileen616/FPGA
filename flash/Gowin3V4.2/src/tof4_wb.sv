// 功能：保留原100kHz手动字节接口，增加互斥的整寄存器硬件事务；四路XSHUT和微秒时基保持。
// 20描述符：地址[6:0]、读[8]、长度[20:16]；24寄存器；28写字节；2C写1启动/256中止。
// 2C读：bit0在途、bit1完成、bit2错误；30..40为17字节结果，44为能力标识54464231。
module tof4_wb #(parameter integer CLK_HZ=21000000)(
 input wire clk,rst_n,wb_cyc,wb_stb,wb_we,input wire [31:0] wb_addr,wb_wdata,
 input wire [3:0] wb_sel,output reg wb_ack,output reg [31:0] wb_rdata,
 inout wire scl,sda,inout wire [3:0] xshut
);
 reg [31:0] us;integer divider;
 reg [3:0] released;
 reg [4:0] command;
 reg [7:0] tx,rx;
 reg busy,done,nack,error,abort;
 reg [31:0] descriptor;reg [15:0] register_address;reg [7:0] write_value;
 reg burst_owned,burst_go,burst_done,burst_error,finish_pending;
 reg [31:0] burst_since;reg [135:0] burst_result;
 wire transaction_busy,transaction_done,transaction_error,transaction_abort;
 wire [135:0] transaction_result;
 wire [4:0] transaction_cmd;wire [7:0] transaction_tx;
 wire byte_done,byte_nack,bus_busy,arbitration;
 wire [7:0] byte_rx;wire scl_release,sda_release;
 wire [4:0] selected_cmd=burst_owned?transaction_cmd:command;
 wire [7:0] selected_tx=burst_owned?transaction_tx:tx;
 wire occupied=busy||burst_owned;
 assign scl=scl_release?1'bz:1'b0;
 assign sda=sda_release?1'bz:1'b0;
 for(genvar i=0;i<4;i=i+1)begin:shut
   assign xshut[i]=released[i]?1'bz:1'b0;
 end
 tof_register_fsm transfer(.clk(clk),.rst_n(rst_n),.enable(burst_owned&&!abort),.go(burst_go),.now_us(us),
   .address(descriptor[6:0]),.reg_addr(register_address),.read_op(descriptor[8]),.size(descriptor[20:16]),.value(write_value),
   .busy(transaction_busy),.done(transaction_done),.error(transaction_error),.result(transaction_result),
   .byte_cmd(transaction_cmd),.byte_tx(transaction_tx),.abort(transaction_abort),
   .byte_done(byte_done),.byte_nack(byte_nack),.arbitration(arbitration),.byte_rx(byte_rx));
 i2c_master_byte_ctrl engine(.clk(clk),.rst(abort||(burst_owned&&transaction_abort)),.nReset(rst_n),.ena(1'b1),
   .clk_cnt(16'(CLK_HZ/500000-1)),.start(selected_cmd[0]),.stop(selected_cmd[1]),
   .read(selected_cmd[2]),.write(selected_cmd[3]),.ack_in(selected_cmd[4]),.din(selected_tx),
   .cmd_ack(byte_done),.ack_out(byte_nack),.dout(byte_rx),.i2c_busy(bus_busy),.i2c_al(arbitration),
   .scl_i(scl),.scl_o(),.scl_oen(scl_release),.sda_i(sda),.sda_o(),.sda_oen(sda_release));
 always @(posedge clk or negedge rst_n)begin
   if(!rst_n)begin
     us<=0;divider<=0;released<=0;command<=0;tx<=0;rx<=0;busy<=0;
     done<=0;nack<=0;error<=0;abort<=0;wb_ack<=0;wb_rdata<=0;
     descriptor<=0;register_address<=0;write_value<=0;
     burst_owned<=0;burst_go<=0;burst_done<=0;burst_error<=0;finish_pending<=0;burst_since<=0;burst_result<=0;
   end else begin
     abort<=0;burst_go<=0;
     if(divider==CLK_HZ/1000000-1)begin divider<=0;us<=us+1'b1;end
     else divider<=divider+1;
     if(byte_done&&busy&&!burst_owned)begin command<=0;busy<=0;done<=1;nack<=byte_nack;rx<=byte_rx;end
     if(arbitration&&busy&&!burst_owned)begin command<=0;busy<=0;done<=1;error<=1;end
     // FSM在末字节STOP完成后才给done；再等待滤波后的总线空闲，最后释放所有权。
     if(burst_owned&&transaction_done)begin
       if(transaction_error)begin burst_owned<=0;burst_done<=1;burst_error<=1;burst_result<=0;abort<=1;finish_pending<=0;end
       else begin burst_result<=transaction_result;finish_pending<=1;end
     end
     if(burst_owned&&finish_pending&&!bus_busy&&scl&&sda)begin burst_owned<=0;burst_done<=1;finish_pending<=0;end
     // 覆盖go交接和STOP收尾阶段，防止FSM已经结束后遗留永久busy。
     if(burst_owned&&(us-burst_since)>=20000)begin
       burst_owned<=0;burst_done<=1;burst_error<=1;burst_result<=0;finish_pending<=0;abort<=1;
     end
     if(!wb_cyc||!wb_stb)wb_ack<=0;
     else if(!wb_ack)begin
       wb_ack<=1;wb_rdata<=0;
       if(wb_addr[31:8]==24'h200050&&wb_addr[1:0]==0&&(!wb_we||wb_sel==15))begin
         if(wb_we)case(wb_addr[7:0])
           8'h0c:released<=wb_wdata[3:0];
           8'h14:if(!occupied&&!abort)tx<=wb_wdata[7:0];else error<=1;
           8'h10:if(wb_wdata==256)begin
             abort<=1;command<=0;busy<=0;done<=0;nack<=0;error<=0;
             burst_owned<=0;burst_go<=0;burst_done<=0;burst_error<=0;finish_pending<=0;burst_result<=0;
           end else if(!occupied&&!abort&&wb_wdata[4:0]!=0)begin command<=wb_wdata[4:0];busy<=1;done<=0;nack<=0;error<=0;end
             else error<=1;
           8'h20:if(!occupied&&!abort)descriptor<=wb_wdata;else burst_error<=1;
           8'h24:if(!occupied&&!abort)register_address<=wb_wdata[15:0];else burst_error<=1;
           8'h28:if(!occupied&&!abort)write_value<=wb_wdata[7:0];else burst_error<=1;
           8'h2c:if(wb_wdata==256)begin
             abort<=1;command<=0;busy<=0;done<=0;nack<=0;error<=0;
             burst_owned<=0;burst_go<=0;burst_done<=0;burst_error<=0;finish_pending<=0;burst_result<=0;
           end else if(wb_wdata==1&&!occupied&&!abort&&!bus_busy&&scl&&sda)begin
             // 请求接受的同一拍即锁定busy并清历史完成，不能让CPU读到旧done。
             burst_owned<=1;burst_go<=1;burst_done<=0;burst_error<=0;finish_pending<=0;burst_result<=0;burst_since<=us;
             command<=0;done<=0;nack<=0;
           end else begin burst_error<=1;if(!burst_owned)begin burst_done<=1;burst_result<=0;end end
           default:error<=1;
         endcase
         else case(wb_addr[7:0])
           0:wb_rdata<=32'h544f4601;
           4:wb_rdata<={24'b0,sda,scl,bus_busy,1'b0,error,nack,done,occupied};
           8:wb_rdata<=us;
           12:wb_rdata<={28'b0,released};
           24:wb_rdata<={24'b0,rx};
           8'h20:wb_rdata<=descriptor;
           8'h24:wb_rdata<={16'b0,register_address};
           8'h28:wb_rdata<={24'b0,write_value};
           8'h2c:wb_rdata<={29'b0,burst_error,burst_done,burst_owned};
           8'h30:wb_rdata<=burst_result[31:0];
           8'h34:wb_rdata<=burst_result[63:32];
           8'h38:wb_rdata<=burst_result[95:64];
           8'h3c:wb_rdata<=burst_result[127:96];
           8'h40:wb_rdata<={24'b0,burst_result[135:128]};
           8'h44:wb_rdata<=32'h54464231;
           default:wb_rdata<=0;
         endcase
       end
     end
   end
 end
endmodule
