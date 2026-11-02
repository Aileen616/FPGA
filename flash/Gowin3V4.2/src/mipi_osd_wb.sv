// 功能：为裸机提供双页文字和只读显示诊断；地址0x20006000，数据写逐字确认。
// 0x00 ID，0x04 publish/ack，0x08相机状态，0x0C显示状态；0x400..0x7FC双页文字。
// publish位同时指定页；未收到上一页确认时拒绝发布与写入，防止屏幕读写同页。
module mipi_osd_wb(input wire clk,rst_n,cyc,stb,we,input wire [31:0] addr,wdata,
    input wire [3:0] sel,output reg ack,output reg [31:0] rdata,
    output reg text_we,output reg text_bank,output reg [6:0] text_addr,
    output reg [31:0] text_data,output reg publish,input wire displayed,
    input wire [31:0] camera_status,display_status,input wire [511:0] video_diag);
    reg [511:0] diag_snapshot; // 控制域一致快照，读取不清计数或刷新运动租约。
    (* ASYNC_REG="TRUE" *) reg ack_meta,ack_sync;
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin diag_snapshot<=0;ack<=0;rdata<=0;text_we<=0;text_bank<=0;text_addr<=0;text_data<=0;publish<=0;ack_meta<=0;ack_sync<=0;end
        else begin
            ack_meta<=displayed;ack_sync<=ack_meta;ack<=0;text_we<=0;
            if(cyc&&stb&&!ack) begin
                ack<=1;rdata<=0;
                if(we&&sel==15) begin
                    if(addr[11:0]==12'h010 && wdata==1)diag_snapshot<=video_diag;
                    if(addr[11:0]==4 && publish==ack_sync) publish<=wdata[0];
                    if(addr[11:10]==1 && publish==ack_sync && addr[9]!=publish) begin
                        text_we<=1;text_bank<=addr[9];text_addr<=addr[8:2];text_data<=wdata;
                    end
                end else if(!we) case(addr[11:0])
                    0:rdata<=32'h4d495031;
                    4:rdata<={30'b0,ack_sync,publish};
                    8:rdata<=camera_status;
                    12:rdata<=display_status;
                    default:if(addr[11:6]==0 && addr[5] && addr[1:0]==0)rdata<=diag_snapshot[{addr[4:2],5'b0}+:32];else if(addr[11:5]==2 && addr[1:0]==0)rdata<=diag_snapshot[256+{addr[4:2],5'b0}+:32];else rdata<=0;
                endcase
            end
        end
    end
endmodule
