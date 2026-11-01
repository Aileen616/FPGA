// 功能：轮询仲裁八个DDR突发客户端；每次只允许一个未完成事务。
// 参数：N为客户端数量，默认8。地址单位为32位DQ字，BL8每次递增8。
// 接口：请求valid保持至ready；读写完成均返回对应rsp_valid和共享256位数据。
// 命令和写数据分别握手，禁止假定cmd_ready与wr_data_rdy同时有效。
// QoS默认关闭；本候选显式启用。间隔以固定DDR用户时钟计量，与PHY时钟无关。
// DISPLAY_MAX连续优先次数达到上限后让出一次，防止摄像头写入被饿死。
module ddr_native_arbiter #(parameter N=8,QOS_ENABLE=0,DISPLAY_PORT=0,NETWORK_PORT=0,
    NETWORK_GAP=128,DISPLAY_MAX=2)(input wire clk,rst_n,calibrated,
    input wire [N-1:0] req_valid,req_write,output reg [N-1:0] req_ready,
    input wire [N*29-1:0] req_addr,input wire [N*256-1:0] req_data,
    output reg [N-1:0] rsp_valid,output reg [255:0] rsp_data,
    input wire cmd_ready,wr_data_rdy,rd_data_valid,input wire [255:0] rd_data,
    output wire [2:0] cmd,output wire cmd_en,output reg [28:0] addr,
    output reg [255:0] wr_data,output wire wr_data_en,wr_data_end,
    output wire [31:0] wr_data_mask);
    localparam CW=$clog2(N);
    reg [1:0] state;reg [CW-1:0] owner,round_robin,choice;
    reg writing,command_done,data_done;reg found;
    localparam GAP_W=$clog2(NETWORK_GAP+1);
    localparam QUOTA_W=$clog2(DISPLAY_MAX+1);
    reg [GAP_W-1:0] network_cooldown;
    reg [QUOTA_W-1:0] display_streak;
    reg [N-1:0] eligible;
    reg others;
    integer i,k;
    always @* begin
        found=0;choice=round_robin;req_ready=0;
        eligible=req_valid;
        if(QOS_ENABLE && network_cooldown!=0)eligible[NETWORK_PORT]=0;
        others=0;
        for(i=0;i<N;i=i+1)if(i!=DISPLAY_PORT && eligible[i])others=1;
        for(i=0;i<N;i=i+1) begin
            k=round_robin+i;if(k>=N)k=k-N;
            if(!found && eligible[k] &&
               !(QOS_ENABLE && k==DISPLAY_PORT && display_streak==DISPLAY_MAX && others)) begin choice=k;found=1;end
        end
        if(QOS_ENABLE && eligible[DISPLAY_PORT] && (display_streak<DISPLAY_MAX || !others))begin
            choice=DISPLAY_PORT;found=1;
        end
        if(state==0 && calibrated && found) req_ready[choice]=1;
    end
    assign cmd=writing?3'b000:3'b001;
    assign cmd_en=state==1 && !command_done && calibrated;
    assign wr_data_en=state==1 && writing && !data_done && calibrated;
    assign wr_data_end=wr_data_en;
    assign wr_data_mask=0;
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin network_cooldown<=0;display_streak<=0;state<=0;owner<=0;round_robin<=0;writing<=0;command_done<=0;data_done<=0;addr<=0;wr_data<=0;rsp_valid<=0;rsp_data<=0;end
        else begin
            rsp_valid<=0;
            if(network_cooldown!=0)network_cooldown<=network_cooldown-1'b1;
            case(state)
                0: if(calibrated && found) begin
                    if(QOS_ENABLE)begin
                        if(choice==NETWORK_PORT)network_cooldown<=NETWORK_GAP-1;
                        if(choice==DISPLAY_PORT)begin
                            if(display_streak<DISPLAY_MAX)display_streak<=display_streak+1'b1;
                        end else display_streak<=0;
                    end
                    owner<=choice;
                    // 优先显示不能重置普通队列游标，否则每次让行都选中同一个写端口。
                    if(!QOS_ENABLE || choice!=DISPLAY_PORT)
                        round_robin<=choice==N-1?0:choice+1'b1;
                    addr<=req_addr[choice*29+:29];wr_data<=req_data[choice*256+:256];
                    writing<=req_write[choice];command_done<=0;data_done<=0;state<=1;
                end
                1: begin
                    if(cmd_en && cmd_ready) command_done<=1;
                    if(wr_data_en && wr_data_rdy) data_done<=1;
                    // 读数据可能与命令接受同拍返回，必须覆盖该边界情况。
                    if(!writing && rd_data_valid && (command_done || cmd_ready)) begin
                        rsp_valid[owner]<=1;rsp_data<=rd_data;state<=0;
                    end else if((command_done || cmd_ready) && (!writing || data_done || wr_data_rdy)) begin
                        if(writing) begin rsp_valid[owner]<=1;state<=0;end else state<=2;
                    end
                end
                2: if(rd_data_valid) begin rsp_valid[owner]<=1;rsp_data<=rd_data;state<=0;end
                default:state<=0;
            endcase
        end
    end
endmodule
