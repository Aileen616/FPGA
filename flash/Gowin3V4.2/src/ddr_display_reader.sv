// 功能：显示域保留两行400像素缓存；帧只在消隐更换，提前两条扫描行读取下一行。
// 参数：BASE和STRIDE单位32位DQ字，显示源400×240 RGB565。
// 接口：fetch_start/fetch_row在扫描器产生；col,row为当前扫描源坐标。
// DDR超时不会暂停扫描，只将当前未准备好的图像行判为无效并置underflow。
module ddr_display_reader #(parameter [28:0] BASE=29'd131072,STRIDE=29'd65536)(
    input wire clk,rst_n,vblank,fetch_start,input wire [7:0] fetch_row,row,
    input wire [8:0] col,input wire published,input wire [1:0] published_bank,
    output reg acknowledge,output reg [1:0] read_bank,
    output reg [15:0] pixel,output wire pixel_valid,output reg underflow,input wire image_active,
    input wire mem_clk,mem_rst_n,output wire req_valid,input wire req_ready,
    output wire [28:0] req_addr,input wire rsp_valid,input wire [255:0] rsp_data,output wire miss_event);
    (* ASYNC_REG="TRUE" *) reg pub_meta,pub_sync;reg [1:0] settle;
    reg image_valid,busy,load_bank;reg [7:0] load_row;
    reg [4:0] burst_index;reg [1:0] state;
    reg [1:0] line_valid;reg [7:0] row0,row1;reg [1:0] bank0,bank1;
    (* ram_style="block" *) reg [15:0] line_memory[0:799];
    wire hit0=line_valid[0] && row0==row && bank0==read_bank;
    wire hit1=line_valid[1] && row1==row && bank1==read_bank;
    assign pixel_valid=image_valid && (hit0 || hit1);
    wire bridge_ready,result_valid;wire [255:0] result;
    reg [255:0] held_result;reg [3:0] unpack_index;
    // 只读事件旁路，保留原欠载锁存及数据路径。
    assign miss_event=(image_valid && image_active && !pixel_valid) ||
                      (fetch_start && image_valid && state!=0);
    wire issue=state==1 && bridge_ready;
    wire [28:0] address=BASE+read_bank*STRIDE+load_row*29'd200+{burst_index,3'b0};
    ddr_read_bridge bridge(clk,rst_n,issue,bridge_ready,address,result_valid,result,
        mem_clk,mem_rst_n,req_valid,req_ready,req_addr,rsp_valid,rsp_data);
    always @(posedge clk)begin
        pixel<=line_memory[(hit1?10'd400:10'd0)+col];
        if(state==3)line_memory[(load_bank?10'd400:10'd0)+burst_index*16+unpack_index]<=held_result[unpack_index*16+:16];
    end
    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)begin
            pub_meta<=0;pub_sync<=0;settle<=0;acknowledge<=0;read_bank<=0;image_valid<=0;
            busy<=0;load_bank<=0;load_row<=0;burst_index<=0;state<=0;line_valid<=0;
            row0<=0;row1<=0;bank0<=0;bank1<=0;underflow<=0;held_result<=0;unpack_index<=0;
        end else begin
            pub_meta<=published;pub_sync<=pub_meta;settle<={settle[0],pub_sync!=acknowledge};
            if(image_valid && image_active && !pixel_valid)underflow<=1;
            // 未结束的行读取期间不释放旧帧；极端等待最多推迟一次换帧。
            if(vblank && state==0)begin
                if(pub_sync!=acknowledge && settle[1])begin read_bank<=published_bank;acknowledge<=pub_sync;image_valid<=1;end
                if(image_valid || (pub_sync!=acknowledge && settle[1]))begin
                    load_row<=0;load_bank<=0;line_valid<=0;burst_index<=0;state<=1;
                end
            end else if(fetch_start && image_valid)begin
                if(state==0)begin
                    load_row<=fetch_row;load_bank<=~load_bank;burst_index<=0;state<=1;
                    if(load_bank)line_valid[0]<=0;else line_valid[1]<=0;
                end else underflow<=1;
            end
            case(state)
                1:if(issue)state<=2;
                2:if(result_valid)begin held_result<=result;unpack_index<=0;state<=3;end
                3:begin
                    if(unpack_index==15)begin
                        if(burst_index==24)begin
                            state<=0;
                            if(load_bank)begin row1<=load_row;bank1<=read_bank;line_valid[1]<=1;end
                            else begin row0<=load_row;bank0<=read_bank;line_valid[0]<=1;end
                        end else begin burst_index<=burst_index+1'b1;state<=1;end
                    end else unpack_index<=unpack_index+1'b1;
                end
            endcase
        end
    end
endmodule
