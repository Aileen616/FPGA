// 功能：网络域单突发像素缓存，使用两级4选1流水线缩短125MHz像素选择路径。
// 参数：BASE单位32位DQ字。rd_addr在等待期间保持；pixel_ready有效时pixel对应当前地址和帧标签。
module ddr_snapshot_reader #(parameter [28:0] BASE=0)(input wire clk,rst_n,frame_tag,
    input wire [16:0] rd_addr,output reg [15:0] pixel,output reg pixel_ready,
    input wire mem_clk,mem_rst_n,output wire req_valid,input wire req_ready,
    output wire [28:0] req_addr,input wire rsp_valid,input wire [255:0] rsp_data);
    reg cached_tag,cached_valid,busy;reg [12:0] cached_addr,held_addr;reg held_tag;
    reg [255:0] cache;wire bridge_ready,result_valid;wire [255:0] result;
    // 每组先按低两位选择一个像素，下一拍按高两位选择组，避免单拍256位动态切片。
    reg [15:0] group_pixel[0:3];reg [1:0] group_select;
    reg [16:0] stage_address;reg stage_tag,stage_valid;integer group_index;
    wire miss=!cached_valid || cached_addr!=rd_addr[16:4] || cached_tag!=frame_tag;
    wire issue=miss && !busy && bridge_ready;
    ddr_read_bridge bridge(clk,rst_n,issue,bridge_ready,BASE+{rd_addr[16:4],3'b0},result_valid,result,
        mem_clk,mem_rst_n,req_valid,req_ready,req_addr,rsp_valid,rsp_data);
    always @(posedge clk or negedge rst_n)
        if(!rst_n)begin
            cached_tag<=0;cached_valid<=0;busy<=0;cached_addr<=0;held_addr<=0;held_tag<=0;
            cache<=0;pixel<=0;pixel_ready<=0;group_select<=0;stage_address<=0;stage_tag<=0;stage_valid<=0;
            for(group_index=0;group_index<4;group_index=group_index+1)group_pixel[group_index]<=0;
        end else begin
            for(group_index=0;group_index<4;group_index=group_index+1)
                group_pixel[group_index]<=cache[group_index*64+rd_addr[1:0]*16+:16];
            group_select<=rd_addr[3:2];stage_address<=rd_addr;stage_tag<=frame_tag;stage_valid<=!miss;
            pixel<=group_pixel[group_select];
            // 地址/帧改变或迟到响应替换缓存时，旧流水线结果必须无效，发送器会继续等待。
            pixel_ready<=stage_valid && stage_address==rd_addr && stage_tag==frame_tag && !miss;
            if(issue)begin busy<=1;held_addr<=rd_addr[16:4];held_tag<=frame_tag;end
            if(result_valid)begin cache<=result;cached_addr<=held_addr;cached_tag<=held_tag;cached_valid<=1;busy<=0;end
        end
endmodule
