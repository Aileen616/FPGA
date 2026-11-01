// 功能：以保持负载的toggle握手传递一个DDR读突发，支持任意读取等待。
// 接口：源端valid/ready接受地址；返回一次result_valid，数据保持至下一结果。
// 两端复位来自同一源；地址保持至返回，响应数据保持至下一请求。
module ddr_read_bridge(input wire src_clk,src_rst_n,input wire valid,output wire ready,
    input wire [28:0] address,output reg result_valid,output reg [255:0] result,
    input wire dst_clk,dst_rst_n,output wire req_valid,input wire req_ready,
    output wire [28:0] req_addr,input wire rsp_valid,input wire [255:0] rsp_data);
    reg request,acknowledge;reg [28:0] held_addr,dst_address;reg [255:0] held_result;
    (* ASYNC_REG="TRUE" *) reg ack_meta,ack_sync,req_meta,req_sync;
    reg busy,accepted;reg [1:0] settle,req_settle;
    assign ready=!busy;
    assign req_addr=dst_address;
    assign req_valid=req_sync!=acknowledge && req_settle[1] && !accepted;
    always @(posedge src_clk or negedge src_rst_n) begin
        if(!src_rst_n) begin request<=0;held_addr<=0;ack_meta<=0;ack_sync<=0;busy<=0;settle<=0;result_valid<=0;result<=0;end
        else begin
            ack_meta<=acknowledge;ack_sync<=ack_meta;result_valid<=0;
            settle<={settle[0],busy && ack_sync==request};
            if(valid && ready) begin held_addr<=address;request<=~request;busy<=1;settle<=0;end
            else if(busy && ack_sync==request && settle[1]) begin result<=held_result;result_valid<=1;busy<=0;settle<=0;end
        end
    end
    always @(posedge dst_clk or negedge dst_rst_n) begin
        if(!dst_rst_n) begin req_meta<=0;req_sync<=0;req_settle<=0;acknowledge<=0;accepted<=0;held_result<=0;dst_address<=0;end
        else begin
            req_meta<=request;req_sync<=req_meta;
            req_settle<={req_settle[0],req_sync!=acknowledge};
            if(req_settle[0] && !accepted)dst_address<=held_addr;
            if(req_valid && req_ready) accepted<=1;
            if(accepted && rsp_valid) begin held_result<=rsp_data;acknowledge<=req_sync;accepted<=0;end
        end
    end
endmodule
