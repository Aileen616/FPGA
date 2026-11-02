// 功能：发送带字节异或校验的ST2十六进制遥测；每行开始锁存完整载荷。
// 参数：WIDTH为8的倍数，SYS_HZ/BAUD单位Hz，PERIOD_CYCLES为行间等待周期。
// 接口：payload必须已在clk域稳定；tx为115200/8N1，单向只读遥测。
module stereo_telemetry #(parameter integer WIDTH=768,SYS_HZ=21_000_000,
    BAUD=115200,PERIOD_CYCLES=SYS_HZ/2)(
    input wire clk,rst_n,input wire [WIDTH-1:0] payload,output reg tx);
    localparam integer DIV=(SYS_HZ+BAUD/2)/BAUD,HEX_COUNT=WIDTH/4;
    reg [31:0] period_count;
    reg [15:0] baud_count;
    reg [WIDTH-1:0] saved;
    reg [7:0] checksum;
    reg [9:0] pos;
    reg [3:0] bit_pos;
    reg [9:0] shift;
    reg sending;
    reg [7:0] next_char;
    function [7:0] hexchar(input [3:0] nibble);
        // 功能：半字节转ASCII；参数0..15；返回字符0..9/A..F。
        begin hexchar=(nibble<10)?8'd48+nibble:8'd55+nibble;end
    endfunction
    function [7:0] byte_xor(input [WIDTH-1:0] value);
        // 功能：计算传输校验；参数完整载荷；返回所有字节的异或值。
        integer i;
        begin byte_xor=0;for(i=0;i<WIDTH/8;i=i+1) byte_xor=byte_xor^value[i*8+:8];end
    endfunction
    always @* begin
        if(pos>=4 && pos<4+HEX_COUNT) next_char=hexchar(saved[WIDTH-1-:4]);
        else if(pos==4+HEX_COUNT) next_char="*";
        else if(pos==5+HEX_COUNT) next_char=hexchar(checksum[7:4]);
        else if(pos==6+HEX_COUNT) next_char=hexchar(checksum[3:0]);
        else if(pos==7+HEX_COUNT) next_char=13;
        else if(pos==8+HEX_COUNT) next_char=10;
        else case(pos)
            0:next_char="S";1:next_char="T";2:next_char="2";default:next_char=" ";
        endcase
    end
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            tx<=1;period_count<=0;baud_count<=0;saved<=0;checksum<=0;pos<=0;
            bit_pos<=0;shift<=10'h3ff;sending<=0;
        end else if(!sending) begin
            tx<=1;
            if(period_count==PERIOD_CYCLES-1) begin
                saved<=payload;checksum<=byte_xor(payload);pos<=0;bit_pos<=0;
                baud_count<=0;sending<=1;period_count<=0;
            end else period_count<=period_count+1'b1;
        end else if(baud_count==0) begin
            baud_count<=DIV-1;
            if(bit_pos==0) begin
                shift<={1'b1,next_char,1'b0};tx<=0;bit_pos<=1;
                if(pos>=4 && pos<4+HEX_COUNT) saved<={saved[WIDTH-5:0],4'b0};
            end else if(bit_pos<10) begin
                tx<=shift[1];shift<={1'b1,shift[9:1]};bit_pos<=bit_pos+1'b1;
            end else begin
                bit_pos<=0;
                if(pos==8+HEX_COUNT) begin sending<=0;tx<=1;end
                else pos<=pos+1'b1;
            end
        end else baud_count<=baud_count-1'b1;
    end
endmodule
