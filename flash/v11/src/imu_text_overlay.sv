// 功能：双页ASCII字符OSD；系统域写不可见页，像素域仅在消隐交换，确认后才可复用旧页。
// 接口：text_addr为32位字偏移0..127，低字节先显示；publish/displayed为翻转握手。
// 输入坐标为已旋转的854×480逻辑坐标，与RGB/HS/VS/DE同拍；输出四级寄存，字符像素2倍放大。
// 复位须同源异步置位、分别同步释放；新页20帧未到时遮蔽旧值，新扫描下约356.4ms。
module imu_text_overlay(
    input wire sys_clk,sys_rst_n,text_we,text_bank,input wire [6:0] text_addr,
    input wire [31:0] text_data,input wire publish,
    input wire pixel_clk,pixel_rst_n,vblank,input wire [10:0] x,input wire [9:0] y,
    input wire hs_in,vs_in,de_in,input wire [7:0] red_in,green_in,blue_in,
    output reg hs,vs,de,output reg [7:0] red,green,blue,output reg displayed
);
    reg [31:0] text_mem [0:255]; /* 双时钟简单双口RAM，无复位清零以保留BSRAM推断。 */
    always @(posedge sys_clk) if(sys_rst_n && text_we) text_mem[{text_bank,text_addr}]<=text_data;
    (* ASYNC_REG="TRUE" *) reg req_meta,req_sync;
    reg bank,have_page;
    reg [5:0] age;
    always @(posedge pixel_clk or negedge pixel_rst_n) begin
        if(!pixel_rst_n) begin req_meta<=0;req_sync<=0;bank<=0;displayed<=0;have_page<=0;age<=20;end
        else begin
            req_meta<=publish;req_sync<=req_meta;
            if(vblank) begin
                if(req_sync!=displayed) begin bank<=req_sync;displayed<=req_sync;have_page<=1;age<=0;end
                else if(age<20) age<=age+1'b1;
            end
        end
    end
    wire inside_panel=x>=16 && x<528 && y>=256 && y<464;
    wire [10:0] dx=x-11'd16;
    wire [9:0] dy=y-10'd256;
    wire [8:0] character_addr={dy[7:4],dx[8:4]};
    reg [31:0] word_q;
    reg [1:0] byte_q;
    reg [2:0] row_q,col_q;
    reg [4:0] char_col_q;
    reg [3:0] char_row_q;
    reg panel_q,timeout_q;
    reg hs_q,vs_q,de_q;
    reg [7:0] r_q,g_q,b_q;
    always @(posedge pixel_clk) word_q<=text_mem[{bank,character_addr[8:2]}];
    // 功能：8x8英文字库；c为ASCII、row为0..7；返回该行8个像素，MSB在左。
    function automatic [7:0] glyph(input [7:0] c,input [2:0] row);
        reg [63:0] bits;
        begin
            case(c)
                "0":bits=64'h1c22262a32221c00; "1":bits=64'h0818080808081c00;
                "2":bits=64'h1c22020408103e00; "3":bits=64'h3e04080c02221c00;
                "4":bits=64'h040c14243e040400; "5":bits=64'h3e203c0202221c00;
                "6":bits=64'h0c10203c22221c00; "7":bits=64'h3e02040810101000;
                "8":bits=64'h1c22221c22221c00; "9":bits=64'h1c22221e02041800;
                "A":bits=64'h1c22223e22222200; "B":bits=64'h3c22223c22223c00;
                "C":bits=64'h1c22202020221c00; "D":bits=64'h3c22222222223c00;
                "E":bits=64'h3e20203c20203e00; "F":bits=64'h3e20203c20202000;
                "G":bits=64'h1c22202e22221e00; "H":bits=64'h2222223e22222200;
                "I":bits=64'h1c08080808081c00; "J":bits=64'h0e04040424241800;
                "K":bits=64'h2224283028242200; "L":bits=64'h2020202020203e00;
                "M":bits=64'h22362a2a22222200; "N":bits=64'h22322a2622222200;
                "O":bits=64'h1c22222222221c00; "P":bits=64'h3c22223c20202000;
                "Q":bits=64'h1c2222222a241a00; "R":bits=64'h3c22223c28242200;
                "S":bits=64'h1e20201c02023c00; "T":bits=64'h3e08080808080800;
                "U":bits=64'h2222222222221c00; "V":bits=64'h2222222222140800;
                "W":bits=64'h2222222a2a362200; "X":bits=64'h2222140814222200;
                "Y":bits=64'h2222140808080800; "Z":bits=64'h3e02040810203e00;
                "+":bits=64'h0008083e08080000; "-":bits=64'h0000003e00000000;
                ".":bits=64'h0000000000181800; ":":bits=64'h0018180018180000;
                "/":bits=64'h0202040810202000;
                default:bits=0;
            endcase
            glyph=bits>>((7-row)*8);
        end
    endfunction
    // 功能：超时提示的字符ROM；index为0..31；返回ASCII，不显示缓存数值。
    function automatic [7:0] timeout_char(input [4:0] index);
        begin case(index)
            0:timeout_char="N";1:timeout_char="O";2:timeout_char=" ";
            3:timeout_char="F";4:timeout_char="R";5:timeout_char="E";6:timeout_char="S";7:timeout_char="H";
            8:timeout_char=" ";9:timeout_char="D";10:timeout_char="A";11:timeout_char="T";12:timeout_char="A";
            default:timeout_char=" ";
        endcase end
    endfunction
    wire [7:0] c=timeout_q?(char_row_q==1?timeout_char(char_col_q):8'h20):(word_q>>(byte_q*8));
    // 将RAM字节选择、字库行选择、最终像素选择分开，缩短74.167MHz像素域关键路径。
    reg [7:0] c2,dots3;
    reg [2:0] row2,col2,col3;
    reg panel2,panel3,timeout2,timeout3,hs2,hs3,vs2,vs3,de2,de3;
    reg [7:0] r2,r3,g2,g3,b2,b3;
    always @(posedge pixel_clk or negedge pixel_rst_n) begin
        if(!pixel_rst_n) begin
            byte_q<=0;row_q<=0;col_q<=0;char_col_q<=0;char_row_q<=0;panel_q<=0;timeout_q<=1;
            hs_q<=0;vs_q<=0;de_q<=0;r_q<=0;g_q<=0;b_q<=0;
            c2<=0;dots3<=0;row2<=0;col2<=0;col3<=0;panel2<=0;panel3<=0;timeout2<=1;timeout3<=1;
            hs2<=0;hs3<=0;vs2<=0;vs3<=0;de2<=0;de3<=0;r2<=0;r3<=0;g2<=0;g3<=0;b2<=0;b3<=0;
            hs<=0;vs<=0;de<=0;red<=0;green<=0;blue<=0;
        end else begin
            byte_q<=character_addr[1:0];row_q<=dy[3:1];col_q<=dx[3:1];
            char_col_q<=dx[8:4];char_row_q<=dy[7:4];panel_q<=inside_panel;
            timeout_q<=!have_page || age>=20;
            hs_q<=hs_in;vs_q<=vs_in;de_q<=de_in;r_q<=red_in;g_q<=green_in;b_q<=blue_in;
            c2<=c;row2<=row_q;col2<=col_q;panel2<=panel_q;timeout2<=timeout_q;
            hs2<=hs_q;vs2<=vs_q;de2<=de_q;r2<=r_q;g2<=g_q;b2<=b_q;
            dots3<=glyph(c2,row2);col3<=col2;panel3<=panel2;timeout3<=timeout2;
            hs3<=hs2;vs3<=vs2;de3<=de2;r3<=r2;g3<=g2;b3<=b2;
            hs<=hs3;vs<=vs3;de<=de3;
            if(de3 && panel3) begin
                red<=dots3[7-col3]?8'hff:8'h08;
                green<=dots3[7-col3]?(timeout3?8'h60:8'hff):8'h10;
                blue<=dots3[7-col3]?(timeout3?8'h40:8'hff):8'h18;
            end else begin red<=r3;green<=g3;blue<=b3;end
        end
    end
endmodule
