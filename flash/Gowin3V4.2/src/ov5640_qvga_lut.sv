// 功能：在原255项表上只修改六个值，输出320x240 RGB565并将PCLK降低约一半。
// 参数无；index为原表0..255；tuple为设备地址/寄存器地址/值，255仍为终止标记。
// 依据OV5640 CSP3 v2.01：3035高四位系统分频，3036为PLL倍率，3808..380B为输出尺寸。
// 原表4300=60输出BGR565；依OV5640数据手册改为61输出RGB565，与显示扫描位序一致。
// 原倍率140改70、系统分频保持1；由60fps基线推算约30fps，须实板确认。
module ov5640_qvga_lut(input wire [9:0] index,output reg [31:0] tuple);
    wire [31:0] original;
    lut_ov5640_rgb565_640_480_60 reference_table(index,original);
    always @* begin
        tuple=original;
        case(index)
            56:tuple=32'h78430061;
            209:tuple=32'h78303646;
            223:tuple=32'h78380801;
            224:tuple=32'h78380940;
            225:tuple=32'h78380a00;
            226:tuple=32'h78380bf0;
            default:;
        endcase
    end
endmodule
