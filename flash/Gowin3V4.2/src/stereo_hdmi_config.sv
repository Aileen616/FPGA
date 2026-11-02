// 功能：实例化BigPig原配置控制器及降速QVGA表；双路共用启动复位，各自独立连续写表。
// 参数：SYS_HZ为平台名义控制时钟Hz；原板25MHz取25，BX72名义21MHz取21。
// 接口：rst_n低有效；done表示原控制器到表尾，error只报告原控制器错误，不阻止推进。
// RESETB持续高、PWDN持续低，按用户要求复用原顶层策略；不产生XCLK。
module stereo_hdmi_config #(parameter integer SYS_HZ=21_000_000)(
    input wire clk,rst_n,
    inout wire [1:0] scl,sda,
    output wire [1:0] reset_n,pwdn,done,error,
    output wire [9:0] index0,index1);
    wire [31:0] tuple0,tuple1;
    localparam [15:0] DIVIDER=SYS_HZ/1_000_000;
    assign reset_n=2'b11;
    assign pwdn=2'b00;
    ov5640_qvga_lut table0(index0,tuple0);
    ov5640_qvga_lut table1(index1,tuple1);
    i2c_config config0(.rst(!rst_n),.clk(clk),.clk_div_cnt(DIVIDER),.i2c_addr_2byte(1'b1),
        .lut_index(index0),.lut_dev_addr(tuple0[31:24]),.lut_reg_addr(tuple0[23:8]),
        .lut_reg_data(tuple0[7:0]),.error(error[0]),.done(done[0]),.i2c_scl(scl[0]),.i2c_sda(sda[0]));
    i2c_config config1(.rst(!rst_n),.clk(clk),.clk_div_cnt(DIVIDER),.i2c_addr_2byte(1'b1),
        .lut_index(index1),.lut_dev_addr(tuple1[31:24]),.lut_reg_addr(tuple1[23:8]),
        .lut_reg_data(tuple1[7:0]),.error(error[1]),.done(done[1]),.i2c_scl(scl[1]),.i2c_sda(sda[1]));
endmodule
