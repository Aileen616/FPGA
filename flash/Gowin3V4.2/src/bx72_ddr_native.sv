// 功能：复用BX72厂家DDR3 IP及50/200MHz PLL，提供八客户端原生突发接口。
// 接口：board_clk为Y18的50MHz；初始化复位独立于电机复位键；用户时钟由IP输出。
// 控制器参数和SSTL15引脚沿用厂家工程，不在候选中猜测或重新生成器件时序。
module bx72_ddr_native(input wire board_clk,init_rst_n,user_rst_n,
    output wire user_clk,calibrated,reset_active,input wire [7:0] req_valid,req_write,output wire [7:0] req_ready,
    input wire [231:0] req_addr,input wire [2047:0] req_data,output wire [7:0] rsp_valid,
    output wire [255:0] rsp_data,
    output wire [14:0] O_ddr_addr,output wire [2:0] O_ddr_ba,
    output wire O_ddr_cs_n,O_ddr_ras_n,O_ddr_cas_n,O_ddr_we_n,O_ddr_clk,O_ddr_clk_n,
    output wire O_ddr_cke,O_ddr_odt,O_ddr_reset_n,output wire [3:0] O_ddr_dqm,
    inout wire [31:0] IO_ddr_dq,inout wire [3:0] IO_ddr_dqs,IO_ddr_dqs_n);
    wire memory_clk,pll_lock,pll_stop,ddr_rst,board_reset_n,pll_reset;
    assign reset_active=ddr_rst;
    wire [5:0] icpsel;wire [2:0] lpfres;
    // 官方指南要求clk连续、rst_n在clk域同步释放，memory_clk使用CLKOUT2并受pll_stop控制。
    reset_sync board_reset(board_clk,init_rst_n,board_reset_n);
    bx72_ddr_pll_mod clock_generator(.clkin(board_clk),.memory_clk(memory_clk),.lock(pll_lock),
        .reset(pll_reset),.clock_enable(pll_stop),.icpsel(icpsel),.lpfres(lpfres));
    PLL_INIT #(.CLK_PERIOD(20),.MULTI_FAC(16)) clock_init(.CLKIN(board_clk),.I_RST(!board_reset_n),
        .O_RST(pll_reset),.PLLLOCK(pll_lock),.O_LOCK(),.ICPSEL(icpsel),.LPFRES(lpfres));
    wire cmd_ready,wr_data_rdy,rd_data_valid,cmd_en,wr_data_en,wr_data_end;
    wire [2:0] cmd;wire [28:0] addr;wire [255:0] wr_data,rd_data;wire [31:0] mask;
    ddr_native_arbiter #(.QOS_ENABLE(1),.DISPLAY_PORT(5),.NETWORK_PORT(4),
        .NETWORK_GAP(128),.DISPLAY_MAX(2)) arbiter(user_clk,user_rst_n,calibrated,req_valid,req_write,req_ready,
        req_addr,req_data,rsp_valid,rsp_data,cmd_ready,wr_data_rdy,rd_data_valid,rd_data,
        cmd,cmd_en,addr,wr_data,wr_data_en,wr_data_end,mask);
    DDR3_Memory_Interface_Top controller(
        .clk(board_clk),.memory_clk(memory_clk),.pll_lock(pll_lock),.rst_n(board_reset_n),
        .pll_stop(pll_stop),.clk_out(user_clk),.ddr_rst(ddr_rst),.init_calib_complete(calibrated),
        .cmd_ready(cmd_ready),.cmd(cmd),.cmd_en(cmd_en),.addr(addr),
        .wr_data_rdy(wr_data_rdy),.wr_data(wr_data),.wr_data_en(wr_data_en),.wr_data_end(wr_data_end),
        .wr_data_mask(mask),.rd_data(rd_data),.rd_data_valid(rd_data_valid),.rd_data_end(),
        .sr_req(1'b0),.ref_req(1'b0),.sr_ack(),.ref_ack(),.burst(1'b1),
        .O_ddr_addr(O_ddr_addr),.O_ddr_ba(O_ddr_ba),.O_ddr_cs_n(O_ddr_cs_n),.O_ddr_ras_n(O_ddr_ras_n),
        .O_ddr_cas_n(O_ddr_cas_n),.O_ddr_we_n(O_ddr_we_n),.O_ddr_clk(O_ddr_clk),.O_ddr_clk_n(O_ddr_clk_n),
        .O_ddr_cke(O_ddr_cke),.O_ddr_odt(O_ddr_odt),.O_ddr_reset_n(O_ddr_reset_n),.O_ddr_dqm(O_ddr_dqm),
        .IO_ddr_dq(IO_ddr_dq),.IO_ddr_dqs(IO_ddr_dqs),.IO_ddr_dqs_n(IO_ddr_dqs_n));
endmodule
