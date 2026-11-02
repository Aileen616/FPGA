// 功能：异步置位复位、两拍同步释放；arst_n 为低有效异步输入，rst_n 为本时钟域复位。
module reset_sync(input wire clk, input wire arst_n, output wire rst_n);
    (* ASYNC_REG = "TRUE" *) reg [1:0] release_ff;
    always @(posedge clk or negedge arst_n)
        if (!arst_n) release_ff <= 2'b00;
        else release_ff <= {release_ff[0], 1'b1};
    assign rst_n = release_ff[1];
endmodule
