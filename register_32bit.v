module register32 (
    input wire [31:0] d,
    input wire clk,
    input wire rst,
    output wire [31:0] q
);
    genvar i;
    generate
        for (i = 0; i < 32; i = i + 1) begin : gen_reg
            d_flip_flop dff (
                .d(d[i]),
                .clk(clk),
                .rst(rst),
                .q(q[i])
            );
        end
    endgenerate
endmodule
