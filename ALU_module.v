
module alu (
    input wire [31:0] a,
    input wire [31:0] b,
    input wire [3:0] alu_control_out,
    output wire zero,
    output wire [31:0] result,
    output wire overflow
);
    wire [31:0] carry;
    wire [31:0] r;
    wire set;
    
    genvar i;
    generate
        for (i = 0; i < 31; i = i + 1) begin : alu_bits
            if (i == 0) begin
                alu_i alu_bit (
                    .a(a[i]),
                    .b(b[i]),
                    .less(set), 
                    .alu_control_out(alu_control_out),
                    .carry_in((alu_control_out == 4'b0110 || alu_control_out == 4'b0111) ? 1'b1 : (alu_control_out == 4'b0010) ? 1'b0 : 1'b0),
                    .result(r[i]),
                    .carry_out(carry[i])
                );
            end else begin
                alu_i alu_bit (
                    .a(a[i]),
                    .b(b[i]),
                    .less(1'b0),
                    .alu_control_out(alu_control_out),
                    .carry_in(carry[i-1]),
                    .result(r[i]),
                    .carry_out(carry[i])
                );
            end
        end
    endgenerate

    alu_msb alu_bit31 (
        .a(a[31]),
        .b(b[31]),
        .alu_control_out(alu_control_out),
        .carry_in(carry[30]),
        .less(1'b0),
        .result(r[31]),
        .carry_out(),
        .set(set),
        .overflow(overflow)
    );

    assign result = r;
    assign zero = ~|result; 
endmodule
