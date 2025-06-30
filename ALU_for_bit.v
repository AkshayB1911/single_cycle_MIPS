module alu_i (
    input wire a,
    input wire b,
    input wire less,     
    input wire [3:0] alu_control_out,
    input wire carry_in,
    output wire result,
    output wire carry_out
);
    wire b_invert, sum, and_out, or_out;

    assign b_invert = (alu_control_out == 4'b0110 || alu_control_out == 4'b0111); // SUB or SLT
    wire b_mux = b_invert ? ~b : b;

    assign and_out = a & b;
    assign or_out = a | b;
    assign sum = a ^ b_mux ^ carry_in;
    assign carry_out = (a & b_mux) | (a & carry_in) | (b_mux & carry_in);

    assign result = (alu_control_out == 4'b0000) ? and_out :
                    (alu_control_out == 4'b0001) ? or_out  :
                    (alu_control_out == 4'b0010 || alu_control_out == 4'b0110) ? sum :
                    (alu_control_out == 4'b0111) ? less :
                    (alu_control_out == 4'b1100) ? ~(a | b) :
                    1'b0;
endmodule
