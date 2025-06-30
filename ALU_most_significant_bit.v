module alu_msb (
    input wire a,
    input wire b,
    input wire [3:0] alu_control_out,
    input wire carry_in,
    input wire less,          
    output wire result,
    output wire carry_out,
    output wire set,          
    output wire overflow
);
    wire b_invert = (alu_control_out == 4'b0110 || alu_control_out == 4'b0111);
    wire b_mux = b_invert ? ~b : b;

    wire sum = a ^ b_mux ^ carry_in;
    assign carry_out = (a & b_mux) | (a & carry_in) | (b_mux & carry_in);
    assign overflow = carry_in ^ carry_out;
    assign set = sum;

    assign result = (alu_control_out == 4'b0000) ? (a & b) :
                    (alu_control_out == 4'b0001) ? (a | b) :
                    (alu_control_out == 4'b0010 || alu_control_out == 4'b0110) ? sum :
                    (alu_control_out == 4'b0111) ? less :
                    (alu_control_out == 4'b1100) ? ~(a | b) :
                    1'b0;
endmodule
