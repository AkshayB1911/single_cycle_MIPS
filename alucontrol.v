

module alucontrol(
    input wire [5:0] instr_last_6,
    input wire [1:0] ALUop,
    output wire [3:0] alu_control_out
);

    assign alu_control_out = (ALUop == 2'b00) ? 4'b0010 :
                             (ALUop == 2'b01) ? 4'b0110 :
                             (instr_last_6 == 6'b100000) ? 4'b0010 : // add
                             (instr_last_6 == 6'b100010) ? 4'b0110 : // sub
                             (instr_last_6 == 6'b100100) ? 4'b0000 : // and
                             (instr_last_6 == 6'b100101) ? 4'b0001 : // or
                             (instr_last_6 == 6'b101010) ? 4'b0111 : // slt
                             4'b0000; 

endmodule
