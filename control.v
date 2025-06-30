
module control(
    input wire [5:0] opcode,
    output wire RegDst,
    output wire ALUSrc,
    output wire MemtoReg,
    output wire reg_write,
    output wire Memread,
    output wire Memwrite,
    output wire branch,
    output wire [1:0] ALUop,
    output wire Jump,
    output wire jal
);

    assign RegDst   = (opcode == 6'b000000) ? 1'b1 : 1'b0;
    assign ALUSrc   = (opcode == 6'b100011 || opcode == 6'b101011) ? 1'b1 : 1'b0;
    assign MemtoReg = (opcode == 6'b100011) ? 1'b1 : 1'b0;
    assign reg_write= (opcode == 6'b000000 || opcode == 6'b100011) ? 1'b1 : 1'b0;
    assign Memread  = (opcode == 6'b100011) ? 1'b1 : 1'b0;
    assign Memwrite = (opcode == 6'b101011) ? 1'b1 : 1'b0;
    assign branch   = (opcode == 6'b000100) ? 1'b1 : 1'b0;
    assign ALUop    = (opcode == 6'b000000) ? 2'b10 :
                      (opcode == 6'b000100) ? 2'b01 :
                      2'b00;

    assign Jump=(opcode == 6'b000010) ? 1'b1 : 1'b0;
    assign jal=(opcode==6'b000011) ? 1'b1 :1'b0;


endmodule
