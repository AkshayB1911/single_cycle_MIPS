
module signextend(input [15:0] in, output [31:0] out);
    assign out = {{16{in[15]}}, in};
endmodule


// module d_flip_flop (
//     input wire d,
//     input wire clk,
//     input wire rst,
//     output wire q
// );
//     wire n1, n2, n3, n4;

//     nand (n1, d, clk);
//     nand (n2, n1, clk);
//     nand (n3, n2, rst);
//     nand (q, n3, rst);
// endmodule

module d_flip_flop (
    input wire d,
    input wire clk,
    input wire rst,
    output reg q
);
    always @(posedge clk or posedge rst) begin
        if (rst) q <= 1'b0;
        else q <= d;
    end
endmodule




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


module instruction_memory (
    input wire load_memory,
    input wire [31:0] instr_mem_input,
    input wire [7:0] mem_addr_select,
    input wire [7:0] pc_output,
    input wire clk,
    input wire inst_reset,
    output wire [31:0] instr
);
    reg [31:0] instr_mem_q [255:0];
    integer i;


    always @(posedge clk or posedge inst_reset) begin
        if (inst_reset) begin
            for (i = 0; i < 256; i = i + 1)
                instr_mem_q[i] <= 32'b0;
        end
        else if (load_memory) begin
            instr_mem_q[mem_addr_select] <= instr_mem_input;
        end
    end

    assign instr = instr_mem_q[pc_output];

endmodule




module data_memory (
    input wire load_memory,
    input wire [31:0] data_mem_input,
    input wire [7:0] mem_addr_select,
    input wire [31:0] ALUresult,
    input wire [31:0] write_data,
    input wire clk,
    input wire data_reset,
    output wire [31:0] data,
    input wire Memwrite,
    input wire Memread
);
    reg [31:0] data_mem_q [255:0];
    integer i;


    always @(posedge clk or posedge data_reset) begin
        if (data_reset) begin
            for (i = 0; i < 256; i = i + 1)
                data_mem_q[i] <= 32'b0;
        end
        else if (load_memory) begin
            data_mem_q[mem_addr_select] <= data_mem_input;
    
        end
        else if (Memwrite) begin
            data_mem_q[ALUresult[7:0]] <= write_data;
           
        end
    end

    assign data = Memread ? data_mem_q[ALUresult[7:0]] : 32'b0;


endmodule




module register_file (
    input wire reg_write,
    input wire [4:0] read_reg_no_1,
    input wire [4:0] read_reg_no_2,
    input wire [4:0] write_reg_no,
    input wire [31:0] write_data,
    output wire [31:0] read_data_1,
    output wire [31:0] read_data_2,
    input wire clk,

    input wire jal,
    input wire [7:0] pc_output,

    input wire reg_file_reset
);
    wire [31:0] reg_q [31:0];

   
    assign reg_q[0] = 32'b0;  

    genvar i;
    generate
        for (i = 1; i < 32; i = i + 1) begin : reg_file_gen 
            wire [31:0] d_in;
            assign d_in = (i==5'b11111)&&(jal==1'b1) ? pc_output : (reg_file_reset)          ? 32'b0 :
                         (reg_write && (write_reg_no == i)) ? write_data : reg_q[i];
            register32 reg32 (
                .d(d_in),
                .clk(clk),
                .rst(reg_file_reset),
                .q(reg_q[i])
            );
        end
    endgenerate

    assign read_data_1 = (read_reg_no_1 == 0) ? 32'b0 : reg_q[read_reg_no_1];
    assign read_data_2 = (read_reg_no_2 == 0) ? 32'b0 : reg_q[read_reg_no_2];

    // Debugging
    always @(*) begin
        $display("reg 8 : %d", reg_q[13]);  
    end
endmodule



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






module program_counter(
    input wire pc_select_line,
    input wire [7:0] pc_input_0,
    input wire [7:0] pc_input_1,
    input wire Jump,
    input wire jal,
    input wire [7:0] pc_input_2,
    input wire clk,
    input wire rst,
    output reg [7:0] pc_output
);
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            pc_output <= 8'b0;  
            $display("PC Reset to 0");
        end
        else begin
            pc_output <= (Jump | jal ) ? pc_input_2 : pc_select_line ? pc_input_1 : pc_input_0;
            $display("PC Updated to %d (select=%b, in0=%d, in1=%d)", 
                    pc_select_line ? pc_input_1 : pc_input_0,
                    pc_select_line, pc_input_0, pc_input_1);
        
        end
    end
endmodule




module full_adder_8bit (
    input  [7:0] a,
    input  [7:0] b,
    input  cin,
    output [7:0] sum,
    output  cout
);

    wire [7:0] carry;


    assign sum[0]   = a[0] ^ b[0] ^ cin;
    assign carry[0] = (a[0] & b[0]) | (a[0] & cin) | (b[0] & cin);

    genvar i;
    generate
        for (i = 1; i < 8; i = i + 1) begin : adder_bits
            assign sum[i]   = a[i] ^ b[i] ^ carry[i-1];
            assign carry[i] = (a[i] & b[i]) | (a[i] & carry[i-1]) | (b[i] & carry[i-1]);
        end
    endgenerate

    assign cout = carry[7];

endmodule





module finall (
    input wire load_memory,
    input wire [31:0] instr_mem_input,
    input wire [31:0] data_mem_input,
    input wire [7:0] mem_addr_select_instr,
    input wire [7:0] mem_addr_select_data,

    input wire clk,
    input wire reset,
    input wire rst
);

    wire [31:0] instr;

    wire [7:0] pc_input,pc_output;

    always@(*) begin
        $display("data %d %d",mem_addr_select_data,data_mem_input);
    end
  


    instruction_memory im (
        .load_memory(load_memory),
        .instr_mem_input(instr_mem_input),
        .mem_addr_select(mem_addr_select_instr),

        .pc_output(pc_output),
        .clk(clk),
        .inst_reset(reset),
        .instr(instr)
    );

    wire zeroo;

    wire RegDst,ALUSrc,MemtoReg,reg_write,Memread,Memwrite,branch;


    wire pc_select_line;

    and and_gate(pc_select_line,branch,zeroo);

    wire [7:0] pc_input_0,pc_input_1;

    wire carr_out_idc,carr_out_idc2;


    
    wire [1:0] ALUop;

    wire Jump;
    wire jal;


  control c(.opcode(instr[31:26]),.RegDst(RegDst),.ALUSrc(ALUSrc),.MemtoReg(MemtoReg),.reg_write(reg_write),.Memread(Memread),.Memwrite(Memwrite),.branch(branch),.ALUop(ALUop),.Jump(Jump),.jal(jal));


    full_adder_8bit add1(.a(pc_output),.b(8'b00000001),.cin(1'b0),.sum(pc_input_0),.cout(carr_out_idc));

    program_counter pc(.pc_select_line(pc_select_line),.pc_input_0(pc_input_0),.pc_input_1(pc_input_1),.Jump(Jump),.jal(jal),.pc_input_2(instr[7:0]),.clk(clk),.rst(rst),.pc_output(pc_output));



    wire [31:0] read_data_1,read_data_2;

     wire [31:0] data_mem_out;

      wire [31:0] sign_ext_out;

    signextend sextend(.in(instr[15:0]),.out(sign_ext_out));

  full_adder_8bit add2(.a(pc_input_0),.b(sign_ext_out[7:0]),.cin(1'b0),.sum(pc_input_1),.cout(carr_out_idc2));

    wire [3:0] alu_control_out;
    
    wire [31:0] alu_result;
    wire overflow;

    alu alu_main(.a(read_data_1),.b(ALUSrc?sign_ext_out:read_data_2),.alu_control_out(alu_control_out),.zero(zeroo),.result(alu_result),.overflow(overflow));

    alucontrol a_c(.instr_last_6(instr[5:0]),.ALUop(ALUop),.alu_control_out(alu_control_out));

    register_file rf(
        .reg_write(reg_write),
        .read_reg_no_1(instr[25:21]),
        .read_reg_no_2(instr[20:16]),
        .write_reg_no(RegDst ? instr[15:11] : instr[20:16]),

        .write_data(MemtoReg?data_mem_out:alu_result),

        .read_data_1(read_data_1),
        .read_data_2(read_data_2),
        .clk(clk),
        .jal(jal),
        .pc_output(pc_output),

        .reg_file_reset(rst)
    );

   

   
    data_memory datam(.load_memory(load_memory),.data_mem_input(data_mem_input),.mem_addr_select(mem_addr_select_data),.ALUresult(alu_result),.write_data(read_data_2),.clk(clk),.data_reset(reset),.data(data_mem_out),.Memwrite(Memwrite),.Memread(Memread));



    

endmodule