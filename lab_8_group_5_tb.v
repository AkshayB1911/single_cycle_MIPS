// Code your testbench here
// or browse Examples
module test;

    reg load_memory;
    reg [7:0] instr_add;
    reg [31:0] instr;
    reg [7:0] data_add;
    reg [31:0] data;
    reg clk;
    reg rst;
  reg reset;

    finall uut(
        .load_memory(load_memory),
        .instr_mem_input(instr),
        .data_mem_input(data),
        .mem_addr_select_instr(instr_add),
        .mem_addr_select_data(data_add),
        .clk(clk),
      .rst(rst),
      .reset(reset)
    );

    initial begin
        clk = 0;
        forever #5 clk = ~clk;
    end

    initial begin
        reset = 1;
        #10
        reset = 0;

        load_memory = 1;
        rst = 0;
        #10
        instr_add = 8'b00000000 ;
        instr = 32'b00000000000000000100000000100000;
        #10
        instr_add = 8'b00000001 ;
        instr = 32'b00000000000000000111000000100000;
        #10
        instr_add = 8'b00000010 ;
        instr = 32'b10001101000010010000000001000000;
        #10
        instr_add = 8'b00000011 ;
        instr = 32'b10001101000010100000000001000100;
        #10
        instr_add = 8'b00000100 ;
        instr = 32'b10001101000010110000000001001000;
        #10
        instr_add = 8'b00000101 ;
        instr = 32'b10101101000000000000000001001100;
        #10
        instr_add = 8'b00000110 ;
        instr = 32'b10001101000011000000000000000000;
        #10
        instr_add = 8'b00000111 ;
        instr = 32'b10001101110011010000000001001100;
        #10
        instr_add = 8'b00001000 ;
        instr = 32'b00000001101011000110100000100000;
        #10
        instr_add = 8'b00001001 ;
        instr = 32'b10101101110011010000000001001100;
        #10
        instr_add = 8'b00001010 ;
        instr = 32'b00000001001010100100100000100010;
        #10
        instr_add = 8'b00001011 ;
        instr = 32'b00000001000010110100000000100000;
        #10
        instr_add = 8'b00001100 ;
        instr = 32'b00010001001000000000000000000001;
        #10
        instr_add = 8'b00001101 ;
        instr = 32'b00010001001010011111111111111000;
        #10
      
      	instr_add = 8'b00001111;
      	instr=32'b00001000000000000000000000111111;
      #10

        data_add = 8'b00000000;
        data = 32'b00000000000000000000000000000001;
        #10
        data_add = 8'b00000001;
        data = 32'b00000000000000000000000000000000;
        #10
        data_add = 8'b00000010;
        data = 32'b00000000000000000000000000000000;
        #10
        data_add = 8'b00000011;
        data = 32'b00000000000000000000000000000000;
        #10
        data_add = 8'b00000100;
        data = 32'b00000000000000000000000000000001;
        #10
        data_add = 8'b00000101;
        data = 32'b00000000000000000000000000000000;
        #10
        data_add = 8'b00000110;
        data = 32'b00000000000000000000000000000000;
        #10
        data_add = 8'b00000111;
        data = 32'b00000000000000000000000000000000;
        #10
        data_add = 8'b00001000;
        data = 32'b00000000000000000000000000000001;
        #10
        data_add = 8'b00001001;
        data = 32'b00000000000000000000000000000000;
        #10
        data_add = 8'b00001010;
        data = 32'b00000000000000000000000000000000;
        #10
        data_add = 8'b00001011;
        data = 32'b00000000000000000000000000000001;
        #10
        data_add = 8'b00001100;
        data = 32'b00000000000000000000000000000000;
        #10
        data_add = 8'b00001101;
        data = 32'b00000000000000000000000000000000;
        #10
        data_add = 8'b00001110;
        data = 32'b00000000000000000000000000000000;
        #10
        data_add = 8'b00001111;
        data = 32'b00000000000000000000000000000001;
        #10
        data_add = 8'b01000000;
        data = 32'b00000000000000000000000000010000;
        #10
        data_add = 8'b01000100;
        data = 32'b00000000000000000000000000000001;
        #10
        data_add = 8'b01001000;
        data = 32'b00000000000000000000000000000001;

        #10
        
      
      rst=1;
      
      #10;
      rst=0;
      load_memory = 0;
        #10000
        $finish;
    end

endmodule