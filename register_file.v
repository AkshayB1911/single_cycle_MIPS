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
