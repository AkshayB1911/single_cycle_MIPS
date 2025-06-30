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
