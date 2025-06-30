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
