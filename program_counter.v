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

