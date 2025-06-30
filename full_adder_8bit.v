
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
