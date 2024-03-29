/**
 * @module decoder_5_32
 * @brief 5-to-32 decoder module
 *
 * This module takes a 5-bit input and generates a 32-bit output. Each bit of the output corresponds to a specific input value.
 * The output bit is set to 1 if the input value matches the index of the bit, otherwise it is set to 0.
 *
 * @param in   [4:0]   - 5-bit input
 * @param out  [31:0]  - 32-bit output
 */
module decoder_5_32(
    input  [ 4:0] in,
    output [31:0] out
);

genvar i;
generate for (i=0; i<32; i=i+1) begin : gen_for_dec_5_32
    assign out[i] = (in == i);
end endgenerate

endmodule

/**
 * @module decoder_6_64
 * @brief 6-to-64 decoder module
 *
 * This module takes a 6-bit input and generates a 64-bit output. 
 * Each bit of the output is set to 1 if it matches the input value, otherwise it is set to 0.
 *
 * @param in   [5:0]   - 6-bit input
 * @param out  [63:0]  - 64-bit output
 */
module decoder_6_64(
    input  [ 5:0] in,
    output [63:0] out
);

genvar i;
generate for (i=0; i<63; i=i+1) begin : gen_for_dec_6_64
    assign out[i] = (in == i);
end endgenerate

endmodule

