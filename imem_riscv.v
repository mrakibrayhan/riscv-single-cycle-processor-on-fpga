//====================================================================
// imem_riscv.v  (byte-addressed, 128 instructions)
//
//   Depth set to 128 words (7-bit word index = pc[8:2]) so that the
//   asynchronous-read (logic-based) memory stays small enough to fit
//   the EP4CE6. Program must be at most 127 instructions plus the
//   halt self-loop; a PC past word 127 wraps to word 0.
//
//   Async read cannot map to M9K block RAM (M9K reads are
//   synchronous only), so this array is built from logic cells.
//====================================================================

module imem_riscv #(
    parameter ADDR_BITS = 7       // 2**ADDR_BITS words; 7 = 128 words (board / CPU-only default). Step F: bigger only in simulation
) (
    input  [31:0] pc,             // byte address
    output [31:0] instruction,

    // write port (used only by the UART loader)
    input         wr_clk,
    input         wr_en,
    input  [31:0] wr_addr,        // byte address
    input  [31:0] wr_data
);
    reg [31:0] mem [0:(1<<ADDR_BITS)-1];

    initial begin
        $readmemh("program.hex", mem);
    end

    wire [ADDR_BITS-1:0] word_addr    = pc[ADDR_BITS+1:2];
    wire [ADDR_BITS-1:0] wr_word_addr = wr_addr[ADDR_BITS+1:2];

    always @(posedge wr_clk) begin
        if (wr_en)
            mem[wr_word_addr] <= wr_data;
    end

    assign instruction = mem[word_addr];   // combinational read

endmodule