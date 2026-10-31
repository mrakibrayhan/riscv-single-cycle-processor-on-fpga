//====================================================================
// mem_m9k_riscv_sc.sv - Step G3 ABLATION ONLY (cpu_top_riscv MEM_M9K = 1; the default MEM_M9K = 0 never uses this file).
//   Same idea as mem_m9k_mips_sc.sv: ROM on the rising edge addressed by the PC register's next value (rd = ROM[PC] for the
//   whole cycle, CPI 1); RAM on the falling edge (half a cycle for the data path). The byte lanes and the load aligner are the
//   same as dmem_riscv.v (sb/sh/sw lane rules, lb/lbu/lh/lhu/lw extension).
//====================================================================
module imem_m9k_riscv_sc #(
    parameter ADDR_BITS = 7
) (
    input             clk,
    input      [31:0] pc_next,       // = the PC register's D input
    output reg [31:0] instruction
);
    (* ramstyle = "M9K" *) reg [31:0] mem [0:(1 << ADDR_BITS) - 1];
    initial $readmemh("program.hex", mem);
    always @(posedge clk) instruction <= mem[pc_next[ADDR_BITS+1:2]];
endmodule

module dmem_m9k_riscv_sc #(
    parameter ADDR_BITS = 5
) (
    input         clk,
    input  [31:0] addr,
    input  [31:0] write_data,
    input  [2:0]  funct3,
    input         mem_write,
    output [31:0] read_data
);
    (* ramstyle = "M9K" *) logic [3:0][7:0] mem [0:(1 << ADDR_BITS) - 1];
    initial $readmemh("data.hex", mem);
    wire [ADDR_BITS-1:0] word_idx = addr[ADDR_BITS+1:2];
    wire [1:0] byte_off = addr[1:0];
    wire [1:0] size = funct3[1:0];
    wire       unsigned_ld = funct3[2];
    reg  [3:0]  byte_en;
    reg  [31:0] write_word;
    always @(*) begin
        case (size)
            2'b00:   begin byte_en = 4'b0001 << byte_off;            write_word = {4{write_data[7:0]}};  end
            2'b01:   begin byte_en = byte_off[1] ? 4'b1100 : 4'b0011; write_word = {2{write_data[15:0]}}; end
            default: begin byte_en = 4'b1111;                         write_word = write_data;            end
        endcase
    end
    reg [31:0] word_data;
    always @(negedge clk) begin
        if (mem_write & byte_en[0]) mem[word_idx][0] <= write_word[7:0];
        if (mem_write & byte_en[1]) mem[word_idx][1] <= write_word[15:8];
        if (mem_write & byte_en[2]) mem[word_idx][2] <= write_word[23:16];
        if (mem_write & byte_en[3]) mem[word_idx][3] <= write_word[31:24];
        word_data <= mem[word_idx];
    end
    reg [31:0] r;
    always @(*) begin
        case (size)
            2'b00: begin : rb
                reg [7:0] b;  b = word_data[byte_off*8 +: 8];             r = unsigned_ld ? {24'b0, b} : {{24{b[7]}}, b};  end
            2'b01: begin : rh
                reg [15:0] h; h = byte_off[1] ? word_data[31:16] : word_data[15:0]; r = unsigned_ld ? {16'b0, h} : {{16{h[15]}}, h}; end
            default: r = word_data;
        endcase
    end
    assign read_data = r;
endmodule
