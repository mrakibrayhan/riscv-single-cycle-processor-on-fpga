//====================================================================
// dmem_riscv.v  (byte-addressed load/store unit - lb/lh/lw/lbu/lhu,
// sb/sh/sw)
//
//   The biggest real change of the memory step. dmem.v only ever
//   needed to move a whole 32-bit word at a time (MIPS's lw/sw were
//   the only load/store ops in this project). Full RV32I base adds
//   five more: lb, lh, lbu, lhu (byte/halfword loads, signed and
//   unsigned) and sb, sh (byte/halfword stores) - so this module is
//   a small load/store unit, not just a RAM wrapper.
//
//   Storage is still a plain array of 32-bit words (256 words = 1KB
//   here - deliberately smaller than imem's 4,096, see note below).
//   addr is a BYTE address; addr[9:2] picks the word, addr[1:0]
//   picks which byte(s) within that word a byte/half access touches.
//
//   SIMPLIFYING ASSUMPTION: natural alignment is assumed and NOT
//   checked in hardware - sh/lh expected at even addresses, sw/lw at
//   4-byte-aligned addresses. Real RISC-V hardware either supports
//   misaligned access directly or traps on it; this core does
//   neither - an unaligned access will silently read/write the wrong
//   bytes. This mirrors the same simplification the rest of this
//   project already makes elsewhere (no exception/trap handling
//   exists yet at all) rather than introducing a new one.
//
//   funct3 (passed straight from the instruction, same field the
//   control unit already used to pick ADD as alu_ctrl for loads/
//   stores) tells this module BOTH the access size and, for loads,
//   the sign:
//       funct3   op   size    sign
//       000      lb   byte    signed
//       001      lh   half    signed
//       010      lw   word    -
//       100      lbu  byte    unsigned
//       101      lhu  half    unsigned
//   funct3[1:0] = size, funct3[2] = unsigned-load flag. Stores only
//   ever use 000(sb)/001(sh)/010(sw) - funct3[2] is meaningless for
//   a store and simply ignored.
//
//   Depth is 256 words rather than matching imem's 4,096: data
//   memory (stack + globals) needs far less space than program text
//   for anything you'd hand-assemble to test this core, and keeping
//   it smaller leaves more of the EP4CE6's M9K budget free. Widen
//   DEPTH below if a program actually needs more.
//
//   NO rst-triggered reload of memory contents (earlier revisions of
//   this file had one, copying dmem.v's mem_init-shadow pattern - it
//   was removed after hardware bring-up showed it silently defeats
//   Quartus's block-RAM inference). A synchronous "restore every word
//   to an arbitrary saved value" is not something block RAM hardware
//   can do at all, so having that logic present forces the ENTIRE
//   array to be built out of registers and muxes instead of the
//   chip's dedicated M9K blocks - invisible in a small memory, but at
//   256 words it was consuming ~10,000 logic elements on its own,
//   more than an EP4CE6 (6,272 LEs total) has. On real hardware, a
//   block RAM's contents come from data.hex at FPGA configuration
//   time anyway (that's what the $readmemh below does) - pressing the
//   physical reset button resets the CPU (PC, regfile), not memory
//   contents, which is also just how real hardware actually behaves.
//====================================================================

module dmem_riscv #(
    parameter ADDR_BITS = 5       // 2**ADDR_BITS words; 5 = 32 words (board / CPU-only default). Step F: bigger only in simulation
) (
    input         clk,
    input  [31:0] addr,          // byte address (from ALU: rs1 + imm)
    input  [31:0] write_data,    // raw rs2 value - this module extracts
                                  // the byte/halfword that actually
                                  // gets written
    input  [2:0]  funct3,
    input         mem_write,
    output [31:0] read_data,     // already sign/zero-extended per
                                  // funct3 - ready for the write-back mux

    output [31:0] dbg_mem0,
    output [31:0] dbg_mem1,
    output [31:0] dbg_mem2,
    output [31:0] dbg_mem3,
    output [31:0] dbg_mem4,
    output [31:0] dbg_mem5,
    output [31:0] dbg_mem6,
    output [31:0] dbg_mem7,
    output [31:0] dbg_mem8,
    output [31:0] dbg_mem9,
    output [31:0] dbg_mem10,
    output [31:0] dbg_mem11,
    output [31:0] dbg_mem12,
    output [31:0] dbg_mem13,
    output [31:0] dbg_mem14,
    output [31:0] dbg_mem15,
    output [31:0] dbg_mem16,
    output [31:0] dbg_mem17,
    output [31:0] dbg_mem18,
    output [31:0] dbg_mem19,
    output [31:0] dbg_mem20,
    output [31:0] dbg_mem21,
    output [31:0] dbg_mem22,
    output [31:0] dbg_mem23,
    output [31:0] dbg_mem24,
    output [31:0] dbg_mem25,
    output [31:0] dbg_mem26,
    output [31:0] dbg_mem27,
    output [31:0] dbg_mem28,
    output [31:0] dbg_mem29,
    output [31:0] dbg_mem30,
    output [31:0] dbg_mem31
);

    localparam DEPTH = 1 << ADDR_BITS;   // cut again (64 -> 32) alongside imem's
                              // 256 -> 128 cut, after the Fitter reported
                              // 398 LABs needed vs. the EP4CE6's 392

    reg [31:0] mem [0:DEPTH-1];

    initial $readmemh("data.hex", mem);

    wire [ADDR_BITS-1:0] word_idx  = addr[ADDR_BITS+1:2];
    wire [1:0] byte_off  = addr[1:0];
    wire [1:0] size      = funct3[1:0];   // 00=byte 01=half 10=word
    wire       unsigned_ld = funct3[2];

    wire [31:0] word_data = mem[word_idx];

    // debug taps - first 32 words, matching dmem.v's original window
    assign dbg_mem0  = mem[0];
    assign dbg_mem1  = mem[1];
    assign dbg_mem2  = mem[2];
    assign dbg_mem3  = mem[3];
    assign dbg_mem4  = mem[4];
    assign dbg_mem5  = mem[5];
    assign dbg_mem6  = mem[6];
    assign dbg_mem7  = mem[7];
    assign dbg_mem8  = mem[8];
    assign dbg_mem9  = mem[9];
    assign dbg_mem10 = mem[10];
    assign dbg_mem11 = mem[11];
    assign dbg_mem12 = mem[12];
    assign dbg_mem13 = mem[13];
    assign dbg_mem14 = mem[14];
    assign dbg_mem15 = mem[15];
    assign dbg_mem16 = mem[16];
    assign dbg_mem17 = mem[17];
    assign dbg_mem18 = mem[18];
    assign dbg_mem19 = mem[19];
    assign dbg_mem20 = mem[20];
    assign dbg_mem21 = mem[21];
    assign dbg_mem22 = mem[22];
    assign dbg_mem23 = mem[23];
    assign dbg_mem24 = mem[24];
    assign dbg_mem25 = mem[25];
    assign dbg_mem26 = mem[26];
    assign dbg_mem27 = mem[27];
    assign dbg_mem28 = mem[28];
    assign dbg_mem29 = mem[29];
    assign dbg_mem30 = mem[30];
    assign dbg_mem31 = mem[31];

    //------------------------------------------------------------
    // READ: pull the byte/half/word out of word_data and extend it.
    //------------------------------------------------------------
    reg [31:0] read_data_r;
    assign read_data = read_data_r;

    always @(*) begin
        case (size)
            2'b00: begin // byte
                reg [7:0] b;
                b = word_data[byte_off*8 +: 8];
                read_data_r = unsigned_ld ? {24'b0, b} : {{24{b[7]}}, b};
            end
            2'b01: begin // half
                reg [15:0] h;
                h = byte_off[1] ? word_data[31:16] : word_data[15:0];
                read_data_r = unsigned_ld ? {16'b0, h} : {{16{h[15]}}, h};
            end
            default: // word
                read_data_r = word_data;
        endcase
    end

    //------------------------------------------------------------
    // WRITE: build a 4-bit byte-enable mask + the data lined up in
    // the right lane(s), then apply only the enabled bytes.
    //------------------------------------------------------------
    reg [3:0]  byte_en;
    reg [31:0] write_word;

    always @(*) begin
        case (size)
            2'b00: begin // byte - enable exactly one lane, replicate the
                         // byte into all four so the mask just picks it out
                byte_en    = 4'b0001 << byte_off;
                write_word = {4{write_data[7:0]}};
            end
            2'b01: begin // half - enable the low pair or high pair of lanes
                byte_en    = byte_off[1] ? 4'b1100 : 4'b0011;
                write_word = {2{write_data[15:0]}};
            end
            default: begin // word - all four lanes
                byte_en    = 4'b1111;
                write_word = write_data;
            end
        endcase
    end

    integer i;
    always @(posedge clk) begin
        if (mem_write) begin
            for (i = 0; i < 4; i = i + 1) begin
                if (byte_en[i])
                    mem[word_idx][i*8 +: 8] <= write_word[i*8 +: 8];
            end
        end
    end

endmodule