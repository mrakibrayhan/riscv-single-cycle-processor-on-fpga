//====================================================================
// regfile_riscv.v  (RISC-V RV32I single-cycle CPU - register file)
//
//   Ported from regfile.v (the pipelined MIPS version) with one
//   change: the WB->ID bypass is REMOVED.
//
//   That bypass existed only because the pipeline has five
//   instructions in flight at once, so a write in one instruction's
//   WB stage and a read in a different instruction's ID stage can
//   land on the exact same clock edge. Single-cycle never has that
//   problem - there is exactly one instruction in the datapath at any
//   time, so by the time the NEXT instruction reads the register
//   file (next clock edge), this instruction's write has already
//   completed. Same reasoning your own MIPS project's comments gave
//   for why the multi-cycle version didn't need the bypass either -
//   single-cycle is in the same position.
//
//   Everything else carries over unchanged:
//     - 2 combinational read ports, 1 clocked write port
//     - x0 hardwired to always read 0 (RISC-V's equivalent of MIPS $0)
//     - synchronous reset clears all 32 physical registers
//     - per-register debug outputs for the 7-seg display, x0 excluded
//       since it never needs displaying (always 0)
//
//   RISC-V ABI names for reference (not enforced by hardware - purely
//   a software convention, listed here since you'll want it when
//   writing test programs and reading the debug display):
//     x0=zero  x1=ra   x2=sp   x3=gp   x4=tp   x5-7=t0-2   x8=s0/fp
//     x9=s1    x10-11=a0-1     x12-17=a2-7     x18-27=s2-11
//     x28-31=t3-6
//====================================================================

module regfile_riscv (
    input         clk,
    input         rst,          // synchronous reset - clears all 32
                                 // physical registers to 0 (including the
                                 // storage behind x0, which already reads
                                 // as 0 architecturally either way)
    input  [4:0]  read_reg1,    // rs1
    input  [4:0]  read_reg2,    // rs2
    input  [4:0]  write_reg,    // rd
    input  [31:0] write_data,
    input         write_enable,
    output [31:0] read_data1,
    output [31:0] read_data2,

    output [31:0] dbg_x1,
    output [31:0] dbg_x2,
    output [31:0] dbg_x3,
    output [31:0] dbg_x4,
    output [31:0] dbg_x5,
    output [31:0] dbg_x6,
    output [31:0] dbg_x7,
    output [31:0] dbg_x8,
    output [31:0] dbg_x9,
    output [31:0] dbg_x10,
    output [31:0] dbg_x11,
    output [31:0] dbg_x12,
    output [31:0] dbg_x13,
    output [31:0] dbg_x14,
    output [31:0] dbg_x15,
    output [31:0] dbg_x16,
    output [31:0] dbg_x17,
    output [31:0] dbg_x18,
    output [31:0] dbg_x19,
    output [31:0] dbg_x20,
    output [31:0] dbg_x21,
    output [31:0] dbg_x22,
    output [31:0] dbg_x23,
    output [31:0] dbg_x24,
    output [31:0] dbg_x25,
    output [31:0] dbg_x26,
    output [31:0] dbg_x27,
    output [31:0] dbg_x28,
    output [31:0] dbg_x29,
    output [31:0] dbg_x30,
    output [31:0] dbg_x31
);

    reg [31:0] registers [0:31];

    assign dbg_x1  = registers[1];
    assign dbg_x2  = registers[2];
    assign dbg_x3  = registers[3];
    assign dbg_x4  = registers[4];
    assign dbg_x5  = registers[5];
    assign dbg_x6  = registers[6];
    assign dbg_x7  = registers[7];
    assign dbg_x8  = registers[8];
    assign dbg_x9  = registers[9];
    assign dbg_x10 = registers[10];
    assign dbg_x11 = registers[11];
    assign dbg_x12 = registers[12];
    assign dbg_x13 = registers[13];
    assign dbg_x14 = registers[14];
    assign dbg_x15 = registers[15];
    assign dbg_x16 = registers[16];
    assign dbg_x17 = registers[17];
    assign dbg_x18 = registers[18];
    assign dbg_x19 = registers[19];
    assign dbg_x20 = registers[20];
    assign dbg_x21 = registers[21];
    assign dbg_x22 = registers[22];
    assign dbg_x23 = registers[23];
    assign dbg_x24 = registers[24];
    assign dbg_x25 = registers[25];
    assign dbg_x26 = registers[26];
    assign dbg_x27 = registers[27];
    assign dbg_x28 = registers[28];
    assign dbg_x29 = registers[29];
    assign dbg_x30 = registers[30];
    assign dbg_x31 = registers[31];

    //----------------------------------------------------------------
    // Reads are combinational (instant, no clock wait). No bypass -
    // see header. Priority order:
    //   1. x0 always reads as 0, no exceptions
    //   2. otherwise, return the stored contents
    //----------------------------------------------------------------
    assign read_data1 = (read_reg1 == 5'd0) ? 32'd0 : registers[read_reg1];
    assign read_data2 = (read_reg2 == 5'd0) ? 32'd0 : registers[read_reg2];

    //----------------------------------------------------------------
    // Writes happen only on a clock edge, only if enabled, and never
    // to x0. rst held high clears every register to 0, taking
    // priority over any write happening the same cycle.
    //----------------------------------------------------------------
    integer i;
    always @(posedge clk) begin
        if (rst) begin
            for (i = 0; i < 32; i = i + 1)
                registers[i] <= 32'd0;
        end else if (write_enable && write_reg != 5'd0) begin
            registers[write_reg] <= write_data;
        end
    end

endmodule