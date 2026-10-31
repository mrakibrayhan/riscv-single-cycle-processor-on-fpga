//====================================================================
// cpu_top_riscv.v  (RISC-V RV32I single-cycle CPU - datapath + control)
//
//   Reference: Harris & Harris, Digital Design and Computer Architecture,
//   RISC-V Edition [HH-RV], Section 7.3 (single-cycle processor, Fig. 7.15
//   complete datapath + control), extended to all 40 RV32I instructions:
//     PCNext   = jump ? (jalr ? ALUResult & ~1 : PCTarget) : branch_taken ? PCTarget : PCPlus4
//     PCTarget = PC + ImmExt          SrcA = rs1 (PC for auipc)    SrcB = rs2 / ImmExt
//     Result   = ALUResult / ReadData / PCPlus4 / ImmExt (lui)    ([HH-RV] ResultSrc, 4-way)
//   Memories are asynchronous-read (same-cycle), as in the textbook single-cycle CPU.
//
//   CHANGES (aligned with the ARM project's board conventions):
//     1. RUN CONTROL (same rule as the ARM SC/MC/P cores): the core STOPS on
//          - ECALL / EBREAK            -> halted   (PC = its own address)
//          - jal x0,0  (0x0000006F)    -> halted   (the assembler's end loop)
//          - an ILLEGAL encoding       -> fault    (PC = its address, no side effects)
//        RISC-V has no trap hardware here, so the PC simply stays on that instruction
//        (the address a real trap would save in mepc).
//        halt_now / fault_now are combinational ("this instruction stops the core");
//        the board top registers them into halted / fault.
//     2. The 63 per-register / per-word debug ports are removed. The board reads
//        registers and memory through the snapshot port (snap_*), like the ARM core's
//        dbg_idx / dbg_val view port. pc, instr and a0_nz remain.
//     3. The unused UART imem write port is removed (imem write port tied to 0 here).
//====================================================================

module cpu_top_riscv #(
    parameter ENABLE_ZMMUL = 0,     // 1: RV32I + Zmmul (single-cycle multiply on the embedded multipliers); 0: RV32I (board default)
    parameter IMEM_AW      = 7,     // ROM 2**IMEM_AW words (7 = 128, board / CPU-only default; Step F simulation uses more)
    parameter DMEM_AW      = 5,     // RAM 2**DMEM_AW words (5 = 32,  board / CPU-only default; Step F simulation uses more)
    parameter MEM_M9K      = 0      // Step G3 ablation: 1 = ROM/RAM in M9K (ROM on the rising edge addressed by the PC's next value, RAM on the falling edge; CPI stays 1). 0 = textbook asynchronous memories (default, all paper / board builds)
) (
    input         clk,
    input         rst,
    input         step_enable,    // 1 = execute one instruction per clock (0 = frozen)

    // snapshot view port: while snap_active = 1, register-file read port 1 and the
    // data-memory read address are driven by snap_idx (0..31 = x0..x31, 32..63 = word 0..31)
    input         snap_active,
    input  [5:0]  snap_idx,
    output [31:0] snap_data,

    output [31:0] pc,             // PC
    output [31:0] instr,          // instruction at PC
    output        halt_now,       // this instruction halts (ECALL / EBREAK / taken branch or jump to itself)
    output        fault_now,      // this instruction is illegal
    output        a0_nz           // x10 (a0) != 0  (status LED)
);

    //----------------------------------------------------------------
    // Program counter ([HH-RV] PC register). Held while the current
    // instruction stops the core.
    //----------------------------------------------------------------
    reg  [31:0] pc_r = 32'd0;
    wire [31:0] pc_next;

    always @(posedge clk) begin
        if (rst)
            pc_r <= 32'd0;
        else if (step_enable && !halt_now && !fault_now)
            pc_r <= pc_next;
    end
    assign pc = pc_r;

    //----------------------------------------------------------------
    // Fetch (instruction memory, asynchronous read)
    //----------------------------------------------------------------
    wire [31:0] instruction;
    // PC register's next value (same as the PC register above): address of the M9K ROM (MEM_M9K = 1)
    wire [31:0] pc_d = rst ? 32'd0 : ((step_enable && !halt_now && !fault_now) ? pc_next : pc_r);
    generate if (MEM_M9K) begin : g_m9k_rom
        imem_m9k_riscv_sc #(.ADDR_BITS(IMEM_AW)) instr_mem (.clk(clk), .pc_next(pc_d), .instruction(instruction));
    end else begin : g_rom
    imem_riscv #(.ADDR_BITS(IMEM_AW)) instr_mem (
        .pc          (pc_r),
        .instruction (instruction),
        .wr_clk      (clk),
        .wr_en       (1'b0),
        .wr_addr     (32'd0),
        .wr_data     (32'd0)
    );
    end endgenerate
    assign instr = instruction;

    //----------------------------------------------------------------
    // Decode - fixed bit-slices
    //----------------------------------------------------------------
    wire [6:0] opcode   = instruction[6:0];
    wire [4:0] rs1_addr = instruction[19:15];
    wire [4:0] rs2_addr = instruction[24:20];
    wire [4:0] rd_addr  = instruction[11:7];
    wire [2:0] funct3   = instruction[14:12];
    wire       funct7b5 = instruction[30];

    wire        ctrl_reg_write, ctrl_alu_src, ctrl_alu_src_a, ctrl_mem_read, ctrl_mem_write;
    wire        ctrl_branch, ctrl_jump, ctrl_jump_src, ctrl_ecall, ctrl_ebreak, ctrl_illegal;
    wire [2:0]  ctrl_imm_src;
    wire [3:0]  ctrl_alu_ctrl;
    wire [1:0]  ctrl_result_src;

    wire        ctrl_mul;
    control_unit_riscv #(.ENABLE_ZMMUL(ENABLE_ZMMUL)) ctrl (
        .opcode     (opcode),
        .funct3     (funct3),
        .funct7b5   (funct7b5),
        .sys_imm    (instruction[31:20]),
        .sys_rest   (instruction[19:7]),
        .funct7     (instruction[31:25]),
        .reg_write  (ctrl_reg_write),
        .imm_src    (ctrl_imm_src),
        .alu_src    (ctrl_alu_src),
        .alu_src_a  (ctrl_alu_src_a),
        .alu_ctrl   (ctrl_alu_ctrl),
        .mem_read   (ctrl_mem_read),     // not needed by the single-cycle datapath
        .mem_write  (ctrl_mem_write),
        .result_src (ctrl_result_src),
        .branch     (ctrl_branch),
        .jump       (ctrl_jump),
        .jump_src   (ctrl_jump_src),
        .ecall      (ctrl_ecall),
        .ebreak     (ctrl_ebreak),
        .illegal    (ctrl_illegal),
        .mul_op     (ctrl_mul)
    );

    assign fault_now = ctrl_illegal;

    wire [31:0] imm_out;
    imm_gen immgen (
        .instruction (instruction),
        .imm_src     (ctrl_imm_src),
        .imm_out     (imm_out)
    );

    //----------------------------------------------------------------
    // Register file (2 asynchronous read ports, 1 write port)
    //----------------------------------------------------------------
    wire [31:0] rs1_data, rs2_data, write_back_data, x10;
    wire        reg_write_en = step_enable && ctrl_reg_write;
    wire [4:0]  rf_ra1 = snap_active ? snap_idx[4:0] : rs1_addr;   // snapshot view

    regfile_riscv rf (
        .clk          (clk),
        .rst          (rst),
        .read_reg1    (rf_ra1),
        .read_reg2    (rs2_addr),
        .write_reg    (rd_addr),
        .write_data   (write_back_data),
        .write_enable (reg_write_en),
        .read_data1   (rs1_data),
        .read_data2   (rs2_data),
        .dbg_x1(),  .dbg_x2(),  .dbg_x3(),  .dbg_x4(),  .dbg_x5(),  .dbg_x6(),  .dbg_x7(),  .dbg_x8(),
        .dbg_x9(),  .dbg_x10(x10), .dbg_x11(), .dbg_x12(), .dbg_x13(), .dbg_x14(), .dbg_x15(), .dbg_x16(),
        .dbg_x17(), .dbg_x18(), .dbg_x19(), .dbg_x20(), .dbg_x21(), .dbg_x22(), .dbg_x23(), .dbg_x24(),
        .dbg_x25(), .dbg_x26(), .dbg_x27(), .dbg_x28(), .dbg_x29(), .dbg_x30(), .dbg_x31()
    );
    assign a0_nz = (x10 != 32'd0);

    //----------------------------------------------------------------
    // ALU ([HH-RV] SrcA / SrcB muxes)
    //----------------------------------------------------------------
    wire [31:0] alu_a = ctrl_alu_src_a ? pc_r    : rs1_data;
    wire [31:0] alu_b = ctrl_alu_src   ? imm_out : rs2_data;
    wire [31:0] alu_result;
    wire        alu_zero;

    alu_riscv alu (
        .A          (alu_a),
        .B          (alu_b),
        .alu_ctrl   (ctrl_alu_ctrl),
        .alu_result (alu_result),
        .zero_flag  (alu_zero)
    );

    //----------------------------------------------------------------
    // Data memory (asynchronous read, byte / half / word)
    //----------------------------------------------------------------
    wire [31:0] mem_read_data;
    wire        dmem_we   = step_enable && ctrl_mem_write;
    wire [31:0] dm_addr   = snap_active ? {25'd0, snap_idx[4:0], 2'b00} : alu_result;   // snapshot view
    wire [2:0]  dm_funct3 = snap_active ? 3'b010 : funct3;

    generate if (MEM_M9K) begin : g_m9k_ram
        dmem_m9k_riscv_sc #(.ADDR_BITS(DMEM_AW)) dmem (.clk(clk), .addr(dm_addr), .write_data(rs2_data), .funct3(dm_funct3),
            .mem_write(dmem_we), .read_data(mem_read_data));
    end else begin : g_ram
    dmem_riscv #(.ADDR_BITS(DMEM_AW)) dmem (
        .clk        (clk),
        .addr       (dm_addr),
        .write_data (rs2_data),
        .funct3     (dm_funct3),
        .mem_write  (dmem_we),
        .read_data  (mem_read_data),
        .dbg_mem0(),  .dbg_mem1(),  .dbg_mem2(),  .dbg_mem3(),  .dbg_mem4(),  .dbg_mem5(),  .dbg_mem6(),  .dbg_mem7(),
        .dbg_mem8(),  .dbg_mem9(),  .dbg_mem10(), .dbg_mem11(), .dbg_mem12(), .dbg_mem13(), .dbg_mem14(), .dbg_mem15(),
        .dbg_mem16(), .dbg_mem17(), .dbg_mem18(), .dbg_mem19(), .dbg_mem20(), .dbg_mem21(), .dbg_mem22(), .dbg_mem23(),
        .dbg_mem24(), .dbg_mem25(), .dbg_mem26(), .dbg_mem27(), .dbg_mem28(), .dbg_mem29(), .dbg_mem30(), .dbg_mem31()
    );
    end endgenerate

    assign snap_data = snap_idx[5] ? mem_read_data : rs1_data;

    //----------------------------------------------------------------
    // Result mux ([HH-RV] ResultSrc): alu / mem / pc+4 / imm (lui)
    //----------------------------------------------------------------
    wire [31:0] pc_plus4    = pc_r + 32'd4;          // [HH-RV] PCPlus4
    wire [31:0] pc_plus_imm = pc_r + imm_out;        // [HH-RV] PCTarget
    reg  [31:0] wb_mux_out;
    always @(*) begin
        case (ctrl_result_src)
            2'b00:   wb_mux_out = alu_result;
            2'b01:   wb_mux_out = mem_read_data;
            2'b10:   wb_mux_out = pc_plus4;
            default: wb_mux_out = imm_out;
        endcase
    end
    // Zmmul multiplier (Step E, gap G5): one 33 x 33 signed product, removed when ENABLE_ZMMUL = 0
    wire [31:0] mul_result;
    generate if (ENABLE_ZMMUL) begin : g_mul
        wire signed [32:0] ma = {(funct3 != 3'b011) & rs1_data[31], rs1_data};   // mul, mulh, mulhsu: rs1 signed
        wire signed [32:0] mb = {(funct3 == 3'b001) & rs2_data[31], rs2_data};   // mulh: rs2 signed
        wire signed [65:0] mp = ma * mb;
        assign mul_result = (funct3 == 3'b000) ? mp[31:0] : mp[63:32];
    end else begin : g_nomul
        assign mul_result = 32'd0;
    end endgenerate
    assign write_back_data = ctrl_mul ? mul_result : wb_mux_out;

    //----------------------------------------------------------------
    // Branch resolution - funct3[2] picks Zero (beq/bne) or the SLT/SLTU
    // bit (blt/bge/bltu/bgeu); funct3[0] inverts the condition.
    //----------------------------------------------------------------
    wire branch_taken = ctrl_branch & ((funct3[2] ? alu_result[0] : alu_zero) ^ funct3[0]);

    //----------------------------------------------------------------
    // Next PC ([HH-RV] PCSrc mux, plus the jalr target)
    //----------------------------------------------------------------
    wire [31:0] jalr_target = {alu_result[31:1], 1'b0};
    assign pc_next = ctrl_jump    ? (ctrl_jump_src ? jalr_target : pc_plus_imm) :
                     branch_taken ? pc_plus_imm :
                                    pc_plus4;

    // STOP rule = RV-MC / RV-P (Step E, gap G8): ecall, ebreak, or a taken branch / jump to its own address
    // (jal x0,0 = 0x0000006F and the H&H test's "beq x2,x2,done" are both covered).
    assign halt_now  = ctrl_ecall | ctrl_ebreak | ((ctrl_jump | branch_taken) & (pc_next == pc_r) & ~ctrl_illegal);
endmodule
