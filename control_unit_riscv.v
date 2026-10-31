//====================================================================
// control_unit_riscv.v  (RISC-V RV32I single-cycle CPU - control unit)
//
//   Reference: Harris & Harris, Digital Design and Computer Architecture,
//   RISC-V Edition [HH-RV], Section 7.3.2 (single-cycle control): a Main
//   Decoder reads op and sets the datapath control plus ALUOp; an ALU
//   Decoder reads ALUOp, funct3 and funct7b5 and sets ALUControl.
//   Pure combinational function of the instruction bits.
//
//   CHANGE (standard legality, same rule as decoder_arm.v of the ARM
//   project): every encoding outside RV32I raises 'illegal', and an illegal
//   instruction has NO side effects (no register write, no store, no
//   branch / jump). cpu_top_riscv stops the core there (fault). Checked:
//     R-type      funct7 = 0000000, or 0100000 for sub / sra only
//     slli        imm[11:5] = 0000000;  srli/srai imm[11:5] = 0000000 / 0100000
//     loads       funct3 = lb lh lw lbu lhu only      stores funct3 = sb sh sw only
//     branches    funct3 010 / 011 reserved            jalr funct3 = 000 only
//     FENCE       funct3 = 000 (FENCE.I belongs to Zifencei, not RV32I)
//     ECALL / EBREAK  the exact words 0x00000073 / 0x00100073
//   (the M-extension MUL etc. used to execute silently as ADD/SLL...;
//    the all-zero word 0x00000000 stays illegal, as the spec defines it.)
//
//   funct3 is NOT output from here - cpu_top taps instr[14:12]
//   directly for branch resolution:
//     branch_taken = branch & ((funct3[2] ? alu_result[0] : alu_zero) ^ funct3[0])
//   and for load/store size + sign extension.
//
//   Opcode map implemented (RV32I, all 40 instructions):
//     0110011 R-type    0010011 I-type ALU   0000011 Load
//     0100011 Store     1100011 Branch       1101111 JAL
//     1100111 JALR      0110111 LUI          0010111 AUIPC
//     0001111 FENCE     1110011 SYSTEM (ECALL / EBREAK)
//
//   CHANGE (finish the base ISA):
//     - FENCE is decoded as a NO-OP (single core, no cache, no
//       reordering, so there is nothing to order). funct3 must be 000.
//     - ECALL (instr[31:20]=0) and EBREAK (instr[31:20]=1), with
//       funct3=000, raise the new ecall / ebreak outputs. cpu_top
//       uses them to STOP the PC (halt). There is no trap mechanism
//       (no CSRs / mtvec), so this is a halt, not a real trap.
//     - Any other SYSTEM instruction (the CSR instructions) is still
//       flagged illegal. rs1 and rd of ECALL/EBREAK must be 0.
//     - Inputs sys_imm = instr[31:20], sys_rest = instr[19:7],
//       funct7 = instr[31:25]; outputs ecall, ebreak.
//   ALU decoder = [HH-RV] Table 7.3 extended to all RV32I ALU ops.
//====================================================================

module control_unit_riscv #(
    parameter ENABLE_ZMMUL = 0  // 1: Zmmul (mul, mulh, mulhsu, mulhu) is legal (Step E, gap G5); 0: RV32I only (board default)
) (
    input  [6:0]  opcode,     // instr[6:0]
    input  [2:0]  funct3,     // instr[14:12]
    input         funct7b5,   // instr[30]
    input  [11:0] sys_imm,    // instr[31:20] - ECALL / EBREAK, shift funct7 check
    input  [12:0] sys_rest,   // instr[19:7]  - must be 0 for ECALL / EBREAK
    input  [6:0]  funct7,     // instr[31:25] - legality only

    output reg        reg_write,
    output reg [2:0]  imm_src,     // 000=I 001=S 010=B 011=U 100=J
    output reg        alu_src,     // 0: ALU B = rs2_data   1: ALU B = imm_out
    output reg        alu_src_a,   // 0: ALU A = rs1_data   1: ALU A = pc  (auipc only)
    output reg [3:0]  alu_ctrl,
    output reg        mem_read,
    output reg        mem_write,
    output reg [1:0]  result_src,  // 00=alu_result 01=mem_data 10=pc_plus4 11=imm_out
    output reg        branch,
    output reg        jump,
    output reg        jump_src,    // 0: pc + imm_J (jal)   1: alu_result & ~1 (jalr)
    output reg        ecall,       // NEW: instruction is ECALL
    output reg        ebreak,      // NEW: instruction is EBREAK
    output reg        illegal,
    output reg        mul_op       // Zmmul instruction: the result comes from the multiplier
);

    // ---- alu_ctrl encoding - must match alu_riscv.v exactly ----
    localparam ALU_ADD  = 4'b0000;
    localparam ALU_SUB  = 4'b0001;
    localparam ALU_AND  = 4'b0010;
    localparam ALU_OR   = 4'b0011;
    localparam ALU_XOR  = 4'b0100;
    localparam ALU_SLL  = 4'b0101;
    localparam ALU_SRL  = 4'b0110;
    localparam ALU_SRA  = 4'b0111;
    localparam ALU_SLT  = 4'b1000;
    localparam ALU_SLTU = 4'b1001;

    // ---- opcode values ----
    localparam OP_RTYPE  = 7'b0110011;
    localparam OP_ITYPE  = 7'b0010011;
    localparam OP_LOAD   = 7'b0000011;
    localparam OP_STORE  = 7'b0100011;
    localparam OP_BRANCH = 7'b1100011;
    localparam OP_JAL    = 7'b1101111;
    localparam OP_JALR   = 7'b1100111;
    localparam OP_LUI    = 7'b0110111;
    localparam OP_AUIPC  = 7'b0010111;
    localparam OP_FENCE  = 7'b0001111;   // NEW
    localparam OP_SYSTEM = 7'b1110011;   // NEW

    //------------------------------------------------------------
    // Main decoder: opcode -> everything except the exact ALU op.
    //   alu_op: 00 = force ADD, 01 = branch compare, 10 = funct3/funct7b5
    //------------------------------------------------------------
    reg [1:0] alu_op;
    reg       bad;              // illegal encoding inside a known opcode

    always @(*) begin
        // safe defaults
        reg_write  = 1'b0;
        imm_src    = 3'b000;
        alu_src    = 1'b0;
        alu_src_a  = 1'b0;
        mem_read   = 1'b0;
        mem_write  = 1'b0;
        result_src = 2'b00;
        branch     = 1'b0;
        jump       = 1'b0;
        jump_src   = 1'b0;
        ecall      = 1'b0;
        ebreak     = 1'b0;
        alu_op     = 2'b00;
        illegal    = 1'b0;
        bad        = 1'b0;
        mul_op     = 1'b0;

        case (opcode)
            OP_RTYPE: begin
                mul_op    = (ENABLE_ZMMUL != 0) && (funct7 == 7'b0000001) && !funct3[2];   // mul mulh mulhsu mulhu
                bad       = !(funct7 == 7'b0000000 ||
                              (funct7 == 7'b0100000 && (funct3 == 3'b000 || funct3 == 3'b101)) ||
                              mul_op);
                reg_write = 1'b1;
                alu_src   = 1'b0;
                alu_op    = 2'b10;
            end

            OP_ITYPE: begin
                bad       = (funct3 == 3'b001 && funct7 != 7'b0000000) ||
                            (funct3 == 3'b101 && funct7 != 7'b0000000 && funct7 != 7'b0100000);
                reg_write = 1'b1;
                imm_src   = 3'b000;
                alu_src   = 1'b1;
                alu_op    = 2'b10;
            end

            OP_LOAD: begin
                bad        = (funct3 == 3'b011) || (funct3 == 3'b110) || (funct3 == 3'b111);
                reg_write  = 1'b1;
                imm_src    = 3'b000;
                alu_src    = 1'b1;
                mem_read   = 1'b1;
                result_src = 2'b01;
                alu_op     = 2'b00;
            end

            OP_STORE: begin
                bad       = (funct3[2] == 1'b1) || (funct3 == 3'b011);
                imm_src   = 3'b001;
                alu_src   = 1'b1;
                mem_write = 1'b1;
                alu_op    = 2'b00;
            end

            OP_BRANCH: begin
                bad     = (funct3 == 3'b010) || (funct3 == 3'b011);
                imm_src = 3'b010;
                alu_src = 1'b0;
                branch  = 1'b1;
                alu_op  = 2'b01;
            end

            OP_JAL: begin
                reg_write  = 1'b1;
                imm_src    = 3'b100;
                result_src = 2'b10;
                jump       = 1'b1;
                jump_src   = 1'b0;
            end

            OP_JALR: begin
                bad        = (funct3 != 3'b000);
                reg_write  = 1'b1;
                imm_src    = 3'b000;
                alu_src    = 1'b1;
                result_src = 2'b10;
                jump       = 1'b1;
                jump_src   = 1'b1;
                alu_op     = 2'b00;
            end

            OP_LUI: begin
                reg_write  = 1'b1;
                imm_src    = 3'b011;
                result_src = 2'b11;
            end

            OP_AUIPC: begin
                reg_write  = 1'b1;
                imm_src    = 3'b011;
                alu_src    = 1'b1;
                alu_src_a  = 1'b1;
                result_src = 2'b00;
                alu_op     = 2'b00;
            end

            // NEW: FENCE - architectural no-op on this core. All
            // controls stay at their safe defaults (no reg write,
            // no memory write, no branch/jump), so the PC just
            // advances by 4.
            OP_FENCE: begin
                bad = (funct3 != 3'b000);
            end

            // NEW: SYSTEM - only ECALL / EBREAK are supported.
            OP_SYSTEM: begin
                if (funct3 == 3'b000 && sys_rest == 13'd0 && sys_imm == 12'h000)
                    ecall  = 1'b1;
                else if (funct3 == 3'b000 && sys_rest == 13'd0 && sys_imm == 12'h001)
                    ebreak = 1'b1;
                else
                    bad    = 1'b1;    // CSR instructions, MRET, WFI ... - not implemented
            end

            default: begin
                bad = 1'b1;
            end
        endcase

        // an illegal instruction has no side effects (the core stops on it)
        if (bad) begin
            illegal    = 1'b1;
            mul_op     = 1'b0;
            reg_write  = 1'b0;
            mem_read   = 1'b0;
            mem_write  = 1'b0;
            branch     = 1'b0;
            jump       = 1'b0;
            ecall      = 1'b0;
            ebreak     = 1'b0;
        end
    end

    //------------------------------------------------------------
    // ALU decoder: alu_op + funct3 + funct7b5 -> alu_ctrl
    //------------------------------------------------------------
    wire is_rtype = (opcode == OP_RTYPE);

    always @(*) begin
        case (alu_op)
            2'b00: alu_ctrl = ALU_ADD;

            2'b01: begin
                case (funct3)
                    3'b000, 3'b001: alu_ctrl = ALU_SUB;
                    3'b100, 3'b101: alu_ctrl = ALU_SLT;
                    3'b110, 3'b111: alu_ctrl = ALU_SLTU;
                    default:        alu_ctrl = ALU_SUB;
                endcase
            end

            2'b10: begin
                case (funct3)
                    3'b000:  alu_ctrl = (is_rtype && funct7b5) ? ALU_SUB : ALU_ADD;
                    3'b001:  alu_ctrl = ALU_SLL;
                    3'b010:  alu_ctrl = ALU_SLT;
                    3'b011:  alu_ctrl = ALU_SLTU;
                    3'b100:  alu_ctrl = ALU_XOR;
                    3'b101:  alu_ctrl = funct7b5 ? ALU_SRA : ALU_SRL;
                    3'b110:  alu_ctrl = ALU_OR;
                    3'b111:  alu_ctrl = ALU_AND;
                    default: alu_ctrl = ALU_ADD;
                endcase
            end

            default: alu_ctrl = ALU_ADD;
        endcase
    end

endmodule