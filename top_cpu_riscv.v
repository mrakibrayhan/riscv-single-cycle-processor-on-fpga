//====================================================================
// top_cpu_riscv.v  (board top-level - RZ-EasyFPGA A2.2 / Cyclone IV EP4CE6)
//
//   RISC-V RV32I SINGLE-CYCLE CPU (Harris & Harris RISC-V Edition, Section 7.3) with
//   SEPARATE instruction memory (imem_riscv, 128 words) and data memory (dmem_riscv,
//   32 words), both asynchronous-read as in the textbook single-cycle processor.
//
//   Same board procedure as the ARM SC / MC / P boards (top_cpu_arm_p.v):
//   Buttons: KEY[3] = reset (the CPU is also held in reset until the first press of KEY[2]);
//            KEY[2] = run.
//   LEDG (active low = lit): [0] fault   [1] started   [2] a0 (x10) != 0   [3] halted
//            (ARM uses the Z flag on LED2; RISC-V has no flags, so a0 = the return value.)
//
//   STOP: ECALL / EBREAK / jal x0,0 -> halted;  illegal encoding -> fault.
//         PC after a stop = the address of the stopping instruction (RISC-V mepc convention;
//         ARM shows SWI / fault address + 4).
//
//   SNAPSHOT (snapshot_unit.v): on halt / fault, or when the ISSP source bit is 1, the CPU is
//   frozen and x0..x31 and the 32 data words are copied into four RAMs read with the
//   In-System Memory Content Editor:
//        R00 = x0..x15   R16 = x16..x31   D00 = data 0..15   D16 = data 16..31
//   The status values (PC, instruction, counters, flags) are on the ISSP probe below.
//
//   ISSP (altsource_probe instantiated directly, instance "RV1" - no Qsys IP needed):
//     source[0] = manual snapshot request; probe (125 bits, LSB first, SAME layout as ARM "ARM1"):
//         [0] started  [1] halted  [2] fault  [3] snapshot done
//         [7:4]     0 (ARM: NZCV flags; RISC-V has none)
//         [23:8]    PC (low 16 bits)
//         [55:24]   instruction word at PC
//         [87:56]   CYCLE COUNTER        CPU clock cycles while running (snapshot freeze not counted)
//         [119:88]  INSTRUCTION COUNTER  executed instructions (the stopping instruction included)
//         [124:120] 0 (ARM: pipe status)
//       CPI = cycle counter / instruction counter = 1 (single-cycle).
//   data.hex is loaded when the FPGA is programmed; KEY3 does not reload the data memory.
//====================================================================

module top_cpu_riscv #(
    parameter DEBOUNCE = 250000          // button filter length in clock cycles (the testbench makes it short)
) (
    input        clk,      // PIN_23
    input  [4:0] KEY,      // PIN_88,89,90,91,25 (only KEY[3], KEY[2] used)
    output [7:0] SEG,      // PIN_128,121,125,129,132,126,124,127
    output [3:0] DIG,      // PIN_133,135,136,137
    output [3:0] LEDG      // PIN_87,86,85,84
);
    //----------------------------------------------------------------
    // Clock divider: 50 MHz / 4 = 12.5 MHz for the CPU core (same as all ARM cores)
    //----------------------------------------------------------------
    reg [1:0] clkdiv = 2'b00;
    always @(posedge clk) clkdiv <= clkdiv + 2'b01;
    wire cpu_clk = clkdiv[1];

    //----------------------------------------------------------------
    // Buttons: KEY[3] reset (level), KEY[2] run (one pulse)
    //----------------------------------------------------------------
    wire key3_raw = ~KEY[3];
    wire rst_debounced;
    debounce #(.DEBOUNCE_LIMIT(DEBOUNCE)) db_rst (.clk(clk), .btn_in(key3_raw), .btn_out(rst_debounced));

    wire key2_raw = ~KEY[2];
    wire key2_debounced, step_pulse;
    debounce #(.DEBOUNCE_LIMIT(DEBOUNCE)) db_step (.clk(cpu_clk), .btn_in(key2_raw), .btn_out(key2_debounced));
    edge_detector ed_step (.clk(cpu_clk), .signal_in(key2_debounced), .pulse_out(step_pulse));

    reg started = 1'b0;
    always @(posedge cpu_clk) begin
        if (rst_debounced)   started <= 1'b0;
        else if (step_pulse) started <= 1'b1;
    end

    //----------------------------------------------------------------
    // CPU (held in reset until 'started', as on the ARM boards)
    //----------------------------------------------------------------
    wire        cpu_rst = rst_debounced | ~started;
    wire        snap_active, snap_done, halt_now, fault_now, a0_nz;
    wire [5:0]  snap_idx;
    wire [31:0] snap_data, pc, instr;
    wire        snap_req;                         // ISSP source bit

    cpu_top_riscv mycpu (
        .clk         (cpu_clk),
        .rst         (cpu_rst),
        .step_enable (~snap_active),              // frozen while the snapshot runs
        .snap_active (snap_active),
        .snap_idx    (snap_idx),
        .snap_data   (snap_data),
        .pc          (pc),
        .instr       (instr),
        .halt_now    (halt_now),
        .fault_now   (fault_now),
        .a0_nz       (a0_nz)
    );

    //----------------------------------------------------------------
    // Stop flags + counters (cpu_clk). The stopping instruction is counted (ARM convention):
    // halted / fault are set at the end of its cycle, then the counters stop.
    //----------------------------------------------------------------
    reg halted = 1'b0, fault = 1'b0;
    reg [31:0] cycle_count = 32'd0, instr_count = 32'd0;
    wire running = started & ~halted & ~fault & ~snap_active;
    always @(posedge cpu_clk) begin
        if (cpu_rst) begin
            halted <= 1'b0; fault <= 1'b0;
            cycle_count <= 32'd0; instr_count <= 32'd0;
        end else if (running) begin
            cycle_count <= cycle_count + 32'd1;
            instr_count <= instr_count + 32'd1;   // single-cycle: one instruction per cycle
            if (fault_now)     fault  <= 1'b1;
            else if (halt_now) halted <= 1'b1;
        end
    end

    //----------------------------------------------------------------
    // Snapshot unit
    //----------------------------------------------------------------
    snapshot_unit snap_unit (
        .clk          (cpu_clk),
        .rst          (cpu_rst),
        .started      (started),
        .trigger_auto (halted | fault),
        .trigger_src  (snap_req),
        .read_data    (snap_data),
        .snap_active  (snap_active),
        .done         (snap_done),
        .snap_idx     (snap_idx)
    );

    //----------------------------------------------------------------
    // 7-seg display unused
    //----------------------------------------------------------------
    assign SEG = 8'hFF;
    assign DIG = 4'hF;

    // LEDs (active low)
    assign LEDG[0] = ~fault;
    assign LEDG[1] = ~started;
    assign LEDG[2] = ~a0_nz;
    assign LEDG[3] = ~halted;

    //----------------------------------------------------------------
    // ISSP: source = manual snapshot request, probe = status + counters (ARM layout)
    //----------------------------------------------------------------
    wire [124:0] probe_status = {5'd0, instr_count, cycle_count, instr, pc[15:0], 4'd0,
                                 snap_done, fault, halted, started};
    altsource_probe #(
        .lpm_type                ("altsource_probe"),
        .lpm_hint                ("UNUSED"),
        .sld_auto_instance_index ("YES"),
        .sld_instance_index      (0),
        .SLD_NODE_INFO           (4746752),
        .sld_ir_width            (4),
        .instance_id             ("RV1"),
        .probe_width             (125),
        .source_width            (1),
        .source_initial_value    ("0"),
        .enable_metastability    ("NO")
    ) inst_issp (
        .probe      (probe_status),
        .source     (snap_req),
        .source_clk (cpu_clk),
        .source_ena (1'b1)
    );
endmodule
