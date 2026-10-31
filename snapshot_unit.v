//====================================================================
// snapshot_unit.v - copies x0..x31 and the 32 data words into FOUR small block RAMs
//                   (M9K) read over JTAG with Quartus's In-System Memory Content Editor.
//
//   Same unit as snapshot_arm_*.v of the ARM projects (same states, same reset clear):
//        R00 : x0  .. x15                 D00 : data memory word  0 .. 15
//        R16 : x16 .. x31                 D16 : data memory word 16 .. 31
//   (RISC-V has 32 registers, so R16 holds x16..x31; ARM uses R16 for its status words.
//    The status values - PC, instruction, counters, started/halted/fault - are on the
//    ISSP probe "RV1".)   e.g. x20 = R16 word 4, data word 18 = D16 word 2.
//
//   WHEN: the program halts or faults (trigger_auto), or the ISSP source bit is set
//   (trigger_src, any time).
//   IDLE (CPU runs) -> SNAP (64 cycles, CPU frozen) -> DONE (frozen while the trigger
//   stays true) -> IDLE.  The single-cycle core reads its register file and data memory
//   combinationally, so the value for index i is written in the same cycle (no WAIT state
//   and no one-cycle lag, unlike the pipelined ARM unit).
//   RESET CLEAR (as in the ARM unit): while rst = 1 (KEY3 held, or the CPU not started
//   yet) the four RAMs are written with 0, word by word (16 cycles).
//   Each ISMCE instance costs about 120 logic elements (JTAG access logic).
//====================================================================

module snapshot_unit (
    input         clk,           // cpu_clk
    input         rst,           // cpu reset (level)
    input         started,       // CPU has been started with KEY[2]
    input         trigger_auto,  // halted | fault
    input         trigger_src,   // manual request: ISSP source bit (JTAG clock domain)
    input  [31:0] read_data,     // core view value for snap_idx (combinational)
    output        snap_active,   // freeze the CPU and drive the view port
    output        done,
    output [5:0]  snap_idx       // view index for the core (0..31 regs, 32..63 data words)
);
    reg s1 = 1'b0, s2 = 1'b0;
    always @(posedge clk) begin
        s1 <= trigger_src;
        s2 <= s1;
    end
    wire trigger = trigger_auto | s2;

    localparam IDLE = 2'd0, SNAP = 2'd1, DONE = 2'd2;
    reg [1:0] state = IDLE;
    reg [5:0] idx   = 6'd0;           // 0..31 registers, 32..63 data words

    always @(posedge clk) begin
        if (rst) begin
            state <= IDLE;
            idx   <= 6'd0;
        end else begin
            case (state)
                IDLE: if (started && trigger) begin state <= SNAP; idx <= 6'd0; end
                SNAP: if (idx == 6'd63) state <= DONE; else idx <= idx + 6'd1;
                DONE: if (!trigger) state <= IDLE;
                default: state <= IDLE;
            endcase
        end
    end

    assign snap_active = (state != IDLE);
    assign done        = (state == DONE);
    assign snap_idx    = idx;
    wire   wr          = (state == SNAP);

    // reset clear: a 4-bit word counter that runs while rst = 1
    reg [3:0] clr = 4'd0;
    always @(posedge clk) clr <= rst ? clr + 4'd1 : 4'd0;

    wire [3:0]  word_in   = rst ? clr : idx[3:0];
    wire [31:0] snap_data = rst ? 32'd0 : read_data;
    // idx[5:4]: 0 R00, 1 R16, 2 D00, 3 D16
    wire we_R00 = rst | (wr && idx[5:4] == 2'd0);
    wire we_R16 = rst | (wr && idx[5:4] == 2'd1);
    wire we_D00 = rst | (wr && idx[5:4] == 2'd2);
    wire we_D16 = rst | (wr && idx[5:4] == 2'd3);
    wire [31:0] q_R00_unused, q_R16_unused, q_D00_unused, q_D16_unused;

    altsyncram #(
        .operation_mode ("SINGLE_PORT"), .width_a (32), .widthad_a (4), .numwords_a (16),
        .outdata_reg_a ("UNREGISTERED"), .clock_enable_input_a ("BYPASS"), .clock_enable_output_a ("BYPASS"),
        .power_up_uninitialized ("FALSE"), .read_during_write_mode_port_a ("NEW_DATA_NO_NBE_READ"),
        .ram_block_type ("M9K"), .intended_device_family ("Cyclone IV E"),
        .lpm_hint ("ENABLE_RUNTIME_MOD=YES,INSTANCE_NAME=R00"), .lpm_type ("altsyncram")
    ) snap_ram_R00 (.address_a (word_in), .clock0 (clk), .data_a (snap_data), .wren_a (we_R00), .q_a (q_R00_unused));

    altsyncram #(
        .operation_mode ("SINGLE_PORT"), .width_a (32), .widthad_a (4), .numwords_a (16),
        .outdata_reg_a ("UNREGISTERED"), .clock_enable_input_a ("BYPASS"), .clock_enable_output_a ("BYPASS"),
        .power_up_uninitialized ("FALSE"), .read_during_write_mode_port_a ("NEW_DATA_NO_NBE_READ"),
        .ram_block_type ("M9K"), .intended_device_family ("Cyclone IV E"),
        .lpm_hint ("ENABLE_RUNTIME_MOD=YES,INSTANCE_NAME=R16"), .lpm_type ("altsyncram")
    ) snap_ram_R16 (.address_a (word_in), .clock0 (clk), .data_a (snap_data), .wren_a (we_R16), .q_a (q_R16_unused));

    altsyncram #(
        .operation_mode ("SINGLE_PORT"), .width_a (32), .widthad_a (4), .numwords_a (16),
        .outdata_reg_a ("UNREGISTERED"), .clock_enable_input_a ("BYPASS"), .clock_enable_output_a ("BYPASS"),
        .power_up_uninitialized ("FALSE"), .read_during_write_mode_port_a ("NEW_DATA_NO_NBE_READ"),
        .ram_block_type ("M9K"), .intended_device_family ("Cyclone IV E"),
        .lpm_hint ("ENABLE_RUNTIME_MOD=YES,INSTANCE_NAME=D00"), .lpm_type ("altsyncram")
    ) snap_ram_D00 (.address_a (word_in), .clock0 (clk), .data_a (snap_data), .wren_a (we_D00), .q_a (q_D00_unused));

    altsyncram #(
        .operation_mode ("SINGLE_PORT"), .width_a (32), .widthad_a (4), .numwords_a (16),
        .outdata_reg_a ("UNREGISTERED"), .clock_enable_input_a ("BYPASS"), .clock_enable_output_a ("BYPASS"),
        .power_up_uninitialized ("FALSE"), .read_during_write_mode_port_a ("NEW_DATA_NO_NBE_READ"),
        .ram_block_type ("M9K"), .intended_device_family ("Cyclone IV E"),
        .lpm_hint ("ENABLE_RUNTIME_MOD=YES,INSTANCE_NAME=D16"), .lpm_type ("altsyncram")
    ) snap_ram_D16 (.address_a (word_in), .clock0 (clk), .data_a (snap_data), .wren_a (we_D16), .q_a (q_D16_unused));
endmodule
