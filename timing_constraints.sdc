## timing_constraints.sdc  (RISC-V RV32I single-cycle CPU, board build)
## Same file as the ARM SC / MC / P projects.
## clk = 50 MHz board clock (PIN_23). cpu_clk = clkdiv[1] = 50 MHz / 4 = 12.5 MHz (made in top_cpu_riscv.v).
## altera_reserved_tck = JTAG clock of the ISSP / In-System Memory Content Editor (asynchronous to the CPU clocks).

create_clock -name clk -period 20.000 [get_ports {clk}]
create_generated_clock -name cpu_clk -source [get_ports {clk}] -divide_by 4 [get_registers {clkdiv[1]}]
create_clock -name altera_reserved_tck -period 40.000 [get_ports {altera_reserved_tck}]
set_clock_groups -asynchronous -group [get_clocks {altera_reserved_tck}]
derive_clock_uncertainty
set_false_path -from [get_ports {KEY[*]}]
