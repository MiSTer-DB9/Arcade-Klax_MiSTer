derive_pll_clocks
derive_clock_uncertainty

# core specific constraints

# The TG68K 68000 core runs in the clk_sys domain but advances only on its
# clock enables: CLKEN (ce_cpu = clk_sys/8) gates the rising-edge logic and
# CLKEN_FALL gates the recreated falling edge, which lands no closer than
# ~3.5 clk_sys to a rising edge. Every register-to-register path *inside* the
# CPU therefore has several clk_sys cycles to settle, but TimeQuest analyses
# them as single-cycle and flags the deep ALU/regfile paths (~1.7 ns over at
# 57.27 MHz). Relax intra-core paths to a conservative 2-cycle window (the real
# minimum is 3.5), scoped strictly from-CPU-to-CPU so external bus paths keep
# their single-cycle requirement.
set tg68k_regs [get_registers {*TG68K:u_cpu|*}]
set_multicycle_path -setup -from $tg68k_regs -to $tg68k_regs 2
set_multicycle_path -hold  -from $tg68k_regs -to $tg68k_regs 1
