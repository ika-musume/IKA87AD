onerror {resume}
quietly WaveActivateNextPane {} 0
add wave -noupdate /IKA87AD_tb/EMUCLK
add wave -noupdate /IKA87AD_tb/RST_n
add wave -noupdate /IKA87AD_tb/PCEN
add wave -noupdate -expand -group TIMINGS /IKA87AD_tb/u_dut/timing_sr
add wave -noupdate -expand -group TIMINGS /IKA87AD_tb/u_dut/current_bus_acc
add wave -noupdate -expand -group TIMINGS /IKA87AD_tb/u_dut/cycle_tick
add wave -noupdate -expand -group TIMINGS /IKA87AD_tb/u_dut/opcode_tick
add wave -noupdate -expand -group TIMINGS /IKA87AD_tb/u_dut/rw_tick
add wave -noupdate -expand -group TIMINGS /IKA87AD_tb/u_dut/md_inlatch_tick
add wave -noupdate -expand -group FLAGS /IKA87AD_tb/u_dut/mc_alter_flag
add wave -noupdate -expand -group FLAGS /IKA87AD_tb/u_dut/flag_Z
add wave -noupdate -expand -group FLAGS /IKA87AD_tb/u_dut/flag_SK
add wave -noupdate -expand -group FLAGS /IKA87AD_tb/u_dut/flag_CY
add wave -noupdate -expand -group FLAGS /IKA87AD_tb/u_dut/flag_HC
add wave -noupdate -expand -group FLAGS /IKA87AD_tb/u_dut/flag_L1
add wave -noupdate -expand -group FLAGS /IKA87AD_tb/u_dut/flag_L0
add wave -noupdate -expand -group BUS -radix hexadecimal /IKA87AD_tb/u_dut/i_PD_I
add wave -noupdate -expand -group BUS /IKA87AD_tb/u_dut/pd_oe
add wave -noupdate -expand -group BUS /IKA87AD_tb/u_dut/o_ALE
add wave -noupdate -expand -group BUS /IKA87AD_tb/u_dut/o_RD_n
add wave -noupdate -expand -group BUS /IKA87AD_tb/u_dut/o_WR_n
add wave -noupdate -expand -group BUS -radix hexadecimal /IKA87AD_tb/u_dut/o_FULL_ADDRESS_DEBUG
add wave -noupdate -expand -group BUS -radix hexadecimal /IKA87AD_tb/u_dut/o_OUTPUT_DATA_DEBUG
add wave -noupdate -expand -group BUS -radix hexadecimal /IKA87AD_tb/u_dut/reg_MDI
add wave -noupdate -expand -group BUS -radix hexadecimal /IKA87AD_tb/u_dut/reg_MDH
add wave -noupdate -expand -group BUS -radix hexadecimal /IKA87AD_tb/u_dut/reg_MDL
add wave -noupdate -expand -group BUS -radix hexadecimal -childformat {{{/IKA87AD_tb/u_dut/reg_FULL_OPCODE_debug[0]} -radix hexadecimal} {{/IKA87AD_tb/u_dut/reg_FULL_OPCODE_debug[1]} -radix hexadecimal} {{/IKA87AD_tb/u_dut/reg_FULL_OPCODE_debug[2]} -radix hexadecimal} {{/IKA87AD_tb/u_dut/reg_FULL_OPCODE_debug[3]} -radix hexadecimal}} -subitemconfig {{/IKA87AD_tb/u_dut/reg_FULL_OPCODE_debug[0]} {-height 15 -radix hexadecimal} {/IKA87AD_tb/u_dut/reg_FULL_OPCODE_debug[1]} {-height 15 -radix hexadecimal} {/IKA87AD_tb/u_dut/reg_FULL_OPCODE_debug[2]} {-height 15 -radix hexadecimal} {/IKA87AD_tb/u_dut/reg_FULL_OPCODE_debug[3]} {-height 15 -radix hexadecimal}} /IKA87AD_tb/u_dut/reg_FULL_OPCODE_debug
add wave -noupdate -group {MC ENGINE} /IKA87AD_tb/u_dut/mc_end_of_instruction
add wave -noupdate -group {MC ENGINE} /IKA87AD_tb/u_dut/engine_state
add wave -noupdate -group {MC ENGINE} -radix unsigned /IKA87AD_tb/u_dut/engine_suspension_cntr
add wave -noupdate -group {MC ENGINE} -radix unsigned /IKA87AD_tb/u_dut/opcode_page
add wave -noupdate -group {MC ENGINE} /IKA87AD_tb/u_dut/opcode_inlatch_tick
add wave -noupdate -group {MC ENGINE} -radix hexadecimal /IKA87AD_tb/u_dut/reg_OPCODE
add wave -noupdate -group {MC ENGINE} -radix unsigned /IKA87AD_tb/u_dut/mcrom_sa
add wave -noupdate -group {MC ENGINE} -radix unsigned /IKA87AD_tb/u_dut/mc_cntr
add wave -noupdate -group {MC ENGINE} /IKA87AD_tb/u_dut/mcrom_read_tick
add wave -noupdate -group {MC ENGINE} -radix unsigned /IKA87AD_tb/u_dut/mcrom_addr
add wave -noupdate -group {MC ENGINE} /IKA87AD_tb/u_dut/mcrom_data
add wave -noupdate -group ALU -radix hexadecimal /IKA87AD_tb/u_dut/alu_adder_op0
add wave -noupdate -group ALU -radix hexadecimal /IKA87AD_tb/u_dut/alu_adder_op1
add wave -noupdate -group ALU -radix hexadecimal /IKA87AD_tb/u_dut/alu_adder_out
add wave -noupdate -group ALU -radix hexadecimal /IKA87AD_tb/u_dut/alu_output
add wave -noupdate -group ALU -radix hexadecimal /IKA87AD_tb/u_dut/alu_ma_output
add wave -noupdate -group {MC CTRL OUTPUT} /IKA87AD_tb/u_dut/mc_ctrl_output
add wave -noupdate -group {MC CTRL OUTPUT} /IKA87AD_tb/u_dut/mc_type
add wave -noupdate -group {MC CTRL OUTPUT} /IKA87AD_tb/u_dut/mc_alter_flag
add wave -noupdate -group {MC CTRL OUTPUT} /IKA87AD_tb/u_dut/mc_jump_to_next_inst
add wave -noupdate -group {MC CTRL OUTPUT} /IKA87AD_tb/u_dut/mc_next_bus_acc
add wave -noupdate -group {MC CTRL OUTPUT} /IKA87AD_tb/u_dut/mc_end_of_instruction
add wave -noupdate -group {REGFILE WR} /IKA87AD_tb/u_dut/reg_PC_wr
add wave -noupdate -group {REGFILE WR} /IKA87AD_tb/u_dut/reg_SP_wr
add wave -noupdate -group {REGFILE WR} /IKA87AD_tb/u_dut/reg_MA_wr
add wave -noupdate -group {REGFILE WR} /IKA87AD_tb/u_dut/reg_MDL_wr
add wave -noupdate -group {REGFILE WR} /IKA87AD_tb/u_dut/reg_MDH_wr
add wave -noupdate -group {REGFILE WR} /IKA87AD_tb/u_dut/reg_EAL_wr
add wave -noupdate -group {REGFILE WR} /IKA87AD_tb/u_dut/reg_EAH_wr
add wave -noupdate -group {REGFILE WR} /IKA87AD_tb/u_dut/reg_V_wr
add wave -noupdate -group {REGFILE WR} /IKA87AD_tb/u_dut/reg_A_wr
add wave -noupdate -group {REGFILE WR} /IKA87AD_tb/u_dut/reg_B_wr
add wave -noupdate -group {REGFILE WR} /IKA87AD_tb/u_dut/reg_C_wr
add wave -noupdate -group {REGFILE WR} /IKA87AD_tb/u_dut/reg_D_wr
add wave -noupdate -group {REGFILE WR} /IKA87AD_tb/u_dut/reg_E_wr
add wave -noupdate -group {REGFILE WR} /IKA87AD_tb/u_dut/reg_H_wr
add wave -noupdate -group {REGFILE WR} /IKA87AD_tb/u_dut/reg_L_wr
add wave -noupdate -group {REGFILE PCSP} -radix hexadecimal /IKA87AD_tb/u_dut/reg_PC
add wave -noupdate -group {REGFILE PCSP} -radix hexadecimal /IKA87AD_tb/u_dut/reg_SP
add wave -noupdate -group {REGFILE PCSP} /IKA87AD_tb/u_dut/reg_MA_inc_ndec
add wave -noupdate -group {REGFILE PCSP} -radix hexadecimal /IKA87AD_tb/u_dut/reg_MA
add wave -noupdate -expand -group {REGFILE VAEA} /IKA87AD_tb/u_dut/sel_VAEA
add wave -noupdate -expand -group {REGFILE VAEA} -radix hexadecimal {/IKA87AD_tb/u_dut/regpair_EAH[0]}
add wave -noupdate -expand -group {REGFILE VAEA} -radix hexadecimal {/IKA87AD_tb/u_dut/regpair_EAL[0]}
add wave -noupdate -expand -group {REGFILE VAEA} -radix hexadecimal {/IKA87AD_tb/u_dut/regpair_V[0]}
add wave -noupdate -expand -group {REGFILE VAEA} -radix hexadecimal {/IKA87AD_tb/u_dut/regpair_A[0]}
add wave -noupdate -expand -group {REGFILE VAEA} -radix hexadecimal {/IKA87AD_tb/u_dut/regpair_EAH[1]}
add wave -noupdate -expand -group {REGFILE VAEA} -radix hexadecimal {/IKA87AD_tb/u_dut/regpair_EAL[1]}
add wave -noupdate -expand -group {REGFILE VAEA} -radix hexadecimal {/IKA87AD_tb/u_dut/regpair_V[1]}
add wave -noupdate -expand -group {REGFILE VAEA} -radix hexadecimal {/IKA87AD_tb/u_dut/regpair_A[1]}
add wave -noupdate -expand -group {REGFILE BCDE} /IKA87AD_tb/u_dut/sel_BCDE
add wave -noupdate -expand -group {REGFILE BCDE} -radix hexadecimal {/IKA87AD_tb/u_dut/regpair_B[0]}
add wave -noupdate -expand -group {REGFILE BCDE} -radix hexadecimal {/IKA87AD_tb/u_dut/regpair_C[0]}
add wave -noupdate -expand -group {REGFILE BCDE} -radix hexadecimal {/IKA87AD_tb/u_dut/regpair_D[0]}
add wave -noupdate -expand -group {REGFILE BCDE} -radix hexadecimal {/IKA87AD_tb/u_dut/regpair_E[0]}
add wave -noupdate -expand -group {REGFILE BCDE} -radix hexadecimal {/IKA87AD_tb/u_dut/regpair_B[1]}
add wave -noupdate -expand -group {REGFILE BCDE} -radix hexadecimal {/IKA87AD_tb/u_dut/regpair_C[1]}
add wave -noupdate -expand -group {REGFILE BCDE} -radix hexadecimal {/IKA87AD_tb/u_dut/regpair_D[1]}
add wave -noupdate -expand -group {REGFILE BCDE} -radix hexadecimal {/IKA87AD_tb/u_dut/regpair_E[1]}
add wave -noupdate -expand -group {REGFILE HL} /IKA87AD_tb/u_dut/sel_VAEA
add wave -noupdate -expand -group {REGFILE HL} -radix hexadecimal {/IKA87AD_tb/u_dut/regpair_H[0]}
add wave -noupdate -expand -group {REGFILE HL} -radix hexadecimal {/IKA87AD_tb/u_dut/regpair_L[0]}
add wave -noupdate -expand -group {REGFILE HL} -radix hexadecimal {/IKA87AD_tb/u_dut/regpair_H[1]}
add wave -noupdate -expand -group {REGFILE HL} -radix hexadecimal {/IKA87AD_tb/u_dut/regpair_L[1]}
TreeUpdate [SetDefaultTree]
WaveRestoreCursors {{Cursor 1} {9360 ps} 0}
quietly wave cursor active 1
configure wave -namecolwidth 150
configure wave -valuecolwidth 100
configure wave -justifyvalue left
configure wave -signalnamewidth 1
configure wave -snapdistance 10
configure wave -datasetprefix 0
configure wave -rowmargin 4
configure wave -childrowmargin 2
configure wave -gridoffset 0
configure wave -gridperiod 1
configure wave -griddelta 40
configure wave -timeline 0
configure wave -timelineunits ps
update
WaveRestoreZoom {4690 ps} {16030 ps}
