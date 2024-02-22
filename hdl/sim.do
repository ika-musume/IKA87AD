onerror {resume}
quietly WaveActivateNextPane {} 0
add wave -noupdate /IKA87AD_tb/EMUCLK
add wave -noupdate /IKA87AD_tb/RST_n
add wave -noupdate /IKA87AD_tb/PCEN
add wave -noupdate -expand -group TIMINGS /IKA87AD_tb/u_dut/mcrom_read_tick
add wave -noupdate -expand -group TIMINGS /IKA87AD_tb/u_dut/timing_sr
add wave -noupdate -expand -group TIMINGS /IKA87AD_tb/u_dut/current_bus_acc
add wave -noupdate -expand -group TIMINGS /IKA87AD_tb/u_dut/cycle_tick
add wave -noupdate -expand -group TIMINGS /IKA87AD_tb/u_dut/opcode_tick
add wave -noupdate -expand -group TIMINGS /IKA87AD_tb/u_dut/rw_tick
add wave -noupdate -expand -group TIMINGS /IKA87AD_tb/u_dut/md_inlatch_tick
add wave -noupdate -expand -group FLAGS /IKA87AD_tb/u_dut/mc_alter_flag
add wave -noupdate -expand -group FLAGS /IKA87AD_tb/u_dut/flag_Z
add wave -noupdate -expand -group FLAGS /IKA87AD_tb/u_dut/flag_C
add wave -noupdate -expand -group FLAGS /IKA87AD_tb/u_dut/flag_SK
add wave -noupdate -expand -group FLAGS /IKA87AD_tb/u_dut/flag_HC
add wave -noupdate -expand -group FLAGS /IKA87AD_tb/u_dut/flag_L1
add wave -noupdate -expand -group FLAGS /IKA87AD_tb/u_dut/flag_L0
add wave -noupdate -expand -group IRQ /IKA87AD_tb/u_dut/irq
add wave -noupdate -expand -group IRQ /IKA87AD_tb/u_dut/irq_mask_n
add wave -noupdate -expand -group IRQ /IKA87AD_tb/u_dut/iflag
add wave -noupdate -expand -group IRQ /IKA87AD_tb/u_dut/irq_enabled
add wave -noupdate -expand -group IRQ /IKA87AD_tb/u_dut/irq_pending
add wave -noupdate -expand -group IRQ /IKA87AD_tb/u_dut/irq_detected
add wave -noupdate -expand -group IRQ /IKA87AD_tb/u_dut/hardi_proc_cyc
add wave -noupdate -expand -group IRQ /IKA87AD_tb/u_dut/softi_proc_cyc
add wave -noupdate -expand -group IRQ /IKA87AD_tb/u_dut/iflag_manual_ack
add wave -noupdate -expand -group IRQ /IKA87AD_tb/u_dut/iflag_auto_ack
add wave -noupdate -expand -group IRQ /IKA87AD_tb/u_dut/irq_lv
add wave -noupdate -expand -group IRQ /IKA87AD_tb/u_dut/irq_lv_z
add wave -noupdate -expand -group BUS /IKA87AD_tb/u_dut/mc_next_bus_acc
add wave -noupdate -expand -group BUS /IKA87AD_tb/u_dut/current_bus_acc
add wave -noupdate -expand -group BUS -radix hexadecimal /IKA87AD_tb/u_dut/i_PD_I
add wave -noupdate -expand -group BUS /IKA87AD_tb/u_dut/pd_oe
add wave -noupdate -expand -group BUS /IKA87AD_tb/u_dut/o_ALE
add wave -noupdate -expand -group BUS /IKA87AD_tb/u_dut/o_RD_n
add wave -noupdate -expand -group BUS /IKA87AD_tb/u_dut/o_WR_n
add wave -noupdate -expand -group BUS -radix hexadecimal /IKA87AD_tb/u_dut/o_FULL_ADDRESS_DEBUG
add wave -noupdate -expand -group BUS -radix hexadecimal /IKA87AD_tb/u_dut/o_OUTPUT_DATA_DEBUG
add wave -noupdate -expand -group BUS /IKA87AD_tb/u_dut/mc_s_cond_read
add wave -noupdate -expand -group BUS /IKA87AD_tb/u_dut/md_in_byte_sel
add wave -noupdate -expand -group BUS -radix hexadecimal /IKA87AD_tb/u_dut/reg_MDI
add wave -noupdate -expand -group BUS /IKA87AD_tb/u_dut/md_dirty
add wave -noupdate -expand -group BUS -radix hexadecimal /IKA87AD_tb/u_dut/reg_MDH
add wave -noupdate -expand -group BUS -radix hexadecimal /IKA87AD_tb/u_dut/reg_MDL
add wave -noupdate -expand -group BUS -radix hexadecimal -childformat {{{/IKA87AD_tb/u_dut/reg_FULL_OPCODE_debug[0]} -radix hexadecimal} {{/IKA87AD_tb/u_dut/reg_FULL_OPCODE_debug[1]} -radix hexadecimal} {{/IKA87AD_tb/u_dut/reg_FULL_OPCODE_debug[2]} -radix hexadecimal} {{/IKA87AD_tb/u_dut/reg_FULL_OPCODE_debug[3]} -radix hexadecimal}} -subitemconfig {{/IKA87AD_tb/u_dut/reg_FULL_OPCODE_debug[0]} {-height 15 -radix hexadecimal} {/IKA87AD_tb/u_dut/reg_FULL_OPCODE_debug[1]} {-height 15 -radix hexadecimal} {/IKA87AD_tb/u_dut/reg_FULL_OPCODE_debug[2]} {-height 15 -radix hexadecimal} {/IKA87AD_tb/u_dut/reg_FULL_OPCODE_debug[3]} {-height 15 -radix hexadecimal}} /IKA87AD_tb/u_dut/reg_FULL_OPCODE_debug
add wave -noupdate -expand -group {MC ENGINE} /IKA87AD_tb/u_dut/is_arith_eval_op
add wave -noupdate -expand -group {MC ENGINE} /IKA87AD_tb/u_dut/mc_s_bra_on_alu
add wave -noupdate -expand -group {MC ENGINE} /IKA87AD_tb/u_dut/mc_end_of_instruction
add wave -noupdate -expand -group {MC ENGINE} /IKA87AD_tb/u_dut/mc_jump_to_next_inst
add wave -noupdate -expand -group {MC ENGINE} -radix unsigned /IKA87AD_tb/u_dut/opcode_page
add wave -noupdate -expand -group {MC ENGINE} /IKA87AD_tb/u_dut/opcode_inlatch_tick
add wave -noupdate -expand -group {MC ENGINE} -radix hexadecimal /IKA87AD_tb/u_dut/reg_OPCODE
add wave -noupdate -expand -group {MC ENGINE} -radix unsigned /IKA87AD_tb/u_dut/mcrom_sa
add wave -noupdate -expand -group {MC ENGINE} /IKA87AD_tb/u_dut/mcrom_read_tick
add wave -noupdate -expand -group {MC ENGINE} -radix unsigned /IKA87AD_tb/u_dut/mcrom_addr
add wave -noupdate -expand -group {MC ENGINE} /IKA87AD_tb/u_dut/mcrom_data
add wave -noupdate -group {ALU MULDIV} /IKA87AD_tb/u_dut/alu_mul_start
add wave -noupdate -group {ALU MULDIV} /IKA87AD_tb/u_dut/alu_div_start
add wave -noupdate -group {ALU MULDIV} -radix unsigned /IKA87AD_tb/u_dut/alu_muldiv_cntr
add wave -noupdate -group {ALU MULDIV} -radix hexadecimal /IKA87AD_tb/u_dut/alu_mul_pa
add wave -noupdate -group {ALU MULDIV} -radix hexadecimal /IKA87AD_tb/u_dut/alu_mul_pb
add wave -noupdate -group {ALU MULDIV} -radix hexadecimal /IKA87AD_tb/u_dut/alu_div_pa
add wave -noupdate -group {ALU MULDIV} -radix hexadecimal /IKA87AD_tb/u_dut/alu_div_pb
add wave -noupdate -group {ALU MULDIV} -radix hexadecimal /IKA87AD_tb/u_dut/alu_div_out
add wave -noupdate -group {ALU MULDIV} /IKA87AD_tb/u_dut/alu_muldiv_reg_TEMP_wr
add wave -noupdate -group {ALU MULDIV} /IKA87AD_tb/u_dut/alu_muldiv_reg_EA_wr
add wave -noupdate -radix hexadecimal /IKA87AD_tb/u_dut/reg_TEMP
add wave -noupdate -group ALU -radix hexadecimal /IKA87AD_tb/u_dut/alu_pa
add wave -noupdate -group ALU -radix hexadecimal /IKA87AD_tb/u_dut/alu_pb
add wave -noupdate -group ALU -radix hexadecimal /IKA87AD_tb/u_dut/alu_adder_op0
add wave -noupdate -group ALU -radix hexadecimal /IKA87AD_tb/u_dut/alu_adder_op1
add wave -noupdate -group ALU -radix hexadecimal /IKA87AD_tb/u_dut/alu_adder_out
add wave -noupdate -group ALU -radix hexadecimal /IKA87AD_tb/u_dut/alu_output
add wave -noupdate -group ALU -radix hexadecimal /IKA87AD_tb/u_dut/alu_ma_output
add wave -noupdate -group {MC CTRL OUTPUT} /IKA87AD_tb/u_dut/mc_bk_carry_ctrl
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
add wave -noupdate -group {REGFILE WR} /IKA87AD_tb/u_dut/reg_TEMP_wr
add wave -noupdate -expand -group {REGFILE PCSP} -radix hexadecimal /IKA87AD_tb/u_dut/reg_PC
add wave -noupdate -expand -group {REGFILE PCSP} -radix hexadecimal /IKA87AD_tb/u_dut/reg_SP
add wave -noupdate -expand -group {REGFILE PCSP} /IKA87AD_tb/u_dut/reg_MA_inc_ndec
add wave -noupdate -expand -group {REGFILE PCSP} -radix hexadecimal /IKA87AD_tb/u_dut/reg_MA
add wave -noupdate -expand -group {REGFILE PCSP} -radix hexadecimal /IKA87AD_tb/u_dut/reg_TEMP
add wave -noupdate -group {REGFILE VAEA} /IKA87AD_tb/u_dut/sel_VAEA
add wave -noupdate -group {REGFILE VAEA} -radix hexadecimal {/IKA87AD_tb/u_dut/regpair_EAH[0]}
add wave -noupdate -group {REGFILE VAEA} -radix hexadecimal {/IKA87AD_tb/u_dut/regpair_EAL[0]}
add wave -noupdate -group {REGFILE VAEA} -radix hexadecimal {/IKA87AD_tb/u_dut/regpair_V[0]}
add wave -noupdate -group {REGFILE VAEA} -radix hexadecimal {/IKA87AD_tb/u_dut/regpair_A[0]}
add wave -noupdate -group {REGFILE VAEA} -radix hexadecimal {/IKA87AD_tb/u_dut/regpair_EAH[1]}
add wave -noupdate -group {REGFILE VAEA} -radix hexadecimal {/IKA87AD_tb/u_dut/regpair_EAL[1]}
add wave -noupdate -group {REGFILE VAEA} -radix hexadecimal {/IKA87AD_tb/u_dut/regpair_V[1]}
add wave -noupdate -group {REGFILE VAEA} -radix hexadecimal {/IKA87AD_tb/u_dut/regpair_A[1]}
add wave -noupdate -group {REGFILE BCDE} /IKA87AD_tb/u_dut/sel_BCDE
add wave -noupdate -group {REGFILE BCDE} -radix hexadecimal {/IKA87AD_tb/u_dut/regpair_B[0]}
add wave -noupdate -group {REGFILE BCDE} -radix hexadecimal {/IKA87AD_tb/u_dut/regpair_C[0]}
add wave -noupdate -group {REGFILE BCDE} -radix hexadecimal {/IKA87AD_tb/u_dut/regpair_D[0]}
add wave -noupdate -group {REGFILE BCDE} -radix hexadecimal {/IKA87AD_tb/u_dut/regpair_E[0]}
add wave -noupdate -group {REGFILE BCDE} -radix hexadecimal {/IKA87AD_tb/u_dut/regpair_B[1]}
add wave -noupdate -group {REGFILE BCDE} -radix hexadecimal {/IKA87AD_tb/u_dut/regpair_C[1]}
add wave -noupdate -group {REGFILE BCDE} -radix hexadecimal {/IKA87AD_tb/u_dut/regpair_D[1]}
add wave -noupdate -group {REGFILE BCDE} -radix hexadecimal {/IKA87AD_tb/u_dut/regpair_E[1]}
add wave -noupdate -group {REGFILE HL} /IKA87AD_tb/u_dut/sel_VAEA
add wave -noupdate -group {REGFILE HL} -radix hexadecimal {/IKA87AD_tb/u_dut/regpair_H[0]}
add wave -noupdate -group {REGFILE HL} -radix hexadecimal {/IKA87AD_tb/u_dut/regpair_L[0]}
add wave -noupdate -group {REGFILE HL} /IKA87AD_tb/u_dut/reg_TEMP_wr
add wave -noupdate -group {REGFILE HL} -radix hexadecimal {/IKA87AD_tb/u_dut/regpair_H[1]}
add wave -noupdate -group {REGFILE HL} -radix hexadecimal {/IKA87AD_tb/u_dut/regpair_L[1]}
TreeUpdate [SetDefaultTree]
WaveRestoreCursors {{Cursor 1} {7690 ps} 0}
quietly wave cursor active 1
configure wave -namecolwidth 166
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
WaveRestoreZoom {0 ps} {15220 ps}
