onerror {resume}
quietly WaveActivateNextPane {} 0
add wave -noupdate /IKA87AD_tb/EMUCLK
add wave -noupdate /IKA87AD_tb/PCEN
add wave -noupdate /IKA87AD_tb/RST_n
add wave -noupdate -group TIMINGS /IKA87AD_tb/u_dut/mcrom_read_tick
add wave -noupdate -group TIMINGS /IKA87AD_tb/u_dut/timing_sr
add wave -noupdate -group TIMINGS /IKA87AD_tb/u_dut/current_bus_acc
add wave -noupdate -group TIMINGS /IKA87AD_tb/u_dut/cycle_tick
add wave -noupdate -group TIMINGS /IKA87AD_tb/u_dut/opcode_tick
add wave -noupdate -group TIMINGS /IKA87AD_tb/u_dut/rw_tick
add wave -noupdate -group TIMINGS /IKA87AD_tb/u_dut/md_inlatch_tick
add wave -noupdate -group TIMINGS /IKA87AD_tb/u_dut/mc_end_of_instruction
add wave -noupdate -group SUSP /IKA87AD_tb/u_dut/i_STOP_n
add wave -noupdate -group SUSP /IKA87AD_tb/u_dut/soft_halt_detected
add wave -noupdate -group SUSP /IKA87AD_tb/u_dut/soft_stop_detected
add wave -noupdate -group SUSP /IKA87AD_tb/u_dut/soft_halt_flag
add wave -noupdate -group SUSP /IKA87AD_tb/u_dut/soft_stop_flag
add wave -noupdate -group SUSP /IKA87AD_tb/u_dut/hard_stop_flag
add wave -noupdate -group SUSP /IKA87AD_tb/u_dut/sr_stop
add wave -noupdate -group SUSP {/IKA87AD_tb/u_dut/stop_syncchain[1]}
add wave -noupdate -group SUSP /IKA87AD_tb/u_dut/force_exec_nop
add wave -noupdate -group SUSP -radix decimal /IKA87AD_tb/u_dut/hstop_osc_wait
add wave -noupdate -group SUSP /IKA87AD_tb/u_dut/hstop_osc_unstable
add wave -noupdate -group FLAGS /IKA87AD_tb/u_dut/mc_alter_flag
add wave -noupdate -group FLAGS /IKA87AD_tb/u_dut/flag_Z
add wave -noupdate -group FLAGS /IKA87AD_tb/u_dut/flag_C
add wave -noupdate -group FLAGS /IKA87AD_tb/u_dut/flag_SK
add wave -noupdate -group FLAGS /IKA87AD_tb/u_dut/flag_HC
add wave -noupdate -group FLAGS /IKA87AD_tb/u_dut/flag_L1
add wave -noupdate -group FLAGS /IKA87AD_tb/u_dut/flag_L0
add wave -noupdate -group IRQ /IKA87AD_tb/u_dut/i_NMI_n
add wave -noupdate -group IRQ /IKA87AD_tb/u_dut/i_INT1
add wave -noupdate -group IRQ /IKA87AD_tb/u_dut/i_INT2_n
add wave -noupdate -group IRQ /IKA87AD_tb/u_dut/irq_mask_n
add wave -noupdate -group IRQ /IKA87AD_tb/u_dut/iflag
add wave -noupdate -group IRQ /IKA87AD_tb/u_dut/irq_enabled
add wave -noupdate -group IRQ /IKA87AD_tb/u_dut/irq_pending
add wave -noupdate -group IRQ /IKA87AD_tb/u_dut/irq_detected
add wave -noupdate -group IRQ /IKA87AD_tb/u_dut/force_exec_hardi
add wave -noupdate -group IRQ /IKA87AD_tb/u_dut/hardi_proc_cyc
add wave -noupdate -group IRQ /IKA87AD_tb/u_dut/softi_proc_cyc
add wave -noupdate -group IRQ /IKA87AD_tb/u_dut/iflag_manual_ack
add wave -noupdate -group IRQ /IKA87AD_tb/u_dut/iflag_auto_ack
add wave -noupdate -group IRQ /IKA87AD_tb/u_dut/irq_lv
add wave -noupdate -group IRQ /IKA87AD_tb/u_dut/irq_lv_z
add wave -noupdate -expand -group BUS /IKA87AD_tb/u_dut/o_M1_n
add wave -noupdate -expand -group BUS /IKA87AD_tb/u_dut/o_IO_n
add wave -noupdate -expand -group BUS /IKA87AD_tb/u_dut/mc_next_bus_acc
add wave -noupdate -expand -group BUS /IKA87AD_tb/u_dut/current_bus_acc
add wave -noupdate -expand -group BUS -radix hexadecimal /IKA87AD_tb/u_dut/o_A
add wave -noupdate -expand -group BUS -radix hexadecimal /IKA87AD_tb/u_dut/i_DI
add wave -noupdate -expand -group BUS -radix hexadecimal /IKA87AD_tb/u_dut/o_DO
add wave -noupdate -expand -group BUS /IKA87AD_tb/u_dut/o_PD_DO_OE
add wave -noupdate -expand -group BUS /IKA87AD_tb/u_dut/o_DO_OE
add wave -noupdate -expand -group BUS /IKA87AD_tb/u_dut/o_ALE
add wave -noupdate -expand -group BUS /IKA87AD_tb/u_dut/o_RD_n
add wave -noupdate -expand -group BUS /IKA87AD_tb/u_dut/o_WR_n
add wave -noupdate -expand -group BUS /IKA87AD_tb/u_dut/mc_s_cond_read
add wave -noupdate -expand -group BUS /IKA87AD_tb/u_dut/md_in_byte_sel
add wave -noupdate -expand -group BUS -radix hexadecimal /IKA87AD_tb/u_dut/reg_MDI
add wave -noupdate -expand -group BUS /IKA87AD_tb/u_dut/md_dirty
add wave -noupdate -expand -group BUS -radix hexadecimal /IKA87AD_tb/u_dut/reg_MDH
add wave -noupdate -expand -group BUS -radix hexadecimal /IKA87AD_tb/u_dut/reg_MDL
add wave -noupdate -expand -group BUS -radix hexadecimal -childformat {{{/IKA87AD_tb/u_dut/reg_FULL_OPCODE_debug[0]} -radix hexadecimal} {{/IKA87AD_tb/u_dut/reg_FULL_OPCODE_debug[1]} -radix hexadecimal} {{/IKA87AD_tb/u_dut/reg_FULL_OPCODE_debug[2]} -radix hexadecimal} {{/IKA87AD_tb/u_dut/reg_FULL_OPCODE_debug[3]} -radix hexadecimal}} -subitemconfig {{/IKA87AD_tb/u_dut/reg_FULL_OPCODE_debug[0]} {-height 15 -radix hexadecimal} {/IKA87AD_tb/u_dut/reg_FULL_OPCODE_debug[1]} {-height 15 -radix hexadecimal} {/IKA87AD_tb/u_dut/reg_FULL_OPCODE_debug[2]} {-height 15 -radix hexadecimal} {/IKA87AD_tb/u_dut/reg_FULL_OPCODE_debug[3]} {-height 15 -radix hexadecimal}} /IKA87AD_tb/u_dut/reg_FULL_OPCODE_debug
add wave -noupdate -group {MC ENGINE} /IKA87AD_tb/u_dut/mseq_state
add wave -noupdate -group {MC ENGINE} /IKA87AD_tb/u_dut/is_arith_eval_op
add wave -noupdate -group {MC ENGINE} /IKA87AD_tb/u_dut/mc_s_bra_on_alu
add wave -noupdate -group {MC ENGINE} /IKA87AD_tb/u_dut/mc_end_of_instruction
add wave -noupdate -group {MC ENGINE} /IKA87AD_tb/u_dut/mc_jump_to_next_inst
add wave -noupdate -group {MC ENGINE} -radix unsigned /IKA87AD_tb/u_dut/opcode_page
add wave -noupdate -group {MC ENGINE} /IKA87AD_tb/u_dut/opcode_inlatch_tick
add wave -noupdate -group {MC ENGINE} -radix hexadecimal /IKA87AD_tb/u_dut/reg_OPCODE
add wave -noupdate -group {MC ENGINE} -radix unsigned /IKA87AD_tb/u_dut/mcrom_sa
add wave -noupdate -group {MC ENGINE} /IKA87AD_tb/u_dut/mcrom_read_tick
add wave -noupdate -group {MC ENGINE} -radix unsigned /IKA87AD_tb/u_dut/mseq_cntr
add wave -noupdate -group {MC ENGINE} -radix unsigned /IKA87AD_tb/u_dut/mcrom_addr
add wave -noupdate -group {MC ENGINE} /IKA87AD_tb/u_dut/mcrom_data
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
add wave -noupdate -group {REGFILE PCSP} -radix hexadecimal /IKA87AD_tb/u_dut/reg_PC
add wave -noupdate -group {REGFILE PCSP} -radix hexadecimal /IKA87AD_tb/u_dut/reg_SP
add wave -noupdate -group {REGFILE PCSP} /IKA87AD_tb/u_dut/reg_MA_inc_ndec
add wave -noupdate -group {REGFILE PCSP} -radix hexadecimal /IKA87AD_tb/u_dut/reg_MA
add wave -noupdate -group {REGFILE PCSP} -radix hexadecimal /IKA87AD_tb/u_dut/reg_TEMP
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
add wave -noupdate -group PORTS -radix hexadecimal /IKA87AD_tb/u_dut/i_PA_I
add wave -noupdate -group PORTS -radix hexadecimal /IKA87AD_tb/u_dut/o_PA_O
add wave -noupdate -group PORTS -radix hexadecimal /IKA87AD_tb/u_dut/o_PA_OE
add wave -noupdate -group PORTS -radix hexadecimal /IKA87AD_tb/u_dut/i_PB_I
add wave -noupdate -group PORTS -radix hexadecimal /IKA87AD_tb/u_dut/o_PB_O
add wave -noupdate -group PORTS -radix hexadecimal /IKA87AD_tb/u_dut/o_PB_OE
add wave -noupdate -group PORTS -radix hexadecimal /IKA87AD_tb/u_dut/i_PC_I
add wave -noupdate -group PORTS -radix hexadecimal /IKA87AD_tb/u_dut/o_PC_O
add wave -noupdate -group PORTS -radix hexadecimal /IKA87AD_tb/u_dut/o_PC_OE
add wave -noupdate -group PORTS -radix hexadecimal /IKA87AD_tb/u_dut/i_PD_I
add wave -noupdate -group PORTS -radix hexadecimal /IKA87AD_tb/u_dut/o_PD_O
add wave -noupdate -group PORTS /IKA87AD_tb/u_dut/o_PD_OE
add wave -noupdate -group PORTS -radix hexadecimal /IKA87AD_tb/u_dut/i_PF_I
add wave -noupdate -group PORTS -radix hexadecimal /IKA87AD_tb/u_dut/o_PF_O
add wave -noupdate -group PORTS -radix hexadecimal /IKA87AD_tb/u_dut/o_PF_OE
TreeUpdate [SetDefaultTree]
WaveRestoreCursors {{Cursor 1} {5760 ps} 0}
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
WaveRestoreZoom {0 ps} {19620 ps}
