onerror {resume}
quietly WaveActivateNextPane {} 0
add wave -noupdate -group CLK /IKA87AD_tb/u_dut/i_EMUCLK
add wave -noupdate -group CLK /IKA87AD_tb/u_dut/i_MCUCLK_PCEN
add wave -noupdate -group CLK /IKA87AD_tb/u_dut/i_RESET_n
add wave -noupdate -group CLK /IKA87AD_tb/u_dut/sr_stop
add wave -noupdate -group CLK /IKA87AD_tb/u_dut/timing_sr
add wave -noupdate -group TICKS /IKA87AD_tb/u_dut/mcrom_read_tick
add wave -noupdate -group TICKS /IKA87AD_tb/u_dut/opcode_tick
add wave -noupdate -group TICKS /IKA87AD_tb/u_dut/rw_tick
add wave -noupdate -group TICKS /IKA87AD_tb/u_dut/cycle_tick
add wave -noupdate -group TICKS /IKA87AD_tb/u_dut/opcode_inlatch_tick
add wave -noupdate -group TICKS /IKA87AD_tb/u_dut/md_inlatch_tick
add wave -noupdate -group TICKS /IKA87AD_tb/u_dut/md_outlatch_tick
add wave -noupdate -expand -group BUS /IKA87AD_tb/u_dut/o_M1_n
add wave -noupdate -expand -group BUS /IKA87AD_tb/u_dut/o_ALE
add wave -noupdate -expand -group BUS /IKA87AD_tb/u_dut/o_RD_n
add wave -noupdate -expand -group BUS /IKA87AD_tb/u_dut/o_WR_n
add wave -noupdate -expand -group BUS -radix hexadecimal /IKA87AD_tb/u_dut/o_A
add wave -noupdate -expand -group BUS -radix hexadecimal /IKA87AD_tb/u_dut/i_DI
add wave -noupdate -expand -group BUS -radix hexadecimal /IKA87AD_tb/u_dut/o_DO
add wave -noupdate -expand -group BUS -radix unsigned /IKA87AD_tb/u_dut/mc_next_bus_acc
add wave -noupdate -expand -group BUS -radix unsigned /IKA87AD_tb/u_dut/current_bus_acc
add wave -noupdate -group OPCODE -radix unsigned /IKA87AD_tb/u_dut/opcode_page
add wave -noupdate -group OPCODE -radix hexadecimal -childformat {{{/IKA87AD_tb/u_dut/reg_OPCODE[7]} -radix hexadecimal} {{/IKA87AD_tb/u_dut/reg_OPCODE[6]} -radix hexadecimal} {{/IKA87AD_tb/u_dut/reg_OPCODE[5]} -radix hexadecimal} {{/IKA87AD_tb/u_dut/reg_OPCODE[4]} -radix hexadecimal} {{/IKA87AD_tb/u_dut/reg_OPCODE[3]} -radix hexadecimal} {{/IKA87AD_tb/u_dut/reg_OPCODE[2]} -radix hexadecimal} {{/IKA87AD_tb/u_dut/reg_OPCODE[1]} -radix hexadecimal} {{/IKA87AD_tb/u_dut/reg_OPCODE[0]} -radix hexadecimal}} -subitemconfig {{/IKA87AD_tb/u_dut/reg_OPCODE[7]} {-height 15 -radix hexadecimal} {/IKA87AD_tb/u_dut/reg_OPCODE[6]} {-height 15 -radix hexadecimal} {/IKA87AD_tb/u_dut/reg_OPCODE[5]} {-height 15 -radix hexadecimal} {/IKA87AD_tb/u_dut/reg_OPCODE[4]} {-height 15 -radix hexadecimal} {/IKA87AD_tb/u_dut/reg_OPCODE[3]} {-height 15 -radix hexadecimal} {/IKA87AD_tb/u_dut/reg_OPCODE[2]} {-height 15 -radix hexadecimal} {/IKA87AD_tb/u_dut/reg_OPCODE[1]} {-height 15 -radix hexadecimal} {/IKA87AD_tb/u_dut/reg_OPCODE[0]} {-height 15 -radix hexadecimal}} /IKA87AD_tb/u_dut/reg_OPCODE
add wave -noupdate -expand -group ADDRESS /IKA87AD_tb/u_dut/mc_t3_cond_pc_dec
add wave -noupdate -expand -group ADDRESS -radix hexadecimal /IKA87AD_tb/u_dut/next_pc
add wave -noupdate -expand -group ADDRESS -radix hexadecimal /IKA87AD_tb/u_dut/reg_PC
add wave -noupdate -expand -group ADDRESS /IKA87AD_tb/u_dut/pc_hold
add wave -noupdate -expand -group ADDRESS /IKA87AD_tb/u_dut/sp_autocnt
add wave -noupdate -expand -group ADDRESS -radix hexadecimal /IKA87AD_tb/u_dut/reg_SP
add wave -noupdate -expand -group ADDRESS /IKA87AD_tb/u_dut/reg_MA_wr
add wave -noupdate -expand -group ADDRESS -radix hexadecimal /IKA87AD_tb/u_dut/reg_MA
add wave -noupdate -expand -group ADDRESS -radix hexadecimal /IKA87AD_tb/u_dut/reg_DAUX
add wave -noupdate -expand -group ADDRESS /IKA87AD_tb/u_dut/aaux_we
add wave -noupdate -expand -group ADDRESS -radix hexadecimal /IKA87AD_tb/u_dut/reg_AAUX
add wave -noupdate -expand -group DATA -radix unsigned /IKA87AD_tb/u_dut/md_tos
add wave -noupdate -expand -group DATA /IKA87AD_tb/u_dut/reg_MD0_wr
add wave -noupdate -expand -group DATA /IKA87AD_tb/u_dut/reg_MD1_wr
add wave -noupdate -expand -group DATA /IKA87AD_tb/u_dut/reg_MD2_wr
add wave -noupdate -expand -group DATA -radix hexadecimal -childformat {{{/IKA87AD_tb/u_dut/reg_MD[0]} -radix hexadecimal} {{/IKA87AD_tb/u_dut/reg_MD[1]} -radix hexadecimal} {{/IKA87AD_tb/u_dut/reg_MD[2]} -radix hexadecimal} {{/IKA87AD_tb/u_dut/reg_MD[3]} -radix hexadecimal}} -subitemconfig {{/IKA87AD_tb/u_dut/reg_MD[0]} {-height 15 -radix hexadecimal} {/IKA87AD_tb/u_dut/reg_MD[1]} {-height 15 -radix hexadecimal} {/IKA87AD_tb/u_dut/reg_MD[2]} {-height 15 -radix hexadecimal} {/IKA87AD_tb/u_dut/reg_MD[3]} {-height 15 -radix hexadecimal}} /IKA87AD_tb/u_dut/reg_MD
add wave -noupdate -group FLAGS /IKA87AD_tb/u_dut/mc_alter_flag
add wave -noupdate -group FLAGS /IKA87AD_tb/u_dut/flag_Z
add wave -noupdate -group FLAGS /IKA87AD_tb/u_dut/flag_SK
add wave -noupdate -group FLAGS /IKA87AD_tb/u_dut/flag_C
add wave -noupdate -group FLAGS /IKA87AD_tb/u_dut/flag_HC
add wave -noupdate -group FLAGS /IKA87AD_tb/u_dut/flag_L1
add wave -noupdate -group FLAGS /IKA87AD_tb/u_dut/flag_L0
add wave -noupdate -group FLAGS /IKA87AD_tb/u_dut/flag_EXX
add wave -noupdate -group FLAGS /IKA87AD_tb/u_dut/flag_EXA
add wave -noupdate -group FLAGS /IKA87AD_tb/u_dut/flag_EXH
add wave -noupdate -group MICROSEQUENCER -radix hexadecimal /IKA87AD_tb/u_dut/reg_OPCODE
add wave -noupdate -group MICROSEQUENCER -radix hexadecimal /IKA87AD_tb/u_dut/opcode_page
add wave -noupdate -group MICROSEQUENCER /IKA87AD_tb/u_dut/mc_end_of_instruction
add wave -noupdate -group MICROSEQUENCER /IKA87AD_tb/u_dut/mseq_state
add wave -noupdate -group MICROSEQUENCER -radix unsigned /IKA87AD_tb/u_dut/mseq_cntr
add wave -noupdate -group MICROSEQUENCER -radix unsigned /IKA87AD_tb/u_dut/mseq_cntr_next
add wave -noupdate -group MICROSEQUENCER -radix unsigned /IKA87AD_tb/u_dut/mcrom_sa
add wave -noupdate -group MICROSEQUENCER -radix unsigned /IKA87AD_tb/u_dut/mcrom_addr
add wave -noupdate -group MICROSEQUENCER /IKA87AD_tb/u_dut/u_microcode/o_MCROM_DATA
add wave -noupdate -group MICROSEQUENCER /IKA87AD_tb/u_dut/mc_ctrl_output
add wave -noupdate -group MICROSEQUENCER /IKA87AD_tb/u_dut/mc_end_of_instruction
add wave -noupdate -group DEU /IKA87AD_tb/u_dut/mc_t0_deu_op
add wave -noupdate -group DEU -radix hexadecimal /IKA87AD_tb/u_dut/deu_dsize
add wave -noupdate -group DEU -radix hexadecimal /IKA87AD_tb/u_dut/deu_pa
add wave -noupdate -group DEU -radix hexadecimal /IKA87AD_tb/u_dut/deu_pb
add wave -noupdate -group DEU -radix hexadecimal /IKA87AD_tb/u_dut/deu_output
add wave -noupdate -group DEU -radix hexadecimal /IKA87AD_tb/u_dut/deu_add_op0
add wave -noupdate -group DEU -radix hexadecimal /IKA87AD_tb/u_dut/deu_add_op1
add wave -noupdate -group DEU -radix hexadecimal /IKA87AD_tb/u_dut/deu_aux_output
add wave -noupdate -group DEU /IKA87AD_tb/u_dut/deu_mul_start
add wave -noupdate -group DEU /IKA87AD_tb/u_dut/deu_div_start
add wave -noupdate -group DEU /IKA87AD_tb/u_dut/deu_muldiv_busy
add wave -noupdate -group DEU -radix unsigned /IKA87AD_tb/u_dut/deu_muldiv_cntr
add wave -noupdate -group DEU -radix hexadecimal /IKA87AD_tb/u_dut/deu_muldiv_r2_temp
add wave -noupdate -group DEU /IKA87AD_tb/u_dut/deu_mul_aux_wr
add wave -noupdate -group DEU /IKA87AD_tb/u_dut/deu_div_aux_wr
add wave -noupdate -group DEU /IKA87AD_tb/u_dut/deu_rot_aux_wr
add wave -noupdate -group DEU -radix hexadecimal /IKA87AD_tb/u_dut/deu_mul_op0
add wave -noupdate -group DEU -radix hexadecimal /IKA87AD_tb/u_dut/deu_mul_op1
add wave -noupdate -group DEU -radix hexadecimal /IKA87AD_tb/u_dut/deu_div_op0
add wave -noupdate -group DEU -radix hexadecimal /IKA87AD_tb/u_dut/deu_div_op1
add wave -noupdate -group REGFILE_WR /IKA87AD_tb/u_dut/reg_V_wr
add wave -noupdate -group REGFILE_WR /IKA87AD_tb/u_dut/reg_B_wr
add wave -noupdate -group REGFILE_WR /IKA87AD_tb/u_dut/reg_C_wr
add wave -noupdate -group REGFILE_WR /IKA87AD_tb/u_dut/reg_D_wr
add wave -noupdate -group REGFILE_WR /IKA87AD_tb/u_dut/reg_E_wr
add wave -noupdate -group REGFILE_WR /IKA87AD_tb/u_dut/reg_H_wr
add wave -noupdate -group REGFILE_WR /IKA87AD_tb/u_dut/reg_L_wr
add wave -noupdate -group REGFILE_WR /IKA87AD_tb/u_dut/deu_muldiv_ea_wr
add wave -noupdate -group REGFILE_WR /IKA87AD_tb/u_dut/reg_A_wr
add wave -noupdate -group REGFILE_WR /IKA87AD_tb/u_dut/reg_EAL_wr
add wave -noupdate -group REGFILE_WR /IKA87AD_tb/u_dut/reg_EAH_wr
add wave -noupdate -group REGFILE_WR /IKA87AD_tb/u_dut/reg_SP_wr
add wave -noupdate -group REGFILE_WR /IKA87AD_tb/u_dut/reg_PC_wr
add wave -noupdate -group REGFILE_WR /IKA87AD_tb/u_dut/reg_SRTMP_wr
add wave -noupdate -group REGFILE_WR /IKA87AD_tb/u_dut/reg_PSW_wr
add wave -noupdate -group REGFILE_WR /IKA87AD_tb/u_dut/reg_MA_wr
add wave -noupdate -group REGFILE_WR /IKA87AD_tb/u_dut/reg_MD0_wr
add wave -noupdate -group REGFILE_WR /IKA87AD_tb/u_dut/reg_MD1_wr
add wave -noupdate -group REGFILE_WR /IKA87AD_tb/u_dut/reg_MD2_wr
add wave -noupdate -expand -group REGFILE /IKA87AD_tb/u_dut/gpr_rw_addr
add wave -noupdate -expand -group REGFILE -radix hexadecimal /IKA87AD_tb/u_dut/gpr_RDBUS
add wave -noupdate -expand -group REGFILE -radix hexadecimal /IKA87AD_tb/u_dut/gpr_WRBUS
add wave -noupdate -expand -group REGFILE -radix hexadecimal /IKA87AD_tb/u_dut/regpair_EAH
add wave -noupdate -expand -group REGFILE -radix hexadecimal /IKA87AD_tb/u_dut/regpair_EAL
add wave -noupdate -expand -group REGFILE -radix hexadecimal -childformat {{{/IKA87AD_tb/u_dut/regpair_V[0]} -radix hexadecimal} {{/IKA87AD_tb/u_dut/regpair_V[1]} -radix hexadecimal}} -subitemconfig {{/IKA87AD_tb/u_dut/regpair_V[0]} {-height 15 -radix hexadecimal} {/IKA87AD_tb/u_dut/regpair_V[1]} {-height 15 -radix hexadecimal}} /IKA87AD_tb/u_dut/regpair_V
add wave -noupdate -expand -group REGFILE -radix hexadecimal -childformat {{{/IKA87AD_tb/u_dut/regpair_A[0]} -radix hexadecimal} {{/IKA87AD_tb/u_dut/regpair_A[1]} -radix hexadecimal}} -subitemconfig {{/IKA87AD_tb/u_dut/regpair_A[0]} {-height 15 -radix hexadecimal} {/IKA87AD_tb/u_dut/regpair_A[1]} {-height 15 -radix hexadecimal}} /IKA87AD_tb/u_dut/regpair_A
add wave -noupdate -expand -group REGFILE -radix hexadecimal /IKA87AD_tb/u_dut/regpair_B
add wave -noupdate -expand -group REGFILE -radix hexadecimal /IKA87AD_tb/u_dut/regpair_C
add wave -noupdate -expand -group REGFILE -radix hexadecimal /IKA87AD_tb/u_dut/regpair_D
add wave -noupdate -expand -group REGFILE -radix hexadecimal /IKA87AD_tb/u_dut/regpair_E
add wave -noupdate -expand -group REGFILE -radix hexadecimal /IKA87AD_tb/u_dut/regpair_H
add wave -noupdate -expand -group REGFILE -radix hexadecimal /IKA87AD_tb/u_dut/regpair_L
add wave -noupdate -group SPR /IKA87AD_tb/u_dut/mc_t2_atype_sel
add wave -noupdate -group SPR /IKA87AD_tb/u_dut/spr_atype
add wave -noupdate -group SPR /IKA87AD_tb/u_dut/reg_SRTMP_wr
add wave -noupdate -group SPR /IKA87AD_tb/u_dut/srtmp_wr_z
add wave -noupdate -group SPR -radix hexadecimal /IKA87AD_tb/u_dut/reg_SRTMP
add wave -noupdate -group SPR -radix hexadecimal /IKA87AD_tb/u_dut/spr_ETM0
add wave -noupdate -group SPR -radix hexadecimal /IKA87AD_tb/u_dut/spr_ETM1
add wave -noupdate -group SPR -radix hexadecimal /IKA87AD_tb/u_dut/spr_PAO
add wave -noupdate -group SPR -radix hexadecimal /IKA87AD_tb/u_dut/spr_PBO
add wave -noupdate -group AEU -radix hexadecimal /IKA87AD_tb/u_dut/aeu_pa
add wave -noupdate -group AEU -radix hexadecimal /IKA87AD_tb/u_dut/aeu_pb
add wave -noupdate -group AEU -radix hexadecimal /IKA87AD_tb/u_dut/aeu_output
add wave -noupdate -group AEU -radix hexadecimal /IKA87AD_tb/u_dut/aeu_add_op0
add wave -noupdate -group AEU -radix hexadecimal /IKA87AD_tb/u_dut/aeu_add_op1
add wave -noupdate -group AEU -radix hexadecimal /IKA87AD_tb/u_dut/aeu_add_out
TreeUpdate [SetDefaultTree]
WaveRestoreCursors {{Cursor 1} {23710 ps} 0}
quietly wave cursor active 1
configure wave -namecolwidth 150
configure wave -valuecolwidth 175
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
configure wave -timelineunits ns
update
WaveRestoreZoom {22630 ps} {32430 ps}
