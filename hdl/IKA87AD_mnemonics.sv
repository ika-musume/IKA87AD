//bus cycle types
localparam IDLE = 2'b00;
localparam RD4 = 2'b01;
localparam RD3 = 2'b10;
localparam WR3 = 2'b11;

//microcode types
localparam MCTYPE0 = 2'd0;
localparam MCTYPE1 = 2'd1;
localparam MCTYPE2 = 2'd2;
localparam MCTYPE3 = 2'd3;

//general purpose registers read bus
localparam GPRBUS_xV_b = 5'b00000;
localparam GPRBUS_xA_b = 5'b00001;
localparam GPRBUS_VA_w = 5'b00010;
localparam GPRBUS_xB_b = 5'b00100;
localparam GPRBUS_xC_b = 5'b00101;
localparam GPRBUS_BC_w = 5'b00110;
localparam GPRBUS_xD_b = 5'b01000;
localparam GPRBUS_xE_b = 5'b01001;
localparam GPRBUS_DE_w = 5'b01010;
localparam GPRBUS_xH_b = 5'b01100;
localparam GPRBUS_xL_b = 5'b01101;
localparam GPRBUS_HL_w = 5'b01110;
localparam GPRBUS_EH_b = 5'b10000;
localparam GPRBUS_EL_b = 5'b10001;
localparam GPRBUS_EA_w = 5'b10010;
localparam GPRBUS_SP_w = 5'b11010;


//MICROCODE TYPE 0 FIELDS
localparam T0_SRC_R         = 4'b0000;
localparam T0_SRC_R2        = 4'b0001;
localparam T0_SRC_R1        = 4'b0010;
localparam T0_SRC_RP2       = 4'b0011;
localparam T0_SRC_RP        = 4'b0100;
localparam T0_SRC_RP1       = 4'b0101;
localparam T0_SRC_SR        = 4'b0110;
localparam T0_SRC_PSW       = 4'b0111;
localparam T0_SRC_MD        = 4'b1000;
localparam T0_SRC_MD0       = 4'b1001;
localparam T0_SRC_AUX       = 4'b1011;
localparam T0_SRC_A         = 4'b1100;
localparam T0_SRC_EA        = 4'b1110;
/* -------------------------------- */
localparam T0_DST_R         = 4'b0000;
localparam T0_DST_R2        = 4'b0001;
localparam T0_DST_R1        = 4'b0010;
localparam T0_DST_RP2       = 4'b0011;
localparam T0_DST_RP        = 4'b0100;
localparam T0_DST_RP1       = 4'b0101;
localparam T0_DST_SR        = 4'b0110;
localparam T0_DST_PSW       = 4'b0111;
localparam T0_DST_MD        = 4'b1000;
localparam T0_DST_MD0       = 4'b1001;
localparam T0_DST_MD1       = 4'b1010;
localparam T0_DST_A         = 4'b1100;
localparam T0_DST_C         = 4'b1101;
localparam T0_DST_EA        = 4'b1110;
localparam T0_DST_BC        = 4'b1111;
/* -------------------------------- */
localparam T0_DEU_BYPASS    = 4'b0000;
localparam T0_DEU_INC       = 4'b0001;
localparam T0_DEU_DEC       = 4'b0010;
localparam T0_DEU_DAA       = 4'b0011;
localparam T0_DEU_ROT       = 4'b0100;
localparam T0_DEU_SHFT      = 4'b0101;
localparam T0_DEU_MUL       = 4'b0110;
localparam T0_DEU_DIV       = 4'b0111;
localparam T0_DEU_COMMON    = 4'b1000;

//MICROCODE TYPE 1 FIELDS
localparam T1_SRC_A_IM      = 4'b0000;
localparam T1_SRC_A_V_WA    = 4'b0001;
localparam T1_SRC_A_TA      = 4'b0010;
localparam T1_SRC_A_FA      = 4'b0011;
localparam T1_SRC_A_REL_S   = 4'b0100;
localparam T1_SRC_A_REL_L   = 4'b0101;
localparam T1_SRC_A_INT     = 4'b0110;
localparam T1_SRC_RPA_OFFSET= 4'b0111;
localparam T1_SRC_RPA       = 4'b1000;
localparam T1_SRC_RPA2      = 4'b1001;
localparam T1_SRC_BC        = 4'b1010;
localparam T1_SRC_DE        = 4'b1011;
localparam T1_SRC_HL        = 4'b1100;
localparam T1_SRC_SP        = 4'b1101;
localparam T1_SRC_PC        = 4'b1110;
localparam T1_SRC_MA        = 4'b1111;
/* -------------------------------- */
localparam T1_DST_RPA       = 4'b1000;
localparam T1_DST_MD        = 4'b1101;
localparam T1_DST_PC        = 4'b1110;
localparam T1_DST_MA        = 4'b1111;
/* -------------------------------- */
localparam T1_AEU_BYPASS    = 4'b0000;
localparam T1_AEU_ADD       = 4'b0001;
localparam T1_AEU_RPA_ADJ   = 4'b0010;
localparam T1_AEU_RPA3_ADJ  = 4'b0011;
localparam T1_AEU_PUSH      = 4'b0100;
localparam T1_AEU_POP       = 4'b0101;


//MICROCODE ROUTINE ENTRANCE ADDRESS

//2-cycle opcode group
localparam MVI_R_IM         = 8'd000;
localparam LXI_RP2_IM       = 8'd002;
localparam STAX_RPA_A       = 8'd004;
localparam LDAX_A_RPA       = 8'd006;
localparam INX_RP2          = 8'd008; //RP2 actually
localparam INX_EA           = 8'd010;
localparam DCX_RP2          = 8'd012;
localparam DCX_EA           = 8'd014;
localparam JMP              = 8'd016;
localparam JR               = 8'd018;
localparam ALUX_A_RPA       = 8'd020;
localparam ALUI_A_IM        = 8'd022;
localparam ALUI_R_IM        = 8'd024;
localparam EALU_EA_R2       = 8'd026;
localparam DALU_EA_RP       = 8'd028;

//4-cycle opcode group
localparam MOV_MEM_R        = 8'd032;
localparam MVIW_WA_IM       = 8'd036;
localparam STAX_RPA2_A      = 8'd040;
localparam LDAX_A_RPA2      = 8'd044;
localparam LD_RP2_MEM       = 8'd048; //LBCD, LDED, LHLD, LSPD
localparam BLOCK            = 8'd052;
localparam TABLE            = 8'd056;
localparam RLD_RRD          = 8'd060;
localparam PUSH             = 8'd064;
localparam CALB             = 8'd068;
localparam CALF             = 8'd072;
localparam CALL             = 8'd076;
localparam CALT             = 8'd080;
localparam RETI             = 8'd084;

//3-cycle and 5-cycle opcode group
localparam MOV_SR_A         = 8'd088; //3
localparam ST_MEM_RP2       = 8'd091; //5
localparam MOV_A_SR1        = 8'd096;
localparam INRW             = 8'd099;
localparam MOV_R_MEM        = 8'd104;
localparam DCRW             = 8'd107;
localparam STEAX_RPA_EA     = 8'd112;
localparam STEAX_RPA2_EA    = 8'd115;
localparam LDEAX_EA_RPA     = 8'd120;
localparam LDEAX_EA_RPA2    = 8'd123;
localparam MVIX_RPA_IM      = 8'd128;
localparam SOFTI            = 8'd131;
localparam POP              = 8'd136;
localparam HARDI            = 8'd139;

//3-cycle and 1-cycle opcode group
localparam STAW             = 8'd144;
localparam MOV_R1_A         = 8'd147;
localparam LDAW             = 8'd148;
localparam MOV_A_R1         = 8'd151;
localparam MUL              = 8'd152;
localparam DMOV_RP_EA       = 8'd155;
localparam DIV              = 8'd156;
localparam DMOV_EA_RP       = 8'd159;
localparam DMOV_SR3_EA      = 8'd160;
localparam INR              = 8'd163;
localparam DMOV_EA_SR4      = 8'd164;
localparam DCR              = 8'd167;
localparam ALUW_A_WA        = 8'd168;
localparam DAA              = 8'd171;
localparam JRE              = 8'd172;
localparam NEGA             = 8'd175;
localparam RET_RETS         = 8'd176;
localparam STC_CLC          = 8'd179;
localparam BIT              = 8'd180;
localparam EDI              = 8'd183;

//opcode with branch microcode
localparam ALUIW_WA_IM      = 8'd184;
localparam ALUI_SR2_IM      = 8'd192;

//1-cycle opcode group
localparam ALU_A_R          = 8'd200;
localparam ALU_R_A          = 8'd201;
localparam BYTE_RS_R2       = 8'd202;
localparam WORD_RS_EA       = 8'd203;
localparam JB               = 8'd204;
localparam JEA              = 8'd205;
localparam EXX              = 8'd206;
localparam EXA              = 8'd207;
localparam EXH              = 8'd208;
localparam SK               = 8'd209;
localparam SKN              = 8'd210;
localparam SKIT             = 8'd211;
localparam SKNIT            = 8'd212;
localparam NOP              = 8'd213;
localparam SUSP             = 8'd214; //HLT or STOP

//instruction read(similar to nop but cannot be skipped)
localparam IRD              = 8'd255;