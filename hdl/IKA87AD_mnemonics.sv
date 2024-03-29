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

//MICROCODE TYPE 0 FIELDS
//source a/destination types
localparam SA_DST_R      = 5'b00000;
localparam SA_DST_R2     = 5'b00001;
localparam SA_DST_R1     = 5'b00010;
localparam SA_DST_RP2    = 5'b00011;
localparam SA_DST_RP     = 5'b00100;
localparam SA_DST_RP1    = 5'b00101;
localparam SA_DST_C      = 5'b00110;
localparam SA_DST_SP     = 5'b00111;
localparam SA_DST_SR3    = 5'b01000;
localparam SA_DST_MDL    = 5'b01001;
localparam SA_DST_MD     = 5'b01010;
localparam SA_DST_MA     = 5'b01011;
localparam SA_DST_A      = 5'b01100;
localparam SA_DST_EA     = 5'b01101;
localparam SA_DST_PC     = 5'b01110;
localparam SA_DST_SR_SR1 = 5'b01111;
localparam SA_DST_SR2    = 5'b10000;


//source b types
localparam SB_R          = 5'b00000;
localparam SB_R2         = 5'b00001;
localparam SB_R1         = 5'b00010;
localparam SB_RP2        = 5'b00011;
localparam SB_RP         = 5'b00100;
localparam SB_RP1        = 5'b00101;
localparam SB_RPA        = 5'b00110;
localparam SB_RPA2       = 5'b00111;
localparam SB_SR4        = 5'b01000;
localparam SB_MDH        = 5'b01001;
localparam SB_MD         = 5'b01010;
localparam SB_MDI        = 5'b01011;
localparam SB_A          = 5'b01100;
localparam SB_EA         = 5'b01101;
localparam SB_ADDR_V_WA  = 5'b10001;
localparam SB_ADDR_TA    = 5'b10010;
localparam SB_ADDR_FA    = 5'b10011;
localparam SB_ADDR_REL_S = 5'b10100;
localparam SB_ADDR_REL_L = 5'b10101;
localparam SB_ADDR_INT   = 5'b10110;
localparam SB_SUB2       = 5'b10111;
localparam SB_SUB1       = 5'b11000;
localparam SB_ZERO       = 5'b11001;
localparam SB_ADD1       = 5'b11010;
localparam SB_ADD2       = 5'b11011;
localparam SB_TEMP       = 5'b11100;
localparam SB_SR_SR1     = 5'b11101;
localparam SB_SR2        = 5'b11110;
localparam SB_OFFSET     = 5'b11111;

//MICROCODE TYPE 1 FIELDS
//source c types
localparam SC_DST_BC     = 4'b0000;
localparam SC_DST_R2     = 4'b0001;
localparam SC_DST_NOWHERE = 4'b0111;
localparam SC_DST_MA     = 4'b1000;
localparam SC_DST_MD     = 4'b1001;
localparam SC_DST_MDL    = 4'b1010;
localparam SC_DST_MDH    = 4'b1011;
localparam SC_DST_A      = 4'b1100;
localparam SC_DST_EA     = 4'b1101;
localparam SC_DST_PSW    = 4'b1110;
localparam SC_DST_PC     = 4'b1111;


//source d types
localparam SD_BC         = 4'b0000;
localparam SD_DE         = 4'b0001;
localparam SD_HL         = 4'b0010;
localparam SD_SP         = 4'b0011;
localparam SD_RPA        = 4'b0100;
localparam SD_R2         = 4'b0101;
localparam SD_PC         = 4'b1000;
localparam SD_PSW        = 4'b1001;
localparam SD_MDH        = 4'b1010;
localparam SD_MD         = 4'b1011;
localparam SD_A          = 4'b1100;
localparam SD_EA         = 4'b1101;
localparam SD_NOSOURCE   = 4'b1111;

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