`include "IKA87AD_mnemonics.sv"

module IKA87AD_microcode (
    input   wire                i_CLK,
    input   wire                i_MCROM_READ_TICK,
    input   wire    [7:0]       i_MCROM_ADDR,
    output  wire    [17:0]      o_MCROM_DATA
);

reg     [17:0]  mc;
assign  o_MCROM_DATA = mc;

always @(posedge i_CLK) if(i_MCROM_READ_TICK) begin
    case(i_MCROM_ADDR)
        //                       MCTYPE   FLAG  SKIP
        /*
            8-bit data transfer instructions
        */
        //MOV_R1_A        : mc <= {MCTYPE0, 1'b0, 1'b1, SB_A, SA_DST_R1, 2'b00, RD4};                     //r1<-A, RD4
        //MOV_A_R1        : mc <= {MCTYPE0, 1'b0, 1'b1, SB_R1, SA_DST_A, 2'b00, RD4};                     //A<-r1, RD4
        //MOV_SR_A        : mc <= {MCTYPE3, 1'b0, 1'b0, 5'b10000, 1'b0, 1'b0, 3'b000, 1'b0, 1'b0, RD3};   //nop, RD3
        //MOV_SR_A+1      : mc <= {MCTYPE3, 1'b0, 1'b1, 5'b10000, 1'b0, 1'b0, 3'b000, 1'b0, 1'b0, IDLE};  //nop, IDLE
        //MOV_SR_A+2      : mc <= {MCTYPE0, 1'b0, 1'b0, SB_A, SA_DST_SR_SR1, 2'b00, RD4};                 //sr<-A, RD4
        //MOV_A_SR1       : mc <= {MCTYPE3, 1'b0, 1'b0, 5'b10000, 1'b0, 1'b0, 3'b000, 1'b0, 1'b0, RD3};   //nop, RD3
        //MOV_A_SR1+1     : mc <= {MCTYPE3, 1'b0, 1'b1, 5'b10000, 1'b0, 1'b0, 3'b000, 1'b0, 1'b0, IDLE};  //nop, IDLE
        //MOV_A_SR1+2     : mc <= {MCTYPE0, 1'b0, 1'b0, SB_SR_SR1, SA_DST_A, 2'b00, RD4};                 //sr<-A, RD4
        //MOV_R_MEM       : mc <= {MCTYPE3, 1'b0, 1'b0, 5'b10001, 1'b0, 1'b0, 3'b000, 1'b0, 1'b0, RD3};   //nop, RD3
        //MOV_R_MEM+1     : mc <= {MCTYPE3, 1'b0, 1'b0, 5'b10001, 1'b0, 1'b0, 3'b000, 1'b0, 1'b0, RD3};   //nop, RD3
        //MOV_R_MEM+2     : mc <= {MCTYPE0, 1'b0, 1'b1, SB_MD, SA_DST_MA, 2'b00, RD3};                    //MA<-MD, RD3
        //MOV_R_MEM+3     : mc <= {MCTYPE0, 1'b0, 1'b0, SB_MD, SA_DST_R, 2'b00, RD4};                     //r<-MD, RD4
        //MOV_MEM_R       : mc <= {MCTYPE0, 1'b0, 1'b0, SB_R, SA_DST_MDL, 2'b00, RD3};                    //MD<-R
        //MOV_MEM_R+1     : mc <= {MCTYPE3, 1'b0, 1'b0, 5'b10000, 1'b0, 1'b0, 3'b000, 1'b0, 1'b0, RD3};   //nop, RD3
        //MOV_MEM_R+2     : mc <= {MCTYPE0, 1'b0, 1'b1, SB_MDI, SA_DST_MA, 2'b00, WR3};                   //MA<-MDI, WR3
        //MOV_MEM_R+3     : mc <= {MCTYPE3, 1'b0, 1'b0, 5'b10000, 1'b0, 1'b0, 3'b000, 1'b0, 1'b0, RD4};   //nop, RD4

        MVI_R_IM        : mc <= {MCTYPE2, 1'b0, 1'b0, T2_NOP, RD3};                                     //nop, RD3
        MVI_R_IM+1      : mc <= {MCTYPE0, 1'b1, 1'b1, T0_SRC_MD, T0_DST_R, T0_DEU_MOV, RD4};            //R<-MD, RD4

        // * MVI_SR2_IM is included in ALUI_SR2_IM instruction

        //MVIW_WA_IM      : mc <= {MCTYPE3, 1'b0, 1'b0, 5'b00000, 1'b0, 1'b0, 3'b000, 1'b1, 1'b0, RD3};   //swap MD output order, RD3
        //MVIW_WA_IM+1    : mc <= {MCTYPE3, 1'b0, 1'b0, 5'b10000, 1'b0, 1'b0, 3'b000, 1'b0, 1'b0, RD3};   //nop, RD3
        //MVIW_WA_IM+2    : mc <= {MCTYPE0, 1'b0, 1'b1, SB_ADDR_V_WA, SA_DST_MA, 2'b00, WR3};             //MA<-Vwa, WR3
        //MVIW_WA_IM+3    : mc <= {MCTYPE3, 1'b0, 1'b0, 5'b10000, 1'b0, 1'b0, 3'b000, 1'b0, 1'b0, RD4};   //nop, RD4
        //MVIX_RPA_IM     : mc <= {MCTYPE3, 1'b0, 1'b0, 5'b10000, 1'b0, 1'b0, 3'b000, 1'b0, 1'b0, RD3};   //nop, RD3
        //MVIX_RPA_IM+1   : mc <= {MCTYPE0, 1'b0, 1'b1, SB_RPA, SA_DST_MA, 2'b00, WR3};                   //MA<-RPA, WR3
        //MVIX_RPA_IM+2   : mc <= {MCTYPE3, 1'b0, 1'b0, 5'b10000, 1'b0, 1'b0, 3'b000, 1'b0, 1'b0, RD4};   //nop, RD4
        //LDAW            : mc <= {MCTYPE3, 1'b0, 1'b0, 5'b10000, 1'b0, 1'b0, 3'b000, 1'b0, 1'b0, RD3};   //nop, RD3
        //LDAW+1          : mc <= {MCTYPE0, 1'b0, 1'b1, SB_ADDR_V_WA, SA_DST_MA, 2'b00, RD3};             //MA<-Vwa, RD3
        //LDAW+2          : mc <= {MCTYPE0, 1'b0, 1'b0, SB_MDH, SA_DST_A, 2'b00, RD4};                    //A<-MD, RD4
        //STAW            : mc <= {MCTYPE0, 1'b0, 1'b0, SB_A, SA_DST_MDL, 2'b00, RD3};                    //MD<-A
        //STAW+1          : mc <= {MCTYPE0, 1'b0, 1'b1, SB_ADDR_V_WA, SA_DST_MA, 2'b00, WR3};             //MA<-Vwa, WR3
        //STAW+2          : mc <= {MCTYPE3, 1'b0, 1'b0, 5'b10000, 1'b0, 1'b0, 3'b000, 1'b0, 1'b0, RD4};   //nop, RD4
        //LDAX_A_RPA      : mc <= {MCTYPE1, 1'b0, 1'b1, SD_RPA, SC_DST_MA, 4'b1010, RD3};                 //MA<-RPA, WR3
        //LDAX_A_RPA+1    : mc <= {MCTYPE0, 1'b1, 1'b0, SB_MD, SA_DST_A, 2'b00, RD4};                     //A<-MD, RD4
        //STAX_RPA_A      : mc <= {MCTYPE1, 1'b0, 1'b1, SD_RPA, SC_DST_MA, 4'b1010, WR3};                 //MA<-RPA, WR3 //A will be loaded into MD automatically
        //STAX_RPA_A+1    : mc <= {MCTYPE3, 1'b0, 1'b0, 5'b10000, 1'b0, 1'b0, 3'b000, 1'b0, 1'b0, RD4};   //nop, RD4
        //LDAX_A_RPA2     : mc <= {MCTYPE3, 1'b0, 1'b0, 5'b00000, 1'b0, 1'b1, 3'b000, 1'b0, 1'b0, IDLE};  //conditional read, IDLE
        //LDAX_A_RPA2+1   : mc <= {MCTYPE0, 1'b0, 1'b1, SB_OFFSET, SA_DST_MA, 2'b00, IDLE};               //MA<-RPA_OFFSET, IDLE
        //LDAX_A_RPA2+2   : mc <= {MCTYPE0, 1'b0, 1'b0, SB_RPA2, SA_DST_MA, 2'b01, RD3};                  //MA<-MA+RPA2, RD3
        //LDAX_A_RPA2+3   : mc <= {MCTYPE0, 1'b0, 1'b0, SB_MD, SA_DST_A, 2'b00, RD4};                     //A<-MD
        //STAX_RPA2_A     : mc <= {MCTYPE3, 1'b0, 1'b0, 5'b00000, 1'b0, 1'b1, 3'b000, 1'b0, 1'b0, IDLE};  //conditional read, IDLE
        //STAX_RPA2_A+1   : mc <= {MCTYPE0, 1'b0, 1'b1, SB_OFFSET, SA_DST_MA, 2'b00, IDLE};               //MA<-RPA_OFFSET, IDLE
        //STAX_RPA2_A+2   : mc <= {MCTYPE0, 1'b0, 1'b0, SB_RPA2, SA_DST_MA, 2'b01, WR3};                  //MA<-MA+RPA2, WR3
        //STAX_RPA2_A+3   : mc <= {MCTYPE3, 1'b0, 1'b0, 5'b10000, 1'b0, 1'b0, 3'b000, 1'b0, 1'b0, RD4};   //nop, RD4
        //EXX             : mc <= {MCTYPE2, 1'b0, 1'b1, 4'b0000, 1'b0, 1'b0, 2'b01, 1'b0, 3'b000, RD4};   //exx mod
        //EXA             : mc <= {MCTYPE2, 1'b0, 1'b1, 4'b0000, 1'b0, 1'b0, 2'b10, 1'b0, 3'b000, RD4};   //exa mod
        //EXH             : mc <= {MCTYPE2, 1'b0, 1'b1, 4'b0000, 1'b0, 1'b0, 2'b11, 1'b0, 3'b000, RD4};   //exh mod
        //BLOCK           : mc <= {MCTYPE1, 1'b0, 1'b1, SD_HL, SC_DST_MA, 4'b1001, RD3};                  //MA<-pop HL, RD3
        //BLOCK+1         : mc <= {MCTYPE1, 1'b0, 1'b0, SD_DE, SC_DST_MA, 4'b1001, WR3};                  //MA<-pop DE, WR3
        //BLOCK+2         : mc <= {MCTYPE0, 1'b0, 1'b0, SB_SUB1, SA_DST_C, 2'b01, IDLE};                  //C<-C-1(dec), IDLE
        //BLOCK+3         : mc <= {MCTYPE3, 1'b0, 1'b0, 5'b00000, 1'b1, 1'b0, 3'b000, 1'b0, 1'b0, RD4};   //conditional PC decrement, RD4


        /*
            16-bit data transfer instructions
        */
        DMOV_RP_EA      : mc <= {MCTYPE0, 1'b0, 1'b1, T0_SRC_EA, T0_DST_RP, T0_DEU_MOV, RD4};           //rp<-EA, RD4
        
        DMOV_EA_RP      : mc <= {MCTYPE0, 1'b0, 1'b1, T0_SRC_RP, T0_DST_EA, T0_DEU_MOV, RD4};           //EA<-rp, RD4
        
        DMOV_SR3_EA     : mc <= {MCTYPE2, 1'b0, 1'b0, T2_ST_SR3, IDLE};                                 //STSR3, IDLE
        DMOV_SR3_EA+1   : mc <= {MCTYPE0, 1'b0, 1'b1, T0_SRC_EA, T0_DST_SRTMP, T0_DEU_MOV, IDLE};       //SRTMP<-EA, IDLE
        DMOV_SR3_EA+2   : mc <= {MCTYPE2, 1'b0, 1'b0, T2_NOP, RD4};                                     //nop(writeback delay), RD4
        
        DMOV_EA_SR4     : mc <= {MCTYPE2, 1'b0, 1'b0, T2_ST_SR4, IDLE};                                 //STSR4, IDLE
        DMOV_EA_SR4+1   : mc <= {MCTYPE2, 1'b0, 1'b1, T2_NOP, IDLE};                                    //nop(read delay), IDLE
        DMOV_EA_SR4+2   : mc <= {MCTYPE0, 1'b0, 1'b0, T0_SRC_SRTMP, T0_DST_EA, T0_DEU_MOV, RD4};        //EA<-sr4, RD4
        
        LXI_RP2_IM      : mc <= {MCTYPE2, 1'b0, 1'b0, T2_NOP, RD3};                                     //nop, RD3
        LXI_RP2_IM+1    : mc <= {MCTYPE2, 1'b0, 1'b0, T2_NOP, RD3};                                     //nop, RD3
        LXI_RP2_IM+2    : mc <= {MCTYPE0, 1'b1, 1'b1, T0_SRC_MD, T0_DST_RP2, T0_DEU_MOV, RD4};          //rp2<-MD, RD4
        
        LD_RP2_MEM      : mc <= {MCTYPE2, 1'b0, 1'b0, T2_ST_IA, RD3};                                   //STIA, RD3
        LD_RP2_MEM+1    : mc <= {MCTYPE2, 1'b0, 1'b0, T2_NOP, RD3};                                     //nop, RD3
        LD_RP2_MEM+2    : mc <= {MCTYPE1, 1'b0, 1'b1, T1_SRC_A_IM, T1_DST_MA, T1_AEU_MOV, RD3};         //MA<-ADDR_IA, RD3
        LD_RP2_MEM+3    : mc <= {MCTYPE2, 1'b0, 1'b0, T2_NOP, RD3};                                     //nop, RD3
        LD_RP2_MEM+4    : mc <= {MCTYPE0, 1'b0, 1'b0, T0_SRC_MD, T0_DST_RP2, T0_DEU_MOV, RD4};          //rp2<-MD, RD4
        
        ST_MEM_RP2      : mc <= {MCTYPE2, 1'b0, 1'b0, T2_ST_IA, RD3};                                   //STIA, RD3
        ST_MEM_RP2+1    : mc <= {MCTYPE2, 1'b0, 1'b0, T2_NOP, RD3};                                     //nop, RD3
        ST_MEM_RP2+2    : mc <= {MCTYPE1, 1'b0, 1'b1, T1_SRC_A_IM, T1_DST_MA, T1_AEU_MOV, WR3};         //MA<-ADDR_IA, WR3
        ST_MEM_RP2+3    : mc <= {MCTYPE0, 1'b0, 1'b0, T0_SRC_RP2, T0_DST_MD, T0_DEU_MOV, WR3};          //MD<-rp2, WR3
        ST_MEM_RP2+4    : mc <= {MCTYPE2, 1'b0, 1'b0, T2_NOP, RD4};                                     //nop, RD4
        
        LDEAX_EA_RPA    : mc <= {MCTYPE1, 1'b0, 1'b1, T1_SRC_RPA, T1_DST_MA, T1_AEU_MOV, RD3};          //MA<-RPA, RD3
        LDEAX_EA_RPA+1  : mc <= {MCTYPE1, 1'b0, 1'b0, T1_SRC_RPA, T1_DST_RPA, T1_AEU_RPA3_ADJ, RD3};    //RPA<-adj RPA, RD3
        LDEAX_EA_RPA+2  : mc <= {MCTYPE0, 1'b0, 1'b0, T0_SRC_MD, T0_DST_EA, T0_DEU_MOV, RD4};           //EA<-MD, RD4

        LDEAX_EA_RPA2   : mc <= {MCTYPE2, 1'b0, 1'b1, T2_ST_RIO, IDLE};                                 //STRIO(conditional), IDLE
        LDEAX_EA_RPA2+1 : mc <= {MCTYPE1, 1'b0, 1'b0, T1_SRC_RPA2, T1_DST_MA, T1_AEU_MOV, IDLE};        //MA<-RPA2, IDLE
        LDEAX_EA_RPA2+2 : mc <= {MCTYPE1, 1'b0, 1'b0, T1_SRC_RPA_OFFSET, T1_DST_MA, T1_AEU_ADD, RD3};   //MA<-MA+RPA_OFFSET, RD3
        LDEAX_EA_RPA2+3 : mc <= {MCTYPE2, 1'b0, 1'b0, T2_NOP, RD3};                                     //nop, RD3
        LDEAX_EA_RPA2+4 : mc <= {MCTYPE0, 1'b0, 1'b0, T0_SRC_MD, T0_DST_EA, T0_DEU_MOV, RD4};           //EA<-MD, RD4

        STEAX_RPA_EA    : mc <= {MCTYPE1, 1'b0, 1'b1, T1_SRC_RPA, T1_DST_MA, T1_AEU_MOV, WR3};          //MA<-RPA, WR3
        STEAX_RPA_EA+1  : mc <= {MCTYPE0, 1'b0, 1'b0, T0_SRC_EA, T0_DST_MD, T0_DEU_MOV, WR3};           //MD<-EA, WR3
        STEAX_RPA_EA+2  : mc <= {MCTYPE1, 1'b0, 1'b0, T1_SRC_RPA, T1_DST_RPA, T1_AEU_RPA3_ADJ, RD4};    //RPA<-adj RPA, RD4

        STEAX_RPA2_EA   : mc <= {MCTYPE2, 1'b0, 1'b1, T2_ST_RIO, IDLE};                                 //STRIO(conditional), IDLE
        STEAX_RPA2_EA+1 : mc <= {MCTYPE1, 1'b0, 1'b0, T1_SRC_RPA2, T1_DST_MA, T1_AEU_MOV, IDLE};        //MA<-RPA2, IDLE
        STEAX_RPA2_EA+2 : mc <= {MCTYPE1, 1'b0, 1'b0, T1_SRC_RPA_OFFSET, T1_DST_MA, T1_AEU_ADD, WR3};   //MA<-MA+RPA_OFFSET, WR3
        STEAX_RPA2_EA+3 : mc <= {MCTYPE0, 1'b0, 1'b0, T0_SRC_EA, T0_DST_MD, T0_DEU_MOV, WR3};           //MD<-EA, WR3
        STEAX_RPA2_EA+4 : mc <= {MCTYPE2, 1'b0, 1'b0, T2_NOP, RD4};                                     //nop, RD4

        PUSH            : mc <= {MCTYPE0, 1'b0, 1'b1, T0_SRC_RP1, T0_DST_MD, T0_DEU_MOV, IDLE};         //MD<-rp1, IDLE
        PUSH+1          : mc <= {MCTYPE1, 1'b0, 1'b0, T1_SRC_SP, T1_DST_MA, T1_AEU_PUSH, WR3};          //MA<-push SP, WR3
        PUSH+2          : mc <= {MCTYPE1, 1'b0, 1'b0, T1_SRC_SP, T1_DST_MA, T1_AEU_PUSH, WR3};          //MA<-push SP, WR3
        PUSH+3          : mc <= {MCTYPE2, 1'b0, 1'b0, T2_NOP, RD4};                                     //nop, RD4

        POP             : mc <= {MCTYPE1, 1'b0, 1'b1, T1_SRC_SP, T1_DST_MA, T1_AEU_POP, RD3};           //MA<-pop SP, RD3
        POP+1           : mc <= {MCTYPE1, 1'b0, 1'b0, T1_SRC_SP, T1_DST_MA, T1_AEU_POP, RD3};           //MA<-pop SP, RD3
        POP+2           : mc <= {MCTYPE0, 1'b0, 1'b0, T0_SRC_MD, T0_DST_RP1, T0_DEU_MOV, RD4};          //rp1<-MD, RD4

        TABLE           : mc <= {MCTYPE1, 1'b0, 1'b1, T1_SRC_RPA_OFFSET, T1_DST_MA, T1_AEU_ADD, IDLE};  //MA<-MA+A(rpa offset A, opcode[1:0]==00), IDLE
        TABLE+1         : mc <= {MCTYPE1, 1'b0, 1'b0, T1_SRC_MA, T1_DST_MA, T1_AEU_DINC, RD3};          //MA<-MA+2, IDLE
        TABLE+2         : mc <= {MCTYPE2, 1'b0, 1'b0, T2_NOP, RD3};                                     //nop, RD3
        TABLE+3         : mc <= {MCTYPE0, 1'b0, 1'b0, T0_SRC_MD, T0_DST_BC, T0_DEU_MOV, RD4};           //BC<-MD, RD4
        
        /*
            8-bit register-accumulator arithmetic instructions
            (add, adc, addnc, sub, subnc, ana, ora, xra, gta, lta, nea, eqa, ona, offa)
        */
        ALU_A_R         : mc <= {MCTYPE0, 1'b1, 1'b1, T0_SRC_R, T0_DST_A, T0_DEU_COMOP, RD4};           //A<-A(op)R, RD4
        ALU_R_A         : mc <= {MCTYPE0, 1'b1, 1'b1, T0_SRC_A, T0_DST_R, T0_DEU_COMOP, RD4};           //R<-R(op)A, RD4

        /*
            8-bit memory-accumulator arithmetic instructions
            (addx, adcx, addncx, subx, sbbx, subncx, anax, orax, xrax, gtax, ltax, neax, eqax, onax, offax)
        */
        //ALUX_A_RPA      : mc <= {MCTYPE1, 1'b0, 1'b1, SD_RPA, SC_DST_MA, 4'b1010, RD3};                 //MA<-RPA
        //ALUX_A_RPA+1    : mc <= {MCTYPE0, 1'b1, 1'b0, SB_MD, SA_DST_A, 2'b10, RD4};                     //A<-MD
        
        /*
            8-bit immediate-register arithmetic instructions
            (adi, aci, adinc, sui, sbi, suinb, ani, ori, xri, gti, lti, nei, eqi, oni, offi)
        */
        //ALUI_A_IM       : mc <= {MCTYPE3, 1'b0, 1'b0, 5'b10000, 1'b0, 1'b0, 3'b000, 1'b0, 1'b0, RD3};   //nop, RD3
        //ALUI_A_IM+1     : mc <= {MCTYPE0, 1'b1, 1'b1, SB_MD, SA_DST_A, 2'b11, RD4};                     //A<-MD

        //ALUI_R_IM       : mc <= {MCTYPE3, 1'b0, 1'b0, 5'b10000, 1'b0, 1'b0, 3'b000, 1'b0, 1'b0, RD3};   //nop,// RD3
        //ALUI_R_IM+1     : mc <= {MCTYPE0, 1'b1, 1'b1, SB_MD, SA_DST_R, 2'b10, RD4};                     //R<-M//D

        //ALUI_SR2_IM     : mc <= {MCTYPE3, 1'b0, 1'b0, 5'b10000, 1'b0, 1'b0, 3'b000, 1'b0, 1'b0, RD3};   //nop, RD3(data, low)
        //ALUI_SR2_IM+1   : mc <= {MCTYPE3, 1'b0, 1'b1, 5'b00000, 1'b0, 1'b0, 3'b001, 1'b0, 1'b0, IDLE};  //branch+2, IDLE
        //ALUI_SR2_IM+2   : mc <= {MCTYPE3, 1'b0, 1'b0, 5'b10001, 1'b0, 1'b0, 3'b000, 1'b0, 1'b0, IDLE};  //nop*2, IDLE
        //ALUI_SR2_IM+3   : mc <= {MCTYPE0, 1'b1, 1'b0, SB_MD, SA_DST_SR2, 2'b10, RD4};                   //SR2<-SR2(op)MD, RD4

        /*
            8-bit working register arithmetic instructions
            (aniw, oriw(19 cyc) / gti, lti, oni, offi, eqi, nei(13 cyc))
        */
        //ALUIW_WA_IM     : mc <= {MCTYPE3, 1'b0, 1'b0, 5'b10000, 1'b0, 1'b0, 3'b000, 1'b0, 1'b0, RD3};   //nop, RD3(Vwa addr, low)
        //ALUIW_WA_IM+1   : mc <= {MCTYPE3, 1'b0, 1'b0, 5'b00000, 1'b0, 1'b0, 3'b100, 1'b0, 1'b0, RD3};   //branch+5, RD3(im, high)
        //ALUIW_WA_IM+2   : mc <= {MCTYPE0, 1'b0, 1'b1, SB_ADDR_V_WA, SA_DST_MA, 2'b00, RD3};             //MA<-Vwa, RD3(Vwa data, low)
        //ALUIW_WA_IM+3   : mc <= {MCTYPE0, 1'b1, 1'b0, SB_MDH, SA_DST_MDL, 2'b11, IDLE};                 //MDL<-MDL(op)MDH, IDLE
        //ALUIW_WA_IM+4   : mc <= {MCTYPE0, 1'b0, 1'b0, SB_SUB1, SA_DST_MA, 2'b01, WR3};                  //MA<-MA-1, WR3
        //ALUIW_WA_IM+5   : mc <= {MCTYPE3, 1'b0, 1'b0, 5'b10000, 1'b0, 1'b0, 3'b000, 1'b0, 1'b0, RD4};   //nop, RD4
        //ALUIW_WA_IM+6   : mc <= {MCTYPE0, 1'b0, 1'b1, SB_ADDR_V_WA, SA_DST_MA, 2'b00, RD3};             //MA<-Vwa, RD3(Vwa data, low)
        //ALUIW_WA_IM+7   : mc <= {MCTYPE0, 1'b1, 1'b0, SB_MDH, SA_DST_MDL, 2'b11, RD4};                  //MDL<-MDL(op)MDH, RD4

        /*
            16-bit register-accumulator arithmetic instructions
            (eadd, esub / dadd, dadc, daddnc, dsub, dsbb, dsubnb, dan, dor, dxr, dgt, dlt, dne, deq, don, doff)
        */
        //EALU_EA_R2      : mc <= {MCTYPE3, 1'b0, 1'b1, 5'b10000, 1'b0, 1'b0, 3'b000, 1'b0, 1'b0, IDLE};  //nop, IDLE
        //EALU_EA_R2+1    : mc <= {MCTYPE0, 1'b1, 1'b0, SB_R2, SA_DST_EA, 2'b10, RD4};                    //EA<-EA(op)R2

        //DALU_EA_RP      : mc <= {MCTYPE3, 1'b0, 1'b1, 5'b10000, 1'b0, 1'b0, 3'b000, 1'b0, 1'b0, IDLE};  //nop, IDLE
        //DALU_EA_RP+1    : mc <= {MCTYPE0, 1'b1, 1'b0, SB_RP, SA_DST_EA, 2'b10, RD4};                    //EA<-EA(op)RP

        /*
            Multiplication/division instructions
        */
        MUL             : mc <= {MCTYPE0, 1'b0, 1'b1, T0_SRC_R2, T0_DST_EA, T0_DEU_MUL, IDLE};          //EA<-A*r2, IDLE
        MUL+1           : mc <= {MCTYPE2, 1'b0, 1'b0, T2_NOP, RD4};                                     //nop, RD4

        DIV             : mc <= {MCTYPE0, 1'b0, 1'b1, T0_SRC_R2, T0_DST_EA, T0_DEU_DIV, IDLE};          //EA<-EA/r2, IDLE
        DIV+1           : mc <= {MCTYPE0, 1'b0, 1'b0, T0_SRC_AUX, T0_DST_R2, T0_DEU_MOV, RD4};          //r2<-AUX, RD4

        /*
            Increment/decrement instructions
        */
        INR             : mc <= {MCTYPE0, 1'b1, 1'b1, T0_SRC_R2, T0_DST_R2, T0_DEU_INC, RD4};           //r2<-r2+1, IDLE

        INRW            : mc <= {MCTYPE2, 1'b0, 1'b0, T2_ST_WA, RD3};                                   //STWA, RD3
        INRW+1          : mc <= {MCTYPE1, 1'b0, 1'b1, T1_SRC_A_V_WA, T1_DST_MA, T1_AEU_MOV, RD3};       //MA<-Vwa, RD3
        INRW+2          : mc <= {MCTYPE0, 1'b1, 1'b0, T0_SRC_MD0, T0_DST_MD0, T0_DEU_INC, IDLE};        //MD0<-MD0+1, IDLE
        INRW+3          : mc <= {MCTYPE1, 1'b0, 1'b0, T1_SRC_MA, T1_DST_MA, T1_AEU_DEC, WR3};           //MA<-MA-1, WR3
        INRW+4          : mc <= {MCTYPE2, 1'b0, 1'b0, T2_NOP, RD4};                                     //nop, RD4

        INX_RP2         : mc <= {MCTYPE0, 1'b0, 1'b1, T0_SRC_RP2, T0_DST_RP2, T0_DEU_INC, IDLE};        //rp2<-rp2+1, IDLE
        INX_RP2+1       : mc <= {MCTYPE2, 1'b0, 1'b0, T2_NOP, RD4};                                     //nop, RD4

        INX_EA          : mc <= {MCTYPE0, 1'b0, 1'b1, T0_SRC_EA, T0_DST_EA, T0_DEU_INC, IDLE};          //EA<-EA+1, IDLE
        INX_EA+1        : mc <= {MCTYPE2, 1'b0, 1'b0, T2_NOP, RD4};                                     //nop, RD4

        DCR             : mc <= {MCTYPE0, 1'b1, 1'b1, T0_SRC_R2, T0_DST_R2, T0_DEU_DEC, RD4};           //r2<-r2-1, IDLE

        DCRW            : mc <= {MCTYPE2, 1'b0, 1'b0, T2_ST_WA, RD3};                                   //STWA, RD3
        DCRW+1          : mc <= {MCTYPE1, 1'b0, 1'b1, T1_SRC_A_V_WA, T1_DST_MA, T1_AEU_MOV, RD3};       //MA<-Vwa, RD3
        DCRW+2          : mc <= {MCTYPE0, 1'b1, 1'b0, T0_SRC_MD0, T0_DST_MD0, T0_DEU_DEC, IDLE};        //MD0<-MD0-1, IDLE
        DCRW+3          : mc <= {MCTYPE1, 1'b0, 1'b0, T1_SRC_MA, T1_DST_MA, T1_AEU_DEC, WR3};           //MA<-MA-1, WR3
        DCRW+4          : mc <= {MCTYPE2, 1'b0, 1'b0, T2_NOP, RD4};                                     //nop, RD4

        DCX_RP2         : mc <= {MCTYPE0, 1'b0, 1'b1, T0_SRC_RP2, T0_DST_RP2, T0_DEU_DEC, IDLE};        //rp2<-rp2-1, IDLE
        DCX_RP2+1       : mc <= {MCTYPE2, 1'b0, 1'b0, T2_NOP, RD4};                                     //nop, RD4

        DCX_EA          : mc <= {MCTYPE0, 1'b0, 1'b1, T0_SRC_EA, T0_DST_EA, T0_DEU_DEC, IDLE};          //EA<-EA-1, IDLE
        DCX_EA+1        : mc <= {MCTYPE2, 1'b0, 1'b0, T2_NOP, RD4};                                     //nop, RD4
        

        /*
            Shift/rotation instructions
        */
        //ROTSHFT_R2      : mc <= {MCTYPE1, 1'b1, 1'b1, SD_NOSOURCE, SC_DST_R2, 4'b0111, RD4};            //byte rot/shft, RD4
        //ROTSHFT_EA      : mc <= {MCTYPE1, 1'b1, 1'b1, SD_NOSOURCE, SC_DST_EA, 4'b0111, RD4};            //word rot/shft, RD4

        /*
            Misc arithmetic instructions
        */
        //DAA             : mc <= {MCTYPE1, 1'b1, 1'b1, SD_NOSOURCE, SC_DST_A, 4'b0010, RD4};             //A<-(op)A, IDLE
        //NEGA            : mc <= {MCTYPE1, 1'b0, 1'b1, SD_NOSOURCE, SC_DST_A, 4'b0001, RD4};             //A<-negative A, RD4
        //STC_CLC         : mc <= {MCTYPE2, 1'b1, 1'b1, 4'b0000, 1'b1, 1'b0, 2'b00, 1'b0, 3'b000, RD4};   //carry mod

        /*
            Jump/call/return instructions
        */
        
        /*
            Processor control instructions
        */
        IRD             : mc <= {MCTYPE3, 1'b0, 1'b0, 5'b00000, 1'b0, 1'b0, 3'b000, 1'b0, 1'b1, RD4};   //wait for decoding
        NOP             : mc <= {MCTYPE2, 1'b1, 1'b1, T2_NOP, RD4};                                     //nop, RD4

        default         : mc <= {MCTYPE2, 1'b1, 1'b1, T2_NOP, RD4};                                     //nop, RD4
    endcase
end

endmodule