`include "IKA87AD_mnemonics.sv"

module IKA87AD_microcode (
    input   wire                i_CLK,
    input   wire                i_MCROM_READ_TICK,
    input   wire    [7:0]       i_MCROM_ADDR,
    output  wire    [17:0]      o_MCROM_DATA
);

(* ramstyle = "M10K" *) reg     [17:0]  mc;
assign  o_MCROM_DATA = mc;

always @(posedge i_CLK) if(i_MCROM_READ_TICK) begin
    case(i_MCROM_ADDR)
        //
        //  START ADDRESS 0: 2-CYCLE INSTRUCTION GROUP
        //
        //                       MCTYPE   FLAG  SKIP
        //immediate data load
        MVI_R_IM        : mc <= {MCTYPE3, 1'b0, 1'b0, 5'b10000, 1'b0, 1'b0, 3'b000, 1'b0, 1'b0, RD3};   //nop, RD3
        MVI_R_IM+1      : mc <= {MCTYPE0, 1'b1, 1'b1, SB_MD, SA_DST_R, 2'b00, RD4};                     //r<-MD, RD4
        LXI_RP2_IM      : mc <= {MCTYPE3, 1'b0, 1'b0, 5'b10001, 1'b0, 1'b0, 3'b000, 1'b0, 1'b0, RD3};   //nop*2, RD3
        LXI_RP2_IM+1    : mc <= {MCTYPE0, 1'b1, 1'b1, SB_MD, SA_DST_RP2, 2'b00, RD4};                   //rp2<-MD, RD4

        //rpa version of STAX, LDAX
        STAX_RPA_A      : mc <= {MCTYPE1, 1'b0, 1'b1, SD_RPA, SC_DST_MA, 4'b1010, WR3};                 //MA<-RPA, WR3 //A will be loaded into MD automatically
        STAX_RPA_A+1    : mc <= {MCTYPE3, 1'b0, 1'b0, 5'b10000, 1'b0, 1'b0, 3'b000, 1'b0, 1'b0, RD4};   //nop, RD4
        LDAX_A_RPA      : mc <= {MCTYPE1, 1'b0, 1'b1, SD_RPA, SC_DST_MA, 4'b1010, RD3};                 //MA<-RPA, WR3
        LDAX_A_RPA+1    : mc <= {MCTYPE0, 1'b1, 1'b0, SB_MD, SA_DST_A, 2'b00, RD4};                     //A<-MD, RD4

        //INC or DEC, does not touch any flag
        INX_RP2         : mc <= {MCTYPE0, 1'b0, 1'b1, SB_ADD1, SA_DST_MA, 2'b01, IDLE};                 //MA<-MA+1, IDLE(stop PC increment)
        INX_RP2+1       : mc <= {MCTYPE0, 1'b0, 1'b0, SB_ADD1, SA_DST_RP2, 2'b01, RD4};                 //RP2<-RP2+1
        INX_EA          : mc <= {MCTYPE0, 1'b0, 1'b1, SB_ADD1, SA_DST_MA, 2'b01, IDLE};                 //MA<-MA+1, IDLE(stop PC increment)
        INX_EA+1        : mc <= {MCTYPE0, 1'b0, 1'b0, SB_ADD1, SA_DST_EA, 2'b01, RD4};                  //EA<-EA+1
        DCX_RP2         : mc <= {MCTYPE0, 1'b0, 1'b1, SB_ADD1, SA_DST_MA, 2'b01, IDLE};                 //MA<-MA+1, IDLE(stop PC increment)
        DCX_RP2+1       : mc <= {MCTYPE0, 1'b0, 1'b0, SB_SUB1, SA_DST_RP2, 2'b01, RD4};                 //RP2<-RP2-1
        DCX_EA          : mc <= {MCTYPE0, 1'b0, 1'b1, SB_ADD1, SA_DST_MA, 2'b01, IDLE};                 //MA<-MA+1, IDLE(stop PC increment)
        DCX_EA+1        : mc <= {MCTYPE0, 1'b0, 1'b0, SB_SUB1, SA_DST_EA, 2'b01, RD4};                  //EA<-EA-1

        JMP             : mc <= {MCTYPE3, 1'b0, 1'b0, 5'b10001, 1'b0, 1'b0, 3'b000, 1'b0, 1'b0, RD3};   //nop*2, RD3
        JMP+1           : mc <= {MCTYPE0, 1'b0, 1'b1, SB_MD, SA_DST_PC, 2'b00, RD4};                    //PC<-MD

        JR              : mc <= {MCTYPE3, 1'b0, 1'b1, 5'b10001, 1'b0, 1'b0, 3'b000, 1'b0, 1'b0, IDLE};  //nop*2, IDLE
        JR+1            : mc <= {MCTYPE0, 1'b0, 1'b0, SB_ADDR_REL_S, SA_DST_PC, 2'b01, RD4};            //PC<-PC+jdisp1(short)

        //addx, adcx, addncx, subx, sbbx, subncx, anax, orax, xrax, gtax, ltax, neax, eqax, onax, offax
        ALUX_A_RPA      : mc <= {MCTYPE1, 1'b0, 1'b1, SD_RPA, SC_DST_MA, 4'b1010, RD3};                 //MA<-RPA
        ALUX_A_RPA+1    : mc <= {MCTYPE0, 1'b1, 1'b0, SB_MD, SA_DST_A, 2'b10, RD4};                     //A<-MD

        //adi, aci, adinc, sui, sbi, suinb, ani, ori, xri, gti, lti, nei, eqi, oni, offi
        ALUI_A_IM       : mc <= {MCTYPE3, 1'b0, 1'b0, 5'b10000, 1'b0, 1'b0, 3'b000, 1'b0, 1'b0, RD3};   //nop, RD3
        ALUI_A_IM+1     : mc <= {MCTYPE0, 1'b1, 1'b1, SB_MD, SA_DST_A, 2'b11, RD4};                     //A<-MD

        //adi, aci, adinc, sui, sbi, suinb, ani, ori, xri, gti, lti, nei, eqi, oni, offi
        ALUI_R_IM       : mc <= {MCTYPE3, 1'b0, 1'b0, 5'b10000, 1'b0, 1'b0, 3'b000, 1'b0, 1'b0, RD3};   //nop, RD3
        ALUI_R_IM+1     : mc <= {MCTYPE0, 1'b1, 1'b1, SB_MD, SA_DST_R, 2'b10, RD4};                     //R<-MD

        //eadd, esub
        EALU_EA_R2      : mc <= {MCTYPE3, 1'b0, 1'b1, 5'b10000, 1'b0, 1'b0, 3'b000, 1'b0, 1'b0, IDLE};  //nop, IDLE
        EALU_EA_R2+1    : mc <= {MCTYPE0, 1'b1, 1'b0, SB_R2, SA_DST_EA, 2'b10, RD4};                    //EA<-EA(op)R2

        //dadd, dadc, daddnc, dsub, dsbb, dsubnb, dan, dor, dxr, dgt, dlt, dne, deq, don, doff
        DALU_EA_RP      : mc <= {MCTYPE3, 1'b0, 1'b1, 5'b10000, 1'b0, 1'b0, 3'b000, 1'b0, 1'b0, IDLE};  //nop, IDLE
        DALU_EA_RP+1    : mc <= {MCTYPE0, 1'b1, 1'b0, SB_RP, SA_DST_EA, 2'b10, RD4};                    //EA<-EA(op)RP

        8'd030          : mc <= {MCTYPE3, 1'b1, 1'b1, 5'b10000, 1'b0, 1'b0, 3'b000, 1'b0, 1'b0, RD4};
        8'd031          : mc <= {MCTYPE3, 1'b1, 1'b1, 5'b10000, 1'b0, 1'b0, 3'b000, 1'b0, 1'b0, RD4};


        //
        //  START ADDRESS 32: 4-CYCLE INSTRUCTION GROUP
        //
        //                       MCTYPE   FLAG  SKIP
        MOV_MEM_R       : mc <= {MCTYPE0, 1'b0, 1'b0, SB_R, SA_DST_MDL, 2'b00, RD3};                    //MD<-R
        MOV_MEM_R+1     : mc <= {MCTYPE3, 1'b0, 1'b0, 5'b10000, 1'b0, 1'b0, 3'b000, 1'b0, 1'b0, RD3};   //nop, RD3
        MOV_MEM_R+2     : mc <= {MCTYPE0, 1'b0, 1'b1, SB_MDI, SA_DST_MA, 2'b00, WR3};                   //MA<-MDI, WR3
        MOV_MEM_R+3     : mc <= {MCTYPE3, 1'b0, 1'b0, 5'b10000, 1'b0, 1'b0, 3'b000, 1'b0, 1'b0, RD4};   //nop, RD4

        MVIW_WA_IM      : mc <= {MCTYPE3, 1'b0, 1'b0, 5'b00000, 1'b0, 1'b0, 3'b000, 1'b1, 1'b0, RD3};   //swap MD output order, RD3
        MVIW_WA_IM+1    : mc <= {MCTYPE3, 1'b0, 1'b0, 5'b10000, 1'b0, 1'b0, 3'b000, 1'b0, 1'b0, RD3};   //nop, RD3
        MVIW_WA_IM+2    : mc <= {MCTYPE0, 1'b0, 1'b1, SB_ADDR_V_WA, SA_DST_MA, 2'b00, WR3};             //MA<-Vwa, WR3
        MVIW_WA_IM+3    : mc <= {MCTYPE3, 1'b0, 1'b0, 5'b10000, 1'b0, 1'b0, 3'b000, 1'b0, 1'b0, RD4};   //nop, RD4

        //rpa2(register offset) version of STAX, LDAX
        STAX_RPA2_A     : mc <= {MCTYPE3, 1'b0, 1'b0, 5'b00000, 1'b0, 1'b1, 3'b000, 1'b0, 1'b0, IDLE};  //conditional read, IDLE
        STAX_RPA2_A+1   : mc <= {MCTYPE0, 1'b0, 1'b1, SB_OFFSET, SA_DST_MA, 2'b00, IDLE};               //MA<-RPA_OFFSET, IDLE
        STAX_RPA2_A+2   : mc <= {MCTYPE0, 1'b0, 1'b0, SB_RPA2, SA_DST_MA, 2'b01, WR3};                  //MA<-MA+RPA2, WR3
        STAX_RPA2_A+3   : mc <= {MCTYPE3, 1'b0, 1'b0, 5'b10000, 1'b0, 1'b0, 3'b000, 1'b0, 1'b0, RD4};   //nop, RD4
        LDAX_A_RPA2     : mc <= {MCTYPE3, 1'b0, 1'b0, 5'b00000, 1'b0, 1'b1, 3'b000, 1'b0, 1'b0, IDLE};  //conditional read, IDLE
        LDAX_A_RPA2+1   : mc <= {MCTYPE0, 1'b0, 1'b1, SB_OFFSET, SA_DST_MA, 2'b00, IDLE};               //MA<-RPA_OFFSET, IDLE
        LDAX_A_RPA2+2   : mc <= {MCTYPE0, 1'b0, 1'b0, SB_RPA2, SA_DST_MA, 2'b01, RD3};                  //MA<-MA+RPA2, RD3
        LDAX_A_RPA2+3   : mc <= {MCTYPE0, 1'b0, 1'b0, SB_MD, SA_DST_A, 2'b00, RD4};                     //A<-MD

        //rp<-mem: LBCD, LDED, LHLD, LSPD
        LD_RP2_MEM      : mc <= {MCTYPE3, 1'b0, 1'b0, 5'b10001, 1'b0, 1'b0, 3'b000, 1'b0, 1'b0, RD3};   //nop*2, RD3
        LD_RP2_MEM+1    : mc <= {MCTYPE0, 1'b0, 1'b1, SB_MD, SA_DST_MA, 2'b00, RD3};                    //MA<-MD, RD3
        LD_RP2_MEM+2    : mc <= {MCTYPE3, 1'b0, 1'b0, 5'b10000, 1'b0, 1'b0, 3'b000, 1'b0, 1'b0, RD3};   //nop, RD3
        LD_RP2_MEM+3    : mc <= {MCTYPE0, 1'b0, 1'b0, SB_MD, SA_DST_RP2, 2'b00, RD4};                   //rp2<-MD, RD4

        //block transfer
        BLOCK           : mc <= {MCTYPE1, 1'b0, 1'b1, SD_HL, SC_DST_MA, 4'b1001, RD3};                  //MA<-HL+, RD3
        BLOCK+1         : mc <= {MCTYPE1, 1'b0, 1'b0, SD_DE, SC_DST_MA, 4'b1001, WR3};                  //MA<-DE+, WR3
        BLOCK+2         : mc <= {MCTYPE0, 1'b0, 1'b0, SB_SUB1, SA_DST_C, 2'b01, IDLE};                  //C<-C-1, IDLE
        BLOCK+3         : mc <= {MCTYPE3, 1'b0, 1'b0, 5'b00000, 1'b1, 1'b0, 3'b000, 1'b0, 1'b0, RD4};   //conditional PC decrement, RD4

        //table pick up
        TABLE           : mc <= {MCTYPE0, 1'b0, 1'b1, SB_ADD2, SA_DST_MA, 2'b01, IDLE};                 //MA<-MA+2, IDLE
        TABLE+1         : mc <= {MCTYPE0, 1'b0, 1'b0, SB_A, SA_DST_MA, 2'b01, RD3};                     //MA<-MA+A, IDLE
        TABLE+2         : mc <= {MCTYPE3, 1'b0, 1'b0, 5'b10000, 1'b0, 1'b0, 3'b000, 1'b0, 1'b0, RD3};   //nop, RD3
        TABLE+3         : mc <= {MCTYPE1, 1'b0, 1'b0, SD_MD, SC_DST_BC, 4'b0000, RD4};                  //BC<-MD, RD4

        //BCD support: RLD/RRD
        RLD_RRD         : mc <= {MCTYPE1, 1'b0, 1'b1, SD_HL, SC_DST_MA, 4'b0000, RD3};                  //MA<-HL, RD3
        RLD_RRD+1       : mc <= {MCTYPE1, 1'b0, 1'b0, SD_A, SC_DST_MDL, 4'b0011, IDLE};                 //(TEMP, MDL)<-(A, MDL), IDLE
        RLD_RRD+2       : mc <= {MCTYPE0, 1'b0, 1'b0, SB_SUB1, SA_DST_MA, 2'b01, WR3};                  //MA<-MA-1, WR3
        RLD_RRD+3       : mc <= {MCTYPE0, 1'b0, 1'b0, SB_TEMP, SA_DST_A, 2'b00, RD4};                   //A<-ALU temp, RD4

        //PUSH
        PUSH            : mc <= {MCTYPE1, 1'b0, 1'b1, SD_SP, SC_DST_MA, 4'b1000, IDLE};                 //MA<--SP, IDLE
        PUSH+1          : mc <= {MCTYPE0, 1'b0, 1'b0, SB_RP1, SA_DST_MD, 2'b00, WR3};                   //MD<-rp1, WR3
        PUSH+2          : mc <= {MCTYPE0, 1'b0, 1'b0, SB_SUB1, SA_DST_SP, 2'b01, WR3};                  //SP<-SP-1, WR3
        PUSH+3          : mc <= {MCTYPE3, 1'b0, 1'b0, 5'b10000, 1'b0, 1'b0, 3'b000, 1'b0, 1'b0, RD4};   //nop, RD4

        //calls
        CALB            : mc <= {MCTYPE1, 1'b0, 1'b1, SD_SP, SC_DST_MA, 4'b1000, IDLE};                 //MA<--SP, IDLE
        CALB+1          : mc <= {MCTYPE3, 1'b0, 1'b0, 5'b10000, 1'b0, 1'b0, 3'b000, 1'b0, 1'b0, WR3};   //nop, WR3
        CALB+2          : mc <= {MCTYPE0, 1'b0, 1'b0, SB_SUB1, SA_DST_SP, 2'b01, WR3};                  //SP<-SP-1, WR3
        CALB+3          : mc <= {MCTYPE1, 1'b0, 1'b0, SD_BC, SC_DST_PC, 4'b0000, RD4};                  //PC<-BC, RD4

        CALF            : mc <= {MCTYPE3, 1'b0, 1'b0, 5'b10000, 1'b0, 1'b0, 3'b000, 1'b0, 1'b0, RD3};   //nop, RD3
        CALF+1          : mc <= {MCTYPE1, 1'b0, 1'b1, SD_SP, SC_DST_MA, 4'b1000, WR3};                  //MA<--SP, WR3
        CALF+2          : mc <= {MCTYPE0, 1'b0, 1'b0, SB_SUB1, SA_DST_SP, 2'b01, WR3};                  //SP<-SP-1, WR3
        CALF+3          : mc <= {MCTYPE0, 1'b0, 1'b0, SB_ADDR_FA, SA_DST_PC, 2'b00, RD4};               //PC<-fa, RD4

        CALL            : mc <= {MCTYPE3, 1'b0, 1'b0, 5'b10001, 1'b0, 1'b0, 3'b000, 1'b0, 1'b0, RD3};   //nop*2, RD3
        CALL+1          : mc <= {MCTYPE1, 1'b0, 1'b1, SD_SP, SC_DST_MA, 4'b1000, WR3};                  //MA<--SP, WR3
        CALL+2          : mc <= {MCTYPE0, 1'b0, 1'b0, SB_SUB1, SA_DST_SP, 2'b01, WR3};                  //SP<-SP-1, WR3
        CALL+3          : mc <= {MCTYPE0, 1'b0, 1'b0, SB_MDI, SA_DST_PC, 2'b00, RD4};                   //PC<-MDI(im), RD4

        CALT            : mc <= {MCTYPE3, 1'b0, 1'b1, 5'b10001, 1'b0, 1'b0, 3'b000, 1'b0, 1'b0, IDLE};  //nop*2, IDLE
        CALT+1          : mc <= {MCTYPE1, 1'b0, 1'b0, SD_SP, SC_DST_MA, 4'b1000, WR3};                  //MA<--SP, WR3
        CALT+2          : mc <= {MCTYPE0, 1'b0, 1'b0, SB_SUB1, SA_DST_SP, 2'b01, WR3};                  //SP<-SP-1, WR3
        CALT+3          : mc <= {MCTYPE0, 1'b0, 1'b0, SB_ADDR_TA, SA_DST_PC, 2'b00, RD4};               //PC<-ta, RD4

        //return from interrupt
        RETI            : mc <= {MCTYPE1, 1'b0, 1'b1, SD_SP, SC_DST_MA, 4'b1001, RD3};                  //MA<-SP+, RD3
        RETI+1          : mc <= {MCTYPE0, 1'b0, 1'b0, SB_ADD2, SA_DST_SP, 2'b01, RD3};                  //SP<-SP+2, RD3
        RETI+2          : mc <= {MCTYPE0, 1'b0, 1'b0, SB_MD, SA_DST_PC, 2'b00, RD3};                    //PC<-MD, RD3
        RETI+3          : mc <= {MCTYPE1, 1'b0, 1'b0, SD_MD, SC_DST_PSW, 4'b0000, RD4};                 //PSW<-MD, RD4

        //
        //  START ADDRESS 80: 3-CYCLE AND 5-CYCLE INSTRUCTION GROUP
        //
        //                       MCTYPE   FLAG  SKIP
        MOV_SR_A        : mc <= {MCTYPE3, 1'b0, 1'b0, 5'b10000, 1'b0, 1'b0, 3'b000, 1'b0, 1'b0, RD3};   //nop, RD3
        MOV_SR_A+1      : mc <= {MCTYPE3, 1'b0, 1'b1, 5'b10000, 1'b0, 1'b0, 3'b000, 1'b0, 1'b0, IDLE};  //nop, IDLE
        MOV_SR_A+2      : mc <= {MCTYPE0, 1'b0, 1'b0, SB_A, SA_DST_SR_SR1, 2'b00, RD4};                 //sr<-A, RD4

        ST_MEM_RP2      : mc <= {MCTYPE0, 1'b0, 1'b0, SB_RP2, SA_DST_MD, 2'b00, RD3};                   //MD<-RP2, RD3
        ST_MEM_RP2+1    : mc <= {MCTYPE3, 1'b0, 1'b0, 5'b10000, 1'b0, 1'b0, 3'b000, 1'b0, 1'b0, RD3};   //nop, RD3
        ST_MEM_RP2+2    : mc <= {MCTYPE0, 1'b0, 1'b1, SB_MDI, SA_DST_MA, 2'b00, WR3};                   //MA<-MDI, WR3
        ST_MEM_RP2+3    : mc <= {MCTYPE3, 1'b0, 1'b0, 5'b10000, 1'b0, 1'b0, 3'b000, 1'b0, 1'b0, WR3};   //nop, WR3
        ST_MEM_RP2+4    : mc <= {MCTYPE3, 1'b0, 1'b0, 5'b10000, 1'b0, 1'b0, 3'b000, 1'b0, 1'b0, RD4};   //nop, RD4

        MOV_A_SR1       : mc <= {MCTYPE3, 1'b0, 1'b0, 5'b10000, 1'b0, 1'b0, 3'b000, 1'b0, 1'b0, RD3};   //nop, RD3
        MOV_A_SR1+1     : mc <= {MCTYPE3, 1'b0, 1'b1, 5'b10000, 1'b0, 1'b0, 3'b000, 1'b0, 1'b0, IDLE};  //nop, IDLE
        MOV_A_SR1+2     : mc <= {MCTYPE0, 1'b0, 1'b0, SB_SR_SR1, SA_DST_A, 2'b00, RD4};                 //sr<-A, RD4

        INRW            : mc <= {MCTYPE3, 1'b0, 1'b0, 5'b00000, 1'b0, 1'b0, 3'b000, 1'b1, 1'b0, RD3};   //swap MD output order, RD3
        INRW+1          : mc <= {MCTYPE0, 1'b0, 1'b1, SB_ADDR_V_WA, SA_DST_MA, 2'b00, RD3};             //MA<-Vwa, RD3
        INRW+2          : mc <= {MCTYPE1, 1'b1, 1'b0, SD_NOSOURCE, SC_DST_MDH, 4'b1011, IDLE};          //MDH<-MDH+1, IDLE
        INRW+3          : mc <= {MCTYPE0, 1'b0, 1'b0, SB_SUB1, SA_DST_MA, 2'b01, WR3};                  //MA<-MA-1, WR3
        INRW+4          : mc <= {MCTYPE3, 1'b0, 1'b0, 5'b10000, 1'b0, 1'b0, 3'b000, 1'b0, 1'b0, RD4};   //nop, RD4

        MOV_R_MEM       : mc <= {MCTYPE3, 1'b0, 1'b0, 5'b10001, 1'b0, 1'b0, 3'b000, 1'b0, 1'b0, RD3};   //nop*2, RD3
        MOV_R_MEM+1     : mc <= {MCTYPE0, 1'b0, 1'b1, SB_MD, SA_DST_MA, 2'b00, RD3};                    //MA<-MD, RD3
        MOV_R_MEM+2     : mc <= {MCTYPE0, 1'b0, 1'b0, SB_MD, SA_DST_R, 2'b00, RD4};                     //r<-MD, RD4

        DCRW            : mc <= {MCTYPE3, 1'b0, 1'b0, 5'b00000, 1'b0, 1'b0, 3'b000, 1'b1, 1'b0, RD3};   //swap MD output order, RD3
        DCRW+1          : mc <= {MCTYPE0, 1'b0, 1'b1, SB_ADDR_V_WA, SA_DST_MA, 2'b00, RD3};             //MA<-Vwa, RD3
        DCRW+2          : mc <= {MCTYPE1, 1'b1, 1'b0, SD_NOSOURCE, SC_DST_MDH, 4'b1100, IDLE};          //MDH<-MDH-1, IDLE
        DCRW+3          : mc <= {MCTYPE0, 1'b0, 1'b0, SB_SUB1, SA_DST_MA, 2'b01, WR3};                  //MA<-MA-1, WR3
        DCRW+4          : mc <= {MCTYPE3, 1'b0, 1'b0, 5'b10000, 1'b0, 1'b0, 3'b000, 1'b0, 1'b0, RD4};   //nop, RD4

        //STEAX, LDEAX
        STEAX_RPA_EA    : mc <= {MCTYPE1, 1'b0, 1'b1, SD_RPA, SC_DST_MA, 4'b1010, WR3};                 //MA<-RPA, WR3 :EA will be loaded into MD automatically
        STEAX_RPA_EA+1  : mc <= {MCTYPE1, 1'b0, 1'b0, SD_RPA, SC_DST_NOWHERE, 4'b1010, WR3};            //NOWHERE<-RPA, WR3 :poke RPA once more(double increment)
        STEAX_RPA_EA+2  : mc <= {MCTYPE3, 1'b0, 1'b0, 5'b10000, 1'b0, 1'b0, 3'b000, 1'b0, 1'b0, RD4};   //nop, RD4

        STEAX_RPA2_EA   : mc <= {MCTYPE3, 1'b0, 1'b0, 5'b00000, 1'b0, 1'b1, 3'b000, 1'b0, 1'b0, IDLE};  //conditional read, IDLE
        STEAX_RPA2_EA+1 : mc <= {MCTYPE0, 1'b0, 1'b1, SB_OFFSET, SA_DST_MA, 2'b00, IDLE};               //MA<-RPA_OFFSET, IDLE
        STEAX_RPA2_EA+2 : mc <= {MCTYPE0, 1'b0, 1'b0, SB_RPA2, SA_DST_MA, 2'b01, WR3};                  //MA<-MA+RPA2, WR3
        STEAX_RPA2_EA+3 : mc <= {MCTYPE3, 1'b0, 1'b0, 5'b10000, 1'b0, 1'b0, 3'b000, 1'b0, 1'b0, WR3};   //nop, WR3
        STEAX_RPA2_EA+4 : mc <= {MCTYPE3, 1'b0, 1'b0, 5'b10000, 1'b0, 1'b0, 3'b000, 1'b0, 1'b0, RD4};   //nop, RD4

        LDEAX_EA_RPA    : mc <= {MCTYPE1, 1'b0, 1'b1, SD_RPA, SC_DST_MA, 4'b1010, RD3};                 //MA<-RPA, RD3 :EA will be loaded into MD automatically
        LDEAX_EA_RPA+1  : mc <= {MCTYPE1, 1'b0, 1'b0, SD_RPA, SC_DST_NOWHERE, 4'b1010, RD3};            //NOWHERE<-RPA, RD3 :poke RPA once more(double increment)
        LDEAX_EA_RPA+2  : mc <= {MCTYPE0, 1'b0, 1'b0, SB_MD, SA_DST_EA, 2'b00, RD4};                    //EA<-MD, RD4

        LDEAX_EA_RPA2   : mc <= {MCTYPE3, 1'b0, 1'b0, 5'b00000, 1'b0, 1'b1, 3'b000, 1'b0, 1'b0, IDLE};  //conditional read, IDLE
        LDEAX_EA_RPA2+1 : mc <= {MCTYPE0, 1'b0, 1'b1, SB_OFFSET, SA_DST_MA, 2'b00, IDLE};               //MA<-RPA_OFFSET, IDLE
        LDEAX_EA_RPA2+2 : mc <= {MCTYPE0, 1'b0, 1'b0, SB_RPA2, SA_DST_MA, 2'b01, RD3};                  //MA<-MA+RPA2, RD3
        LDEAX_EA_RPA2+3 : mc <= {MCTYPE3, 1'b0, 1'b0, 5'b10000, 1'b0, 1'b0, 3'b000, 1'b0, 1'b0, RD3};   //nop, RD3
        LDEAX_EA_RPA2+4 : mc <= {MCTYPE0, 1'b0, 1'b0, SB_MD, SA_DST_EA, 2'b00, RD4};                    //EA<-MD, RD4

        MVIX_RPA_IM     : mc <= {MCTYPE3, 1'b0, 1'b0, 5'b10000, 1'b0, 1'b0, 3'b000, 1'b0, 1'b0, RD3};   //nop, RD3
        MVIX_RPA_IM+1   : mc <= {MCTYPE0, 1'b0, 1'b1, SB_RPA1, SA_DST_MA, 2'b00, WR3};                  //MA<-RPA, WR3
        MVIX_RPA_IM+2   : mc <= {MCTYPE3, 1'b0, 1'b0, 5'b10000, 1'b0, 1'b0, 3'b000, 1'b0, 1'b0, RD4};   //nop, RD4

        SOFTI           : mc <= {MCTYPE1, 1'b0, 1'b0, SD_SP, SC_DST_MA, 4'b1000, IDLE};                 //MA<--SP, IDLE
        SOFTI+1         : mc <= {MCTYPE1, 1'b0, 1'b0, SD_PSW, SC_DST_MD, 4'b0000, WR3};                 //MD<-PSW, WR3
        SOFTI+2         : mc <= {MCTYPE1, 1'b0, 1'b0, SD_PC, SC_DST_MD, 4'b0000, WR3};                  //MD<-PC, WR3
        SOFTI+3         : mc <= {MCTYPE0, 1'b0, 1'b0, SB_SUB2, SA_DST_SP, 2'b01, WR3};                  //SP<-SP-2, WR3
        SOFTI+4         : mc <= {MCTYPE0, 1'b1, 1'b0, SB_ADDR_INT, SA_DST_PC, 2'b00, RD4};              //PC<-0060, RD4

        POP             : mc <= {MCTYPE1, 1'b0, 1'b1, SD_SP, SC_DST_MA, 4'b1001, RD3};                  //MA<-SP+, RD3
        POP+1           : mc <= {MCTYPE0, 1'b0, 1'b0, SB_ADD1, SA_DST_SP, 2'b01, RD3};                  //SP<-SP+1, RD3
        POP+2           : mc <= {MCTYPE0, 1'b0, 1'b0, SB_MD, SA_DST_RP1, 2'b00, RD4};                   //MD<-rp1, WR3

        HARDI           : mc <= {MCTYPE1, 1'b0, 1'b0, SD_SP, SC_DST_MA, 4'b1000, IDLE};                 //MA<--SP, IDLE
        HARDI+1         : mc <= {MCTYPE1, 1'b0, 1'b0, SD_PSW, SC_DST_MD, 4'b0000, WR3};                 //MD<-PSW, WR3
        HARDI+2         : mc <= {MCTYPE1, 1'b0, 1'b0, SD_PC, SC_DST_MD, 4'b0000, WR3};                  //MD<-PC, WR3
        HARDI+3         : mc <= {MCTYPE0, 1'b0, 1'b0, SB_SUB2, SA_DST_SP, 2'b01, WR3};                  //SP<-SP-2, WR3
        HARDI+4         : mc <= {MCTYPE0, 1'b1, 1'b0, SB_ADDR_INT, SA_DST_PC, 2'b00, RD4};              //PC<-int addr, RD4

        //
        //  START ADDRESS 144: 3-CYCLE AND 1-CYCLE INSTRUCTION GROUP
        //
        //                       MCTYPE   FLAG  SKIP
        STAW            : mc <= {MCTYPE0, 1'b0, 1'b0, SB_A, SA_DST_MDL, 2'b00, RD3};                    //MD<-A
        STAW+1          : mc <= {MCTYPE0, 1'b0, 1'b1, SB_ADDR_V_WA, SA_DST_MA, 2'b00, WR3};             //MA<-Vwa, WR3
        STAW+2          : mc <= {MCTYPE3, 1'b0, 1'b0, 5'b10000, 1'b0, 1'b0, 3'b000, 1'b0, 1'b0, RD4};   //nop, RD4

        MOV_R1_A        : mc <= {MCTYPE0, 1'b0, 1'b1, SB_A, SA_DST_R1, 2'b00, RD4};                     //r1<-A, RD4

        LDAW            : mc <= {MCTYPE3, 1'b0, 1'b0, 5'b10000, 1'b0, 1'b0, 3'b000, 1'b0, 1'b0, RD3};   //nop, RD3
        LDAW+1          : mc <= {MCTYPE0, 1'b0, 1'b1, SB_ADDR_V_WA, SA_DST_MA, 2'b00, RD3};             //MA<-Vwa, RD3
        LDAW+2          : mc <= {MCTYPE0, 1'b0, 1'b0, SB_MDH, SA_DST_A, 2'b00, RD4};                    //A<-MDH, RD4

        MOV_A_R1        : mc <= {MCTYPE0, 1'b0, 1'b1, SB_R1, SA_DST_A, 2'b00, RD4};                     //A<-r1, RD4

        MUL             : mc <= {MCTYPE1, 1'b0, 1'b1, SD_NOSOURCE, SC_DST_EA, 4'b0100, IDLE};           //EA<-A*r2, IDLE
        MUL+1           : mc <= {MCTYPE3, 1'b0, 1'b0, 5'b10110, 1'b0, 1'b0, 3'b000, 1'b0, 1'b0, IDLE};  //nop*7, IDLE
        MUL+2           : mc <= {MCTYPE3, 1'b0, 1'b0, 5'b10000, 1'b0, 1'b0, 3'b000, 1'b0, 1'b0, RD4};   //nop, RD4

        DMOV_RP_EA      : mc <= {MCTYPE0, 1'b0, 1'b1, SB_RP, SA_DST_EA, 2'b00, RD4};                    //rp<-EA, RD4

        DIV             : mc <= {MCTYPE1, 1'b0, 1'b1, SD_NOSOURCE, SC_DST_EA, 4'b0101, IDLE};           //EA<-EA/r2, IDLE
        DIV+1           : mc <= {MCTYPE3, 1'b0, 1'b0, 5'b11111, 1'b0, 1'b0, 3'b000, 1'b0, 1'b0, IDLE};  //nop*16, IDLE
        DIV+2           : mc <= {MCTYPE0, 1'b0, 1'b0, SB_TEMP, SA_DST_R2, 2'b00, RD4};                  //r2<-ALU temp, RD4

        DMOV_EA_RP      : mc <= {MCTYPE0, 1'b0, 1'b1, SB_EA, SA_DST_RP, 2'b00, RD4};                    //EA<-rp, RD4

        DMOV_SR3_EA     : mc <= {MCTYPE3, 1'b0, 1'b0, 5'b10000, 1'b0, 1'b0, 3'b000, 1'b0, 1'b0, RD3};   //nop, RD3
        DMOV_SR3_EA+1   : mc <= {MCTYPE3, 1'b0, 1'b1, 5'b10000, 1'b0, 1'b0, 3'b000, 1'b0, 1'b0, IDLE};  //nop, IDLE
        DMOV_SR3_EA+2   : mc <= {MCTYPE0, 1'b0, 1'b0, SB_EA, SA_DST_SR3, 2'b00, RD4};                   //sr3<-EA, RD4

        INR             : mc <= {MCTYPE1, 1'b1, 1'b1, SD_NOSOURCE, SC_DST_R2, 4'b1011, RD4};            //r2<-r2+1, IDLE

        DMOV_EA_SR4     : mc <= {MCTYPE3, 1'b0, 1'b0, 5'b10000, 1'b0, 1'b0, 3'b000, 1'b0, 1'b0, RD3};   //nop, RD3
        DMOV_EA_SR4+1   : mc <= {MCTYPE3, 1'b0, 1'b1, 5'b10000, 1'b0, 1'b0, 3'b000, 1'b0, 1'b0, IDLE};  //nop, IDLE
        DMOV_EA_SR4+2   : mc <= {MCTYPE0, 1'b0, 1'b0, SB_SR4, SA_DST_EA, 2'b00, RD4};                   //EA<-sr4, RD4

        DCR             : mc <= {MCTYPE1, 1'b1, 1'b1, SD_NOSOURCE, SC_DST_R2, 4'b1100, RD4};            //r2<-r2-1, IDLE

        ALUW_A_WA       : mc <= {MCTYPE3, 1'b0, 1'b0, 5'b10000, 1'b0, 1'b0, 3'b000, 1'b0, 1'b0, RD3};   //nop, RD3
        ALUW_A_WA+1     : mc <= {MCTYPE0, 1'b0, 1'b1, SB_ADDR_V_WA, SA_DST_MA, 2'b00, RD3};             //MA<-Vwa, RD3
        ALUW_A_WA+2     : mc <= {MCTYPE0, 1'b1, 1'b0, SB_MDH, SA_DST_A, 2'b10, RD4};                    //A<-A(op)MDH, RD4

        DAA             : mc <= {MCTYPE1, 1'b1, 1'b1, SD_NOSOURCE, SC_DST_A, 4'b0010, RD4};             //A<-(op)A, IDLE

        JRE             : mc <= {MCTYPE3, 1'b0, 1'b0, 5'b10000, 1'b0, 1'b0, 3'b000, 1'b0, 1'b0, RD3};   //nop, RD3
        JRE+1           : mc <= {MCTYPE3, 1'b0, 1'b1, 5'b10000, 1'b0, 1'b0, 3'b000, 1'b0, 1'b0, IDLE};  //nop, IDLE
        JRE+2           : mc <= {MCTYPE0, 1'b0, 1'b0, SB_ADDR_REL_L, SA_DST_PC, 2'b01, RD4};            //PC<-PC+jdisp(long)

        NEGA            : mc <= {MCTYPE1, 1'b0, 1'b1, SD_NOSOURCE, SC_DST_A, 4'b0001, RD4};             //A<-negative A, RD4

        RET_RETS        : mc <= {MCTYPE1, 1'b0, 1'b1, SD_SP, SC_DST_MA, 4'b1001, RD3};                  //MA<-SP+, RD3
        RET_RETS+1      : mc <= {MCTYPE0, 1'b0, 1'b0, SB_ADD1, SA_DST_SP, 2'b01, RD3};                  //SP<-SP+1, RD3
        RET_RETS+2      : mc <= {MCTYPE0, 1'b1, 1'b0, SB_MD, SA_DST_PC, 2'b00, RD4};                    //PC<-MD, RD4

        STC_CLC         : mc <= {MCTYPE2, 1'b1, 1'b1, 4'b0000, 1'b1, 1'b0, 2'b00, 1'b0, 3'b000, RD4};   //carry mod

        BIT             : mc <= {MCTYPE3, 1'b0, 1'b0, 5'b10000, 1'b0, 1'b0, 3'b000, 1'b0, 1'b0, RD3};   //nop, RD3
        BIT+1           : mc <= {MCTYPE0, 1'b0, 1'b1, SB_ADDR_V_WA, SA_DST_MA, 2'b00, RD3};             //MA<-Vwa, RD3
        BIT+2           : mc <= {MCTYPE2, 1'b1, 1'b0, 4'b0000, 1'b0, 1'b0, 2'b00, 1'b0, 3'b011, RD4};   //skip if a bit is high

        EDI             : mc <= {MCTYPE2, 1'b0, 1'b1, 4'b0000, 1'b0, 1'b1, 2'b00, 1'b0, 3'b000, RD4};   //int flag mod, RD3

        //
        //  START ADDRESS 184: INSTRUCTIONS INCLUDING CONDITIONAL BRANCH MICROCODE
        //
        //                       MCTYPE   FLAG  SKIP
        //aniw, oriw(19 cyc) / gti, lti, oni, offi, eqi, nei(13 cyc)
        ALUIW_WA_IM     : mc <= {MCTYPE3, 1'b0, 1'b0, 5'b10000, 1'b0, 1'b0, 3'b000, 1'b0, 1'b0, RD3};   //nop, RD3(Vwa addr, low)
        ALUIW_WA_IM+1   : mc <= {MCTYPE3, 1'b0, 1'b0, 5'b00000, 1'b0, 1'b0, 3'b100, 1'b0, 1'b0, RD3};   //branch+5, RD3(im, high)
        ALUIW_WA_IM+2   : mc <= {MCTYPE0, 1'b0, 1'b1, SB_ADDR_V_WA, SA_DST_MA, 2'b00, RD3};             //MA<-Vwa, RD3(Vwa data, low)
        ALUIW_WA_IM+3   : mc <= {MCTYPE0, 1'b1, 1'b0, SB_MDH, SA_DST_MDL, 2'b11, IDLE};                 //MDL<-MDL(op)MDH, IDLE
        ALUIW_WA_IM+4   : mc <= {MCTYPE0, 1'b0, 1'b0, SB_SUB1, SA_DST_MA, 2'b01, WR3};                  //MA<-MA-1, WR3
        ALUIW_WA_IM+5   : mc <= {MCTYPE3, 1'b0, 1'b0, 5'b10000, 1'b0, 1'b0, 3'b000, 1'b0, 1'b0, RD4};   //nop, RD4
        ALUIW_WA_IM+6   : mc <= {MCTYPE0, 1'b0, 1'b1, SB_ADDR_V_WA, SA_DST_MA, 2'b00, RD3};             //MA<-Vwa, RD3(Vwa data, low)
        ALUIW_WA_IM+7   : mc <= {MCTYPE0, 1'b1, 1'b0, SB_MDH, SA_DST_MDL, 2'b11, RD4};                  //MDL<-MDL(op)MDH, RD4

        ALUI_SR2_IM     : mc <= {MCTYPE3, 1'b0, 1'b0, 5'b10000, 1'b0, 1'b0, 3'b000, 1'b0, 1'b0, RD3};   //nop, RD3(data, low)
        ALUI_SR2_IM+1   : mc <= {MCTYPE3, 1'b0, 1'b1, 5'b00000, 1'b0, 1'b0, 3'b001, 1'b0, 1'b0, IDLE};  //branch+2, IDLE
        ALUI_SR2_IM+2   : mc <= {MCTYPE3, 1'b0, 1'b0, 5'b10001, 1'b0, 1'b0, 3'b000, 1'b0, 1'b0, IDLE};  //nop*2, IDLE
        ALUI_SR2_IM+3   : mc <= {MCTYPE0, 1'b1, 1'b0, SB_MD, SA_DST_SR2, 2'b10, RD4};                   //SR2<-SR2(op)MD, RD4
        
        8'd196          : mc <= {MCTYPE3, 1'b1, 1'b1, 5'b10000, 1'b0, 1'b0, 3'b000, 1'b0, 1'b0, RD4};
        8'd197          : mc <= {MCTYPE3, 1'b1, 1'b1, 5'b10000, 1'b0, 1'b0, 3'b000, 1'b0, 1'b0, RD4};
        8'd198          : mc <= {MCTYPE3, 1'b1, 1'b1, 5'b10000, 1'b0, 1'b0, 3'b000, 1'b0, 1'b0, RD4};
        8'd199          : mc <= {MCTYPE3, 1'b1, 1'b1, 5'b10000, 1'b0, 1'b0, 3'b000, 1'b0, 1'b0, RD4};

        //
        //  START ADDRESS 200: 1-CYCLE INSTRUCTION GROUP
        //
        //                       MCTYPE   FLAG  SKIP
        ALU_A_R         : mc <= {MCTYPE0, 1'b1, 1'b1, SB_R, SA_DST_A, 2'b10, RD4};                      //A<-A(op)R, RD4
        ALU_R_A         : mc <= {MCTYPE0, 1'b1, 1'b1, SB_A, SA_DST_R, 2'b10, RD4};                      //R<-R(op)A, RD4

        BYTE_RS_R2      : mc <= {MCTYPE1, 1'b1, 1'b1, SD_NOSOURCE, SC_DST_R2, 4'b0111, RD4};            //byte rot/shft, RD4
        WORD_RS_EA      : mc <= {MCTYPE1, 1'b1, 1'b1, SD_NOSOURCE, SC_DST_EA, 4'b0111, RD4};            //word rot/shft, RD4

        JB              : mc <= {MCTYPE1, 1'b0, 1'b1, SD_BC, SC_DST_PC, 4'b0000, RD4};                  //PC<-BC, RD4
        JEA             : mc <= {MCTYPE1, 1'b0, 1'b1, SD_EA, SC_DST_PC, 4'b0000, RD4};                  //PC<-EA, RD4

        EXX             : mc <= {MCTYPE2, 1'b0, 1'b1, 4'b0000, 1'b0, 1'b0, 2'b01, 1'b0, 3'b000, RD4};   //exx mod
        EXA             : mc <= {MCTYPE2, 1'b0, 1'b1, 4'b0000, 1'b0, 1'b0, 2'b10, 1'b0, 3'b000, RD4};   //exa mod
        EXH             : mc <= {MCTYPE2, 1'b0, 1'b1, 4'b0000, 1'b0, 1'b0, 2'b11, 1'b0, 3'b000, RD4};   //exh mod

        SK              : mc <= {MCTYPE2, 1'b1, 1'b1, 4'b0000, 1'b0, 1'b0, 2'b00, 1'b0, 3'b100, RD4};   //skip if flag
        SKN             : mc <= {MCTYPE2, 1'b1, 1'b1, 4'b0000, 1'b0, 1'b0, 2'b00, 1'b0, 3'b101, RD4};   //skip if no flag
        SKIT            : mc <= {MCTYPE2, 1'b1, 1'b1, 4'b0000, 1'b0, 1'b0, 2'b00, 1'b0, 3'b110, RD4};   //skip if interrupt
        SKNIT           : mc <= {MCTYPE2, 1'b1, 1'b1, 4'b0000, 1'b0, 1'b0, 2'b00, 1'b0, 3'b111, RD4};   //skip if no interrupt

        NOP             : mc <= {MCTYPE3, 1'b1, 1'b1, 5'b10000, 1'b0, 1'b0, 3'b000, 1'b0, 1'b0, RD4};   //nop, RD4

        SUSP            : mc <= {MCTYPE2, 1'b0, 1'b1, 4'b0000, 1'b0, 1'b0, 2'b00, 1'b1, 3'b000, RD4};   //set suspension flag, IDLE

        8'd215          : mc <= {MCTYPE3, 1'b1, 1'b1, 5'b10000, 1'b0, 1'b0, 3'b000, 1'b0, 1'b0, RD4};
        8'd216          : mc <= {MCTYPE3, 1'b1, 1'b1, 5'b10000, 1'b0, 1'b0, 3'b000, 1'b0, 1'b0, RD4};
        8'd217          : mc <= {MCTYPE3, 1'b1, 1'b1, 5'b10000, 1'b0, 1'b0, 3'b000, 1'b0, 1'b0, RD4};
        8'd218          : mc <= {MCTYPE3, 1'b1, 1'b1, 5'b10000, 1'b0, 1'b0, 3'b000, 1'b0, 1'b0, RD4};
        8'd219          : mc <= {MCTYPE3, 1'b1, 1'b1, 5'b10000, 1'b0, 1'b0, 3'b000, 1'b0, 1'b0, RD4};
        8'd220          : mc <= {MCTYPE3, 1'b1, 1'b1, 5'b10000, 1'b0, 1'b0, 3'b000, 1'b0, 1'b0, RD4};
        8'd221          : mc <= {MCTYPE3, 1'b1, 1'b1, 5'b10000, 1'b0, 1'b0, 3'b000, 1'b0, 1'b0, RD4};
        8'd222          : mc <= {MCTYPE3, 1'b1, 1'b1, 5'b10000, 1'b0, 1'b0, 3'b000, 1'b0, 1'b0, RD4};
        8'd223          : mc <= {MCTYPE3, 1'b1, 1'b1, 5'b10000, 1'b0, 1'b0, 3'b000, 1'b0, 1'b0, RD4};
        8'd224          : mc <= {MCTYPE3, 1'b1, 1'b1, 5'b10000, 1'b0, 1'b0, 3'b000, 1'b0, 1'b0, RD4};
        8'd225          : mc <= {MCTYPE3, 1'b1, 1'b1, 5'b10000, 1'b0, 1'b0, 3'b000, 1'b0, 1'b0, RD4};
        8'd226          : mc <= {MCTYPE3, 1'b1, 1'b1, 5'b10000, 1'b0, 1'b0, 3'b000, 1'b0, 1'b0, RD4};
        8'd227          : mc <= {MCTYPE3, 1'b1, 1'b1, 5'b10000, 1'b0, 1'b0, 3'b000, 1'b0, 1'b0, RD4};
        8'd228          : mc <= {MCTYPE3, 1'b1, 1'b1, 5'b10000, 1'b0, 1'b0, 3'b000, 1'b0, 1'b0, RD4};
        8'd229          : mc <= {MCTYPE3, 1'b1, 1'b1, 5'b10000, 1'b0, 1'b0, 3'b000, 1'b0, 1'b0, RD4};
        8'd230          : mc <= {MCTYPE3, 1'b1, 1'b1, 5'b10000, 1'b0, 1'b0, 3'b000, 1'b0, 1'b0, RD4};
        8'd231          : mc <= {MCTYPE3, 1'b1, 1'b1, 5'b10000, 1'b0, 1'b0, 3'b000, 1'b0, 1'b0, RD4};
        8'd232          : mc <= {MCTYPE3, 1'b1, 1'b1, 5'b10000, 1'b0, 1'b0, 3'b000, 1'b0, 1'b0, RD4};
        8'd233          : mc <= {MCTYPE3, 1'b1, 1'b1, 5'b10000, 1'b0, 1'b0, 3'b000, 1'b0, 1'b0, RD4};
        8'd234          : mc <= {MCTYPE3, 1'b1, 1'b1, 5'b10000, 1'b0, 1'b0, 3'b000, 1'b0, 1'b0, RD4};
        8'd235          : mc <= {MCTYPE3, 1'b1, 1'b1, 5'b10000, 1'b0, 1'b0, 3'b000, 1'b0, 1'b0, RD4};
        8'd236          : mc <= {MCTYPE3, 1'b1, 1'b1, 5'b10000, 1'b0, 1'b0, 3'b000, 1'b0, 1'b0, RD4};
        8'd237          : mc <= {MCTYPE3, 1'b1, 1'b1, 5'b10000, 1'b0, 1'b0, 3'b000, 1'b0, 1'b0, RD4};
        8'd238          : mc <= {MCTYPE3, 1'b1, 1'b1, 5'b10000, 1'b0, 1'b0, 3'b000, 1'b0, 1'b0, RD4};
        8'd239          : mc <= {MCTYPE3, 1'b1, 1'b1, 5'b10000, 1'b0, 1'b0, 3'b000, 1'b0, 1'b0, RD4};
        8'd240          : mc <= {MCTYPE3, 1'b1, 1'b1, 5'b10000, 1'b0, 1'b0, 3'b000, 1'b0, 1'b0, RD4};
        8'd241          : mc <= {MCTYPE3, 1'b1, 1'b1, 5'b10000, 1'b0, 1'b0, 3'b000, 1'b0, 1'b0, RD4};
        8'd242          : mc <= {MCTYPE3, 1'b1, 1'b1, 5'b10000, 1'b0, 1'b0, 3'b000, 1'b0, 1'b0, RD4};
        8'd243          : mc <= {MCTYPE3, 1'b1, 1'b1, 5'b10000, 1'b0, 1'b0, 3'b000, 1'b0, 1'b0, RD4};
        8'd244          : mc <= {MCTYPE3, 1'b1, 1'b1, 5'b10000, 1'b0, 1'b0, 3'b000, 1'b0, 1'b0, RD4};
        8'd245          : mc <= {MCTYPE3, 1'b1, 1'b1, 5'b10000, 1'b0, 1'b0, 3'b000, 1'b0, 1'b0, RD4};
        8'd246          : mc <= {MCTYPE3, 1'b1, 1'b1, 5'b10000, 1'b0, 1'b0, 3'b000, 1'b0, 1'b0, RD4};
        8'd247          : mc <= {MCTYPE3, 1'b1, 1'b1, 5'b10000, 1'b0, 1'b0, 3'b000, 1'b0, 1'b0, RD4};
        8'd248          : mc <= {MCTYPE3, 1'b1, 1'b1, 5'b10000, 1'b0, 1'b0, 3'b000, 1'b0, 1'b0, RD4};
        8'd249          : mc <= {MCTYPE3, 1'b1, 1'b1, 5'b10000, 1'b0, 1'b0, 3'b000, 1'b0, 1'b0, RD4};
        8'd250          : mc <= {MCTYPE3, 1'b1, 1'b1, 5'b10000, 1'b0, 1'b0, 3'b000, 1'b0, 1'b0, RD4};
        8'd251          : mc <= {MCTYPE3, 1'b1, 1'b1, 5'b10000, 1'b0, 1'b0, 3'b000, 1'b0, 1'b0, RD4};
        8'd252          : mc <= {MCTYPE3, 1'b1, 1'b1, 5'b10000, 1'b0, 1'b0, 3'b000, 1'b0, 1'b0, RD4};
        8'd253          : mc <= {MCTYPE3, 1'b1, 1'b1, 5'b10000, 1'b0, 1'b0, 3'b000, 1'b0, 1'b0, RD4};
        8'd254          : mc <= {MCTYPE3, 1'b1, 1'b1, 5'b10000, 1'b0, 1'b0, 3'b000, 1'b0, 1'b0, RD4};

        IRD             : mc <= {MCTYPE3, 1'b0, 1'b0, 5'b10000, 1'b0, 1'b0, 3'b000, 1'b0, 1'b1, RD4};   //wait for decoding
    endcase
end

endmodule