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
        //
        //  START ADDRESS 0: 2-CYCLE INSTRUCTION GROUP
        //
        //                       MCTYPE   FLAG  SKIP
        //immediate data load
        MVI_R_IM        : mc <= {MCTYPE3, 1'b0, 1'b0, 5'b10000, 1'b0, 1'b0, 3'b000, 1'b0, 1'b0, RD3}; //nop, RD3
        MVI_R_IM+1      : mc <= {MCTYPE0, 1'b1, 1'b1, SB_MD, SA_DST_R, 2'b00, RD4}; //r<-MD, RD4
        LXI_RP2_IM      : mc <= {MCTYPE3, 1'b0, 1'b0, 5'b10001, 1'b0, 1'b0, 3'b000, 1'b0, 1'b0, RD3}; //nop*2, RD3
        LXI_RP2_IM+1    : mc <= {MCTYPE0, 1'b1, 1'b1, SB_MD, SA_DST_RP2, 2'b00, RD4}; //rp2<-MD, RD4

        //rpa version of STAX, LDAX
        STAX_RPA_A      : mc <= {MCTYPE1, 1'b0, 1'b1, SD_RPA, SC_DST_MA, 4'b1010, WR3}; //MA<-RPA, WR3 //A will be loaded into MD automatically
        STAX_RPA_A+1    : mc <= {MCTYPE3, 1'b0, 1'b0, 5'b10000, 1'b0, 1'b0, 3'b000, 1'b0, 1'b0, RD4}; //nop, RD4
        LDAX_A_RPA      : mc <= {MCTYPE1, 1'b0, 1'b1, SD_RPA, SC_DST_MA, 4'b1010, RD3}; //MA<-RPA, WR3
        LDAX_A_RPA+1    : mc <= {MCTYPE0, 1'b1, 1'b0, SB_MD, SA_DST_A, 2'b00, RD4}; //A<-MD, RD4

        //INC or DEC, does not touch any flag
        INX_RP2         : mc <= {MCTYPE0, 1'b0, 1'b1, SB_ADD1, SA_DST_MA, 2'b01, IDLE};                 //MA<-MA+1, IDLE(stop PC increment)
        INX_RP2+1       : mc <= {MCTYPE0, 1'b0, 1'b0, SB_ADD1, SA_DST_RP2, 2'b01, RD4};                 //RP2<-RP2+1
        INX_EA          : mc <= {MCTYPE0, 1'b0, 1'b1, SB_ADD1, SA_DST_MA, 2'b01, IDLE};                 //MA<-MA+1, IDLE(stop PC increment)
        INX_EA+1        : mc <= {MCTYPE0, 1'b0, 1'b0, SB_ADD1, SA_DST_EA, 2'b01, RD4};                  //EA<-EA+1
        DCX_RP2         : mc <= {MCTYPE0, 1'b0, 1'b1, SB_ADD1, SA_DST_MA, 2'b01, IDLE};                 //MA<-MA+1, IDLE(stop PC increment)
        DCX_RP2+1       : mc <= {MCTYPE0, 1'b0, 1'b0, SB_SUB1, SA_DST_RP2, 2'b01, RD4};                 //RP2<-RP2-1
        DCX_EA          : mc <= {MCTYPE0, 1'b0, 1'b1, SB_ADD1, SA_DST_MA, 2'b01, IDLE};                 //MA<-MA+1, b0IDLE(stop PC increment)
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

        //HLT or STOP, CPU suspension control
        SUSP            : mc <= {MCTYPE3, 1'b0, 1'b1, 5'b10000, 1'b0, 1'b0, 3'b000, 1'b0, 1'b0, IDLE};  //nop, IDLE
        SUSP+1          : mc <= {MCTYPE3, 1'b0, 1'b0, 5'b10000, 1'b0, 1'b0, 3'b000, 1'b0, 1'b0, RD4};   //nop, IDLE

        //
        //  START ADDRESS 32: 4-CYCLE INSTRUCTION GROUP
        //
        //                       MCTYPE   FLAG  SKIP
        MOV_MEM_R       : mc <= {MCTYPE0, 1'b0, 1'b0, SB_R, SA_DST_MD, 2'b00, RD3};                     //MD<-R
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
        LD_RP_MEM       : mc <= {MCTYPE3, 1'b0, 1'b1, 5'b10001, 1'b0, 1'b0, 3'b000, 1'b0, 1'b0, RD3};   //nop*2, RD3
        LD_RP_MEM+1     : mc <= {MCTYPE0, 1'b0, 1'b0, SB_MD, SA_DST_MA, 2'b00, RD3};                    //MA<-MD, RD3
        LD_RP_MEM+2     : mc <= {MCTYPE3, 1'b0, 1'b0, 5'b10000, 1'b0, 1'b0, 3'b000, 1'b0, 1'b0, RD3};   //nop, RD3
        LD_RP_MEM+3     : mc <= {MCTYPE0, 1'b0, 1'b0, SB_MD, SA_DST_RP, 2'b00, RD4};                    //rp<-MD, RD4

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

        NOP             : mc <= {MCTYPE3, 1'b1, 1'b1, 5'b10000, 1'b0, 1'b0, 3'b000, 1'b0, 1'b0, RD4};   //nop, RD4

        IRD             : mc <= {MCTYPE3, 1'b0, 1'b0, 5'b10000, 1'b0, 1'b0, 3'b000, 1'b0, 1'b0, RD4};   //wait for decoding
        default         : mc <= {MCTYPE3, 1'b1, 1'b1, 5'b10000, 1'b0, 1'b0, 3'b000, 1'b0, 1'b0, RD4};   //nop, RD4
    endcase
end

endmodule